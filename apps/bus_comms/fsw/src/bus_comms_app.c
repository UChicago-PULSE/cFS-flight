/************************************************************************
 * Bus Communications App 
 ************************************************************************/

#include "bus_comms_app.h"
#include "bus_comms_events.h"
#include "bus_comms_version.h"
#include "bus_comms_table.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Add libcsp headers
#include <csp/csp.h>
#include <csp/csp_debug.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/drivers/can_socketcan.h>

// Add cFE time for timestamps in routing table
#include "cfe_time.h"
#include "cfe_psp.h"
#include "osapi.h"

extern int32 OS_Milli2Ticks(uint32 milli_seconds, int *ticks);

// CSP configuration (adjust as needed)
#define BUS_COMMS_CSP_CAN_IF     "vcan0"
#define BUS_COMMS_CSP_BITRATE    0
#define BUS_COMMS_CSP_MY_ADDR    1
#define BUS_COMMS_CSP_DEST_ADDR  2
#define BUS_COMMS_CSP_PORT       10

// Child task IDs
static CFE_ES_TaskId_t BUS_COMMS_CSP_RouterTaskId     = CFE_ES_TASKID_UNDEFINED;
static CFE_ES_TaskId_t BUS_COMMS_CSP_ReceiverTaskId   = CFE_ES_TASKID_UNDEFINED;
static CFE_ES_TaskId_t BUS_COMMS_CSP_TxTaskId         = CFE_ES_TASKID_UNDEFINED;
static CFE_ES_TaskId_t BUS_COMMS_CSP_CfdpRxTaskId     = CFE_ES_TASKID_UNDEFINED;

/* Dedicated CSP port for CFDP PDU traffic (uplink from ground station).
 * Must be <= CSP_PORT_MAX_BIND (16) and not conflict with port 10 (general BUS_COMMS). */
#define BUS_COMMS_CSP_CFDP_PORT  15

/* Forward declarations for child tasks */
static void BUS_COMMS_CSP_CfdpReceiverTask(void);

// New command codes (define locally if not provided by headers)
#ifndef BUS_COMMS_LIST_ROUTES_CC
#define BUS_COMMS_LIST_ROUTES_CC     0x11
#endif

// Limits for routing and payloads
#define BUS_COMMS_MAX_ROUTES          16
#define BUS_COMMS_MAX_SOURCE_MAPPINGS 16

// New routing table entry
typedef struct {
    uint8_t addr;
    uint16_t last_port;
    uint32_t rx_count;
    uint32_t tx_count;
    CFE_TIME_SysTime_t last_seen;
} bus_comms_route_entry_t;

// Routing table (module-local)
static bus_comms_route_entry_t g_routes[BUS_COMMS_MAX_ROUTES] = {0};
static uint8_t g_route_count = 0;

// Placeholder mapping between message sources and SBN pipes
typedef struct
{
    CFE_SB_MsgId_t SourceMsgId;
    uint16         SbnPublisherPipeId;
    uint16         SbnSubscriberPipeId;
    bool           InUse;
} BUS_COMMS_SourcePipeMapEntry_t;

static BUS_COMMS_SourcePipeMapEntry_t g_source_pipe_map[BUS_COMMS_MAX_SOURCE_MAPPINGS] = {0};
static bool g_missing_sbn_map_logged = false;

static const BUS_COMMS_SourcePipeMapEntry_t * BUS_COMMS_SourceToSbnMap(CFE_SB_MsgId_t msg_id);
static void BUS_COMMS_SourcePipeMapInit(void);

// Forward declarations for new helpers
static void BUS_COMMS_RouteUpdateRx(uint8_t addr, uint16_t port);
static void BUS_COMMS_RouteUpdateTx(uint8_t addr, uint16_t port);
static int  BUS_COMMS_CSP_Send(uint8_t dest, uint8_t port, const void * data, uint16_t len);
static void BUS_COMMS_CSP_TxTask(void);
static void BUS_COMMS_SelectNodeIds(void);

// Forward declarations
static void BUS_COMMS_CSP_RouterTask(void);
static void BUS_COMMS_CSP_ReceiverTask(void);

static uint8_t g_csp_my_addr   = BUS_COMMS_CSP_MY_ADDR;
static uint8_t g_csp_dest_addr = BUS_COMMS_CSP_DEST_ADDR;

BUS_COMMS_AppData_t BUS_COMMS_AppData;

void BUS_COMMS_AppMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /* Trace: entered AppMain */
    CFE_ES_WriteToSysLog("BUS_COMMS: AppMain entered\n");

    CFE_ES_PerfLogEntry(BUS_COMMS_APP_PERF_ID);

    status = BUS_COMMS_AppInit();
    if (status != CFE_SUCCESS)
    {
        BUS_COMMS_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        CFE_ES_WriteToSysLog("BUS_COMMS: AppInit failed RC=0x%08lX\n", (unsigned long)status);
    }
    else
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: AppInit OK\n");
    }

    while (CFE_ES_RunLoop(&BUS_COMMS_AppData.RunStatus) == true)
    {
        CFE_ES_PerfLogExit(BUS_COMMS_APP_PERF_ID);

        status = CFE_SB_ReceiveBuffer(&SBBufPtr, BUS_COMMS_AppData.CmdPipe, CFE_SB_PEND_FOREVER);

        CFE_ES_PerfLogEntry(BUS_COMMS_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            BUS_COMMS_ProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(BUS_COMMS_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "BUS_COMMS: SB Pipe Read Error, App Will Exit");
            BUS_COMMS_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    CFE_ES_PerfLogExit(BUS_COMMS_APP_PERF_ID);
    CFE_ES_ExitApp(BUS_COMMS_AppData.RunStatus);
}

int32 BUS_COMMS_AppInit(void)
{
    int32 status;

    /* Trace: starting AppInit */
    CFE_ES_WriteToSysLog("BUS_COMMS: AppInit starting\n");

    BUS_COMMS_AppData.RunStatus = CFE_ES_RunStatus_APP_RUN;
    BUS_COMMS_AppData.CmdCounter = 0;
    BUS_COMMS_AppData.ErrCounter = 0;

    BUS_COMMS_AppData.PipeDepth = BUS_COMMS_APP_PIPE_DEPTH;
    strncpy(BUS_COMMS_AppData.PipeName, "BUS_COMMS_CMD_PIPE", sizeof(BUS_COMMS_AppData.PipeName));
    BUS_COMMS_AppData.PipeName[sizeof(BUS_COMMS_AppData.PipeName) - 1] = 0;

    status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }
    else
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: EVS registered\n");
    }

    CFE_MSG_Init(CFE_MSG_PTR(BUS_COMMS_AppData.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(BUS_COMMS_HK_TLM_MID),
                 sizeof(BUS_COMMS_AppData.HkTlm));

    status = CFE_SB_CreatePipe(&BUS_COMMS_AppData.CmdPipe, BUS_COMMS_AppData.PipeDepth, BUS_COMMS_AppData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BUS_COMMS_SEND_HK_MID), BUS_COMMS_AppData.CmdPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BUS_COMMS_CMD_MID), BUS_COMMS_AppData.CmdPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error Subscribing to CMD, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    // Initialize routing table and source/SBN map
    memset(g_routes, 0, sizeof(g_routes));
    g_route_count = 0;
    BUS_COMMS_SourcePipeMapInit();

    BUS_COMMS_SelectNodeIds();

    // Initialize CSP and SocketCAN, spawn router, receiver, and periodic TX tasks
    do {
        int err;
        csp_iface_t *iface = NULL;

        csp_init();
        csp_dbg_packet_print = 1;

        err = csp_can_socketcan_open_and_add_interface(
            BUS_COMMS_CSP_CAN_IF,
            CSP_IF_CAN_DEFAULT_NAME,
            g_csp_my_addr,
            BUS_COMMS_CSP_BITRATE,
            false,
            &iface);
        if (err != CSP_ERR_NONE || iface == NULL) {
            CFE_ES_WriteToSysLog("BUS_COMMS: CSP SocketCAN can0 open failed err=%d\n", err);
            return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
        }
        iface->is_default = 1;
        

        // Router child task
        status = CFE_ES_CreateChildTask(
            &BUS_COMMS_CSP_RouterTaskId,
            "BC_CSP_ROUTER",
            BUS_COMMS_CSP_RouterTask,
            NULL,
            16384,   /* stack */
            60,       /* priority */
            0        /* CPU affinity */
        );
        if (status != CFE_SUCCESS) {
            CFE_ES_WriteToSysLog("BUS_COMMS: CreateChildTask Router failed RC=0x%08lX\n", (unsigned long)status);
            return status;
        }
        OS_TaskDelay(10);


        // Receiver child task
        status = CFE_ES_CreateChildTask(
            &BUS_COMMS_CSP_ReceiverTaskId,
            "BC_CSP_RX",
            BUS_COMMS_CSP_ReceiverTask,
            NULL,
            16384,
            50,
            0
        );
        if (status != CFE_SUCCESS) {
            CFE_ES_WriteToSysLog("BUS_COMMS: CreateChildTask Receiver failed RC=0x%08lX\n", (unsigned long)status);
            return status;
        }

        status = CFE_ES_CreateChildTask(
            &BUS_COMMS_CSP_TxTaskId,
            "BC_CSP_TX",
            BUS_COMMS_CSP_TxTask,
            NULL,
            16384,
            55,
            0
        );
        if (status != CFE_SUCCESS) {
            CFE_ES_WriteToSysLog("BUS_COMMS: CreateChildTask TX failed RC=0x%08lX\n", (unsigned long)status);
            return status;
        }

        // CFDP receiver child task (dedicated port 20 for uplink PDUs)
        status = CFE_ES_CreateChildTask(
            &BUS_COMMS_CSP_CfdpRxTaskId,
            "BC_CFDP_RX",
            BUS_COMMS_CSP_CfdpReceiverTask,
            NULL,
            16384,
            50,
            0
        );
        if (status != CFE_SUCCESS) {
            CFE_ES_WriteToSysLog("BUS_COMMS: CreateChildTask CFDP RX failed RC=0x%08lX\n", (unsigned long)status);
            return status;
        }

        
    } while (0);

    /* send a quick CSP ping on startup so external tools can verify connectivity */
    const char *startup_msg = "BUS_COMMS startup ping";

    if (BUS_COMMS_CSP_Send(g_csp_dest_addr, BUS_COMMS_CSP_PORT, startup_msg,
                            (uint16_t)strlen(startup_msg)) != 0)
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: startup CSP send failed\n");
    }

    CFE_EVS_SendEvent(BUS_COMMS_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "BUS_COMMS App Initialized. %s",
                      BUS_COMMS_VERSION_STRING);

    return CFE_SUCCESS;
}

void BUS_COMMS_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case BUS_COMMS_CMD_MID:
            BUS_COMMS_ProcessGroundCommand(SBBufPtr);
            break;
        case BUS_COMMS_SEND_HK_MID:
            BUS_COMMS_ReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;
        default:
            CFE_EVS_SendEvent(BUS_COMMS_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "BUS_COMMS: invalid command packet, MID = 0x%x",
                              (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }
}

void BUS_COMMS_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;
    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    switch (CommandCode)
    {
        case BUS_COMMS_NOOP_CC:
            BUS_COMMS_AppData.CmdCounter++;
            CFE_EVS_SendEvent(BUS_COMMS_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "BUS_COMMS: NOOP command %s", BUS_COMMS_VERSION_STRING);
            // Replace inline demo send with generic sender
            do {
                const char * msg = "CSP hello from BUS_COMMS";
                if (BUS_COMMS_CSP_Send(g_csp_dest_addr, BUS_COMMS_CSP_PORT, msg, (uint16_t)strlen(msg)) != 0) {
                    CFE_ES_WriteToSysLog("BUS_COMMS: NOOP demo send failed\n");
                }
            } while (0);
            break;

        case BUS_COMMS_RESET_COUNTERS_CC:
            BUS_COMMS_AppData.CmdCounter = 0;
            BUS_COMMS_AppData.ErrCounter = 0;
            CFE_EVS_SendEvent(BUS_COMMS_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "BUS_COMMS: RESET command");
            break;

        case BUS_COMMS_SEND_CSP_CC: {
            // Generic send command: dest, port, len, data[]
            size_t total_size = 0;
            CFE_MSG_GetSize(&SBBufPtr->Msg, &total_size);
            if (total_size < sizeof(CFE_MSG_CommandHeader_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t)) {
                CFE_EVS_SendEvent(BUS_COMMS_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "BUS_COMMS: SEND_CSP invalid size (%lu)", (unsigned long) total_size);
                BUS_COMMS_AppData.ErrCounter++;
                break;
            }

            BUS_COMMS_SendCspCmd_t * cmd = (BUS_COMMS_SendCspCmd_t *) SBBufPtr;
            uint16_t len = cmd->len;
            size_t min_size = sizeof(CFE_MSG_CommandHeader_t) + 1 + 1 + 2 + len;
            if (len > BUS_COMMS_MAX_SEND_LEN || total_size < min_size) {
                CFE_EVS_SendEvent(BUS_COMMS_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "BUS_COMMS: SEND_CSP bad len=%u total=%lu", (unsigned)len, (unsigned long)total_size);
                BUS_COMMS_AppData.ErrCounter++;
                break;
            }

            if (BUS_COMMS_CSP_Send(cmd->dest, cmd->port, cmd->data, len) == 0) {
                BUS_COMMS_AppData.CmdCounter++;
                CFE_EVS_SendEvent(BUS_COMMS_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                                  "BUS_COMMS: SENT CSP to %u:%u len=%u", cmd->dest, cmd->port, len);
            } else {
                BUS_COMMS_AppData.ErrCounter++;
            }
            break;
        }

        case BUS_COMMS_LIST_ROUTES_CC: {
            // Dump routing table to syslog
            CFE_ES_WriteToSysLog("BUS_COMMS: Routing table entries: %u\n", g_route_count);
            for (uint8_t i = 0; i < g_route_count; ++i) {
                CFE_ES_WriteToSysLog("BUS_COMMS: Route[%u]: addr=%u last_port=%u rx=%lu tx=%lu\n",
                                     (unsigned)i,
                                     g_routes[i].addr,
                                     (unsigned)g_routes[i].last_port,
                                     (unsigned long)g_routes[i].rx_count,
                                     (unsigned long)g_routes[i].tx_count);
            }
            BUS_COMMS_AppData.CmdCounter++;
            break;
        }

        default:
            CFE_EVS_SendEvent(BUS_COMMS_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "BUS_COMMS: Invalid ground command code: CC = %d", CommandCode);
            BUS_COMMS_AppData.ErrCounter++;
            break;
    }
}

int32 BUS_COMMS_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{
    BUS_COMMS_AppData.HkTlm.Payload.CommandErrorCounter = BUS_COMMS_AppData.ErrCounter;
    BUS_COMMS_AppData.HkTlm.Payload.CommandCounter      = BUS_COMMS_AppData.CmdCounter;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(BUS_COMMS_AppData.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(BUS_COMMS_AppData.HkTlm.TelemetryHeader), true);

    return CFE_SUCCESS;
}

// Child task: CSP router
static void BUS_COMMS_CSP_RouterTask(void)
{
    for (;;)
    {
        csp_route_work();
    }
    CFE_ES_ExitChildTask();
}

// Child task: CSP receiver (like your receiver.c)
static void BUS_COMMS_CSP_ReceiverTask(void)
{
    csp_socket_t sock;
    memset(&sock, 0, sizeof(sock));

    csp_bind(&sock, BUS_COMMS_CSP_PORT);
    csp_listen(&sock, 5);

    for (;;)
    {
        csp_conn_t *conn = csp_accept(&sock, 1000);
        if (conn == NULL)
            continue;

        csp_packet_t *packet = csp_read(conn, 1000);
        if (packet) {
            uint8_t  src   = csp_conn_src(conn);
            uint16_t dport = csp_conn_dport(conn);
            BUS_COMMS_RouteUpdateRx(src, dport);

            CFE_ES_WriteToSysLog("BUS_COMMS: CSP RX from %u:%u: %.*s\n",
                                 src, dport, (int)packet->length, (char *)packet->data);
            csp_buffer_free(packet);
        }

        csp_close(conn);
    }

    CFE_ES_ExitChildTask();
}


// Child task: dedicated CFDP CSP receiver on port 20
// Receives CSP packets from ground station (via radio-mock) and publishes them to the SB
static void BUS_COMMS_CSP_CfdpReceiverTask(void)
{
    csp_socket_t sock;
    memset(&sock, 0, sizeof(sock));

    csp_bind(&sock, BUS_COMMS_CSP_CFDP_PORT);
    csp_listen(&sock, 10);

    CFE_ES_WriteToSysLog("BUS_COMMS: CFDP RX task listening on CSP port %d\n",
                         BUS_COMMS_CSP_CFDP_PORT);

    for (;;)
    {
        csp_conn_t *conn = csp_accept(&sock, 1000);
        if (conn == NULL)
            continue;

        csp_packet_t *packet = csp_read(conn, 1000);
        if (packet)
        {
            uint8_t  src   = csp_conn_src(conn);
            uint16_t dport = csp_conn_dport(conn);

            if (packet->length > 0 && packet->length <= BUS_COMMS_MAX_SEND_LEN)
            {
                BUS_COMMS_CspRxData_t rxMsg;
                memset(&rxMsg, 0, sizeof(rxMsg));

                CFE_MSG_Init(CFE_MSG_PTR(rxMsg.TlmHdr),
                             CFE_SB_ValueToMsgId(BUS_COMMS_CSP_RX_DATA_MID),
                             sizeof(BUS_COMMS_CspRxData_t));
                CFE_MSG_SetMsgTime(CFE_MSG_PTR(rxMsg.TlmHdr), CFE_TIME_GetTime());

                rxMsg.src_addr = src;
                rxMsg.src_port = (uint8_t)dport;
                rxMsg.data_len = packet->length;
                memcpy(rxMsg.data, packet->data, packet->length);

                CFE_SB_TransmitMsg(CFE_MSG_PTR(rxMsg.TlmHdr), true);

                CFE_ES_WriteToSysLog("BUS_COMMS: CFDP RX from %u:%u len=%u -> SB\n",
                                     src, dport, (unsigned)packet->length);
            }
            else
            {
                CFE_ES_WriteToSysLog("BUS_COMMS: CFDP RX from %u:%u dropped (len=%u)\n",
                                     src, dport, (unsigned)packet->length);
            }

            BUS_COMMS_RouteUpdateRx(src, dport);
            csp_buffer_free(packet);
        }

        csp_close(conn);
    }

    CFE_ES_ExitChildTask();
}

// Child task: periodic CSP transmitter
static void BUS_COMMS_CSP_TxTask(void)
{
    const char payload[] = "BUS_COMMS periodic ping";
    int        delay_ticks = 0;

    while (1)
    {
        if (BUS_COMMS_CSP_Send(g_csp_dest_addr, BUS_COMMS_CSP_PORT, payload, (uint16_t)(sizeof(payload) - 1)) != 0)
        {
            CFE_ES_WriteToSysLog("BUS_COMMS: periodic CSP send failed\n");
        }

        if (OS_Milli2Ticks(25000, &delay_ticks) != OS_SUCCESS || delay_ticks <= 0)
        {
            delay_ticks = 25000;
        }

        OS_TaskDelay((uint32)delay_ticks);
    }

    CFE_ES_ExitChildTask();
}


static int BUS_COMMS_RouteFindOrAdd(uint8_t addr) {
    for (uint8_t i = 0; i < g_route_count; ++i) {
        if (g_routes[i].addr == addr) {
            return i;
        }
    }
    if (g_route_count < BUS_COMMS_MAX_ROUTES) {
        g_routes[g_route_count].addr = addr;
        g_routes[g_route_count].last_port = 0;
        g_routes[g_route_count].rx_count = 0;
        g_routes[g_route_count].tx_count = 0;
        g_routes[g_route_count].last_seen = CFE_TIME_GetTime();
        return g_route_count++;
    }
    return -1;
}

static void BUS_COMMS_RouteUpdateRx(uint8_t addr, uint16_t port) {
    int idx = BUS_COMMS_RouteFindOrAdd(addr);
    if (idx >= 0) {
        g_routes[idx].rx_count++;
        g_routes[idx].last_port = port;
        g_routes[idx].last_seen = CFE_TIME_GetTime();
    }
}

static void BUS_COMMS_RouteUpdateTx(uint8_t addr, uint16_t port) {
    int idx = BUS_COMMS_RouteFindOrAdd(addr);
    if (idx >= 0) {
        g_routes[idx].tx_count++;
        g_routes[idx].last_port = port;
        g_routes[idx].last_seen = CFE_TIME_GetTime();
    }
}

// Generic CSP sender with accounting
static int BUS_COMMS_CSP_Send(uint8_t dest, uint8_t port, const void * data, uint16_t len) {
    if (len == 0 || data == NULL) {
        CFE_ES_WriteToSysLog("BUS_COMMS: CSP send invalid params\n");
        return -1;
    }

    // Placeholder to demonstrate how a source/SBN mapping lookup could be incorporated later on.
    // Today we use the message ID associated with the command pipe since actual mappings are TBD.
    const BUS_COMMS_SourcePipeMapEntry_t *map_entry = BUS_COMMS_SourceToSbnMap(CFE_SB_ValueToMsgId(BUS_COMMS_CMD_MID));
    if (map_entry == NULL && !g_missing_sbn_map_logged) {
        g_missing_sbn_map_logged = true;
        CFE_ES_WriteToSysLog("BUS_COMMS: no SBN map entry for command MID\n");
    }

    csp_conn_t *conn = csp_connect(CSP_PRIO_NORM, dest, port, 1000, CSP_O_NONE);
    if (conn == NULL) {
        CFE_ES_WriteToSysLog("BUS_COMMS: csp_connect failed (dest=%u,port=%u)\n", dest, port);
        return -1;
    }

    csp_packet_t *packet = csp_buffer_get(len);
    if (packet == NULL) {
        CFE_ES_WriteToSysLog("BUS_COMMS: buffer get failed len=%u\n", len);
        csp_close(conn);
        return -1;
    }

    memcpy(packet->data, data, len);
    packet->length = len;

    // libcsp in your build returns void from csp_send; it takes ownership of 'packet'
    csp_send(conn, packet);

    BUS_COMMS_RouteUpdateTx(dest, port);
    csp_close(conn);
    return 0;
}

// ------------------ Source/SBN mapping helpers ------------------

static void BUS_COMMS_SelectNodeIds(void)
{
    uint32 cpu_id = CFE_PSP_GetProcessorId();

    switch (cpu_id)
    {
        case 1:
            g_csp_my_addr   = 1;
            g_csp_dest_addr = 2;
            break;

        case 2:
            g_csp_my_addr   = 2;
            g_csp_dest_addr = 1;
            break;

        default:
            g_csp_my_addr   = BUS_COMMS_CSP_MY_ADDR;
            g_csp_dest_addr = BUS_COMMS_CSP_DEST_ADDR;
            break;
    }

    CFE_ES_WriteToSysLog("BUS_COMMS: CPU=%lu my=%u dest=%u\n",
                         (unsigned long)cpu_id,
                         (unsigned)g_csp_my_addr,
                         (unsigned)g_csp_dest_addr);
}

static void BUS_COMMS_SourcePipeMapInit(void)
{
    memset(g_source_pipe_map, 0, sizeof(g_source_pipe_map));

    // Default entry demonstrating how mappings can be pre-seeded.
    g_source_pipe_map[0].SourceMsgId          = CFE_SB_ValueToMsgId(BUS_COMMS_CMD_MID);
    g_source_pipe_map[0].SbnPublisherPipeId   = 0; /* placeholder */
    g_source_pipe_map[0].SbnSubscriberPipeId  = 0; /* placeholder */
    g_source_pipe_map[0].InUse                = true;
}

static const BUS_COMMS_SourcePipeMapEntry_t * BUS_COMMS_SourceToSbnMap(CFE_SB_MsgId_t msg_id)
{
    for (size_t i = 0; i < BUS_COMMS_MAX_SOURCE_MAPPINGS; ++i)
    {
        if (g_source_pipe_map[i].InUse && CFE_SB_MsgId_Equal(g_source_pipe_map[i].SourceMsgId, msg_id))
        {
            return &g_source_pipe_map[i];
        }
    }

    return NULL;
}
