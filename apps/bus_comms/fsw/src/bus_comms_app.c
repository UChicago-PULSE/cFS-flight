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

#include <csp/csp.h>
#include <csp/csp_debug.h>
#include <csp/csp_rtable.h>
#include <csp/interfaces/csp_if_can.h>
#include <csp/drivers/can_socketcan.h>
#include <csp/drivers/usart.h>

#include "cfe_time.h"
#include "cfe_psp.h"
#include "cfe_tbl.h"
#include "osapi.h"

extern int32 OS_Milli2Ticks(uint32 milli_seconds, int *ticks);

/* CSP configuration */
#define BUS_COMMS_CSP_CAN_IF     "can1"
#define BUS_COMMS_CSP_BITRATE    0
#define BUS_COMMS_CSP_RS422_IF   "RS422"
#define BUS_COMMS_CSP_RS422_DEV  "/dev/ttyUSB0"
#define BUS_COMMS_CSP_RS422_BAUD 115200

#define BUS_COMMS_CSP_MY_ADDR    1
#define BUS_COMMS_CSP_DEST_ADDR  2
#define BUS_COMMS_CSP_PORT       10

/* Request queue configuration */
#define BUS_COMMS_REQUEST_QUEUE_DEPTH  16

/* Child task IDs */
static CFE_ES_TaskId_t BUS_COMMS_CSP_RouterTaskId   = CFE_ES_TASKID_UNDEFINED;
static CFE_ES_TaskId_t BUS_COMMS_CSP_ReceiverTaskId = CFE_ES_TASKID_UNDEFINED;
static CFE_ES_TaskId_t BUS_COMMS_SBN_RxTaskId       = CFE_ES_TASKID_UNDEFINED;
static CFE_ES_TaskId_t BUS_COMMS_CSP_TxTaskId       = CFE_ES_TASKID_UNDEFINED;

/* Command codes */
#ifndef BUS_COMMS_SEND_CSP_CC
#define BUS_COMMS_SEND_CSP_CC        0x10
#endif
#ifndef BUS_COMMS_LIST_ROUTES_CC
#define BUS_COMMS_LIST_ROUTES_CC     0x11
#endif
#ifndef BUS_COMMS_TEST_CAN_CC
#define BUS_COMMS_TEST_CAN_CC        0x12   /* Test: send via SBN pipe like other apps would */
#endif

/* Limits */
#define BUS_COMMS_MAX_ROUTES          16
#define BUS_COMMS_MAX_SEND_LEN        220

/* Routing table entry */
typedef struct {
    uint8_t addr;
    uint16_t last_port;
    uint32_t rx_count;
    uint32_t tx_count;
    CFE_TIME_SysTime_t last_seen;
} bus_comms_route_entry_t;

static bus_comms_route_entry_t g_routes[BUS_COMMS_MAX_ROUTES] = {0};
static uint8_t g_route_count = 0;

/*
 * Request queue - FIFO buffer for CAN requests from other apps.
 * Protected by mutex for thread safety between SBN RX task and CSP TX processing.
 */
typedef struct {
    BUS_COMMS_CanRequest_t entries[BUS_COMMS_REQUEST_QUEUE_DEPTH];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    osal_id_t mutex;
} BUS_COMMS_RequestQueue_t;

static BUS_COMMS_RequestQueue_t g_request_queue;

/* SBN pipe for receiving CAN requests from other apps */
static CFE_SB_PipeId_t g_sbn_request_pipe;

/* Telemetry counters */
static uint16_t g_can_tx_count = 0;
static uint16_t g_can_rx_count = 0;
static uint16_t g_sbn_request_count = 0;
static uint16_t g_sbn_response_count = 0;

/* Generic SEND_CSP command payload (for ground commands) */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHdr;
    uint8_t  dest;
    uint8_t  port;
    uint16_t len;
    uint8_t  data[BUS_COMMS_MAX_SEND_LEN];
} BUS_COMMS_SendCspCmd_t;

/* Forward declarations */
static void BUS_COMMS_CSP_RouterTask(void);
static void BUS_COMMS_CSP_ReceiverTask(void);
static void BUS_COMMS_SBN_RxTask(void);
static void BUS_COMMS_CSP_TxTask(void);
static void BUS_COMMS_RouteUpdateRx(uint8_t addr, uint16_t port);
static void BUS_COMMS_RouteUpdateTx(uint8_t addr, uint16_t port);
static int  BUS_COMMS_CSP_Send(uint8_t dest, uint8_t port, const void *data, uint16_t len);
static void BUS_COMMS_SelectNodeIds(void);
static void BUS_COMMS_RequestQueueInit(void);
static int  BUS_COMMS_RequestQueuePush(const BUS_COMMS_CanRequest_t *req);
static int  BUS_COMMS_RequestQueuePop(BUS_COMMS_CanRequest_t *req);
static void BUS_COMMS_PublishResponse(uint8_t subsystem, uint16_t sequence,
                                       uint8_t src_node, uint8_t src_port,
                                       uint8_t error_code,
                                       const uint8_t *data, uint16_t data_len);
static void BUS_COMMS_ProcessCanRequest(const BUS_COMMS_CanRequest_t *req);

static uint8_t g_csp_my_addr   = BUS_COMMS_CSP_MY_ADDR;
static uint8_t g_csp_dest_addr = BUS_COMMS_CSP_DEST_ADDR;

BUS_COMMS_AppData_t BUS_COMMS_AppData;

void BUS_COMMS_AppMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

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

    /* Main loop processes ground commands and HK requests on the command pipe.
     * SBN requests from other apps are handled by the SBN RX child task. */
    while (CFE_ES_RunLoop(&BUS_COMMS_AppData.RunStatus) == true)
    {
        CFE_ES_PerfLogExit(BUS_COMMS_APP_PERF_ID);

        /* Block waiting for the next msg */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, BUS_COMMS_AppData.CmdPipe, CFE_SB_PEND_FOREVER);

        CFE_ES_PerfLogEntry(BUS_COMMS_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            BUS_COMMS_ProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_ES_WriteToSysLog("BUS_COMMS: Error reading cmd pipe, RC = 0x%08lX\n", (unsigned long)status);
            BUS_COMMS_AppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        /* Let cFE manage the table (allows ground updates) */
        CFE_TBL_ReleaseAddress(BUS_COMMS_AppData.TblHandle);
        CFE_TBL_Manage(BUS_COMMS_AppData.TblHandle);
        CFE_TBL_GetAddress((void **)&BUS_COMMS_AppData.TblPtr, BUS_COMMS_AppData.TblHandle);
    }

    CFE_ES_ExitApp(BUS_COMMS_AppData.RunStatus);
}

int32 BUS_COMMS_AppInit(void)
{
    int32 status;

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
    CFE_ES_WriteToSysLog("BUS_COMMS: EVS registered\n");

    CFE_MSG_Init(CFE_MSG_PTR(BUS_COMMS_AppData.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(BUS_COMMS_HK_TLM_MID),
                 sizeof(BUS_COMMS_AppData.HkTlm));

    /* Register and load the Configuration Table */
    status = CFE_TBL_Register(&BUS_COMMS_AppData.TblHandle, "ConfigTbl",
                              sizeof(BUS_COMMS_Table_t), CFE_TBL_OPT_DEFAULT, NULL);
    if (status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error registering table, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    status = CFE_TBL_Load(BUS_COMMS_AppData.TblHandle, CFE_TBL_SRC_FILE, BUS_COMMS_TABLE_FILE);
    if (status != CFE_SUCCESS && status != CFE_TBL_INFO_UPDATED) {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error loading table, RC = 0x%08lX\n", (unsigned long)status);
        /* Non-critical error, we'll just not have table data initially if file is missing */
    }

    status = CFE_TBL_GetAddress((void **)&BUS_COMMS_AppData.TblPtr, BUS_COMMS_AppData.TblHandle);
    if (status != CFE_SUCCESS && status != CFE_TBL_INFO_UPDATED) {
        /* Fallback if table not completely loaded */
        BUS_COMMS_AppData.TblPtr = NULL;
    }

    /* Create command pipe for ground commands and HK requests */
    status = CFE_SB_CreatePipe(&BUS_COMMS_AppData.CmdPipe, BUS_COMMS_AppData.PipeDepth, BUS_COMMS_AppData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error creating cmd pipe, RC = 0x%08lX\n", (unsigned long)status);
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

    /*
     * Create a separate pipe for CAN requests from other apps (GPS, ADCS, Radio).
     * This keeps high-volume inter-app traffic separate from ground commands.
     */
    status = CFE_SB_CreatePipe(&g_sbn_request_pipe, BUS_COMMS_REQUEST_QUEUE_DEPTH, "BUS_COMMS_SBN_PIPE");
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error creating SBN pipe, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BUS_COMMS_CAN_REQUEST_MID), g_sbn_request_pipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("BUS_COMMS: Error subscribing to CAN_REQUEST, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /* Initialize the request queue and routing table */
    BUS_COMMS_RequestQueueInit();
    memset(g_routes, 0, sizeof(g_routes));
    g_route_count = 0;

    BUS_COMMS_SelectNodeIds();

    /* Initialize CSP and SocketCAN */
    do {
        int err;
        csp_iface_t *iface = NULL;
        csp_iface_t *usart_iface = NULL;

        /* Use CSP v1 protocol framing (5-bit address, 32-bit header) */
        csp_conf.version = 1;

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
            CFE_ES_WriteToSysLog("BUS_COMMS: CSP SocketCAN open failed err=%d\n", err);
            return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
        }
        iface->is_default = 1;

        /* Setup RS422/USART interface via KISS */
        csp_usart_conf_t usart_conf = {
            .device = BUS_COMMS_CSP_RS422_DEV,
            .baudrate = BUS_COMMS_CSP_RS422_BAUD,
            .databits = 8,
            .stopbits = 1,
            .paritysetting = 0,
        };
        err = csp_usart_open_and_add_kiss_interface(
            &usart_conf,
            "KISS",
            g_csp_my_addr,
            &usart_iface);
        if (err != CSP_ERR_NONE || usart_iface == NULL) {
            CFE_ES_WriteToSysLog("BUS_COMMS: CSP RS422 open failed err=%d\n", err);
            // Non-critical: continue anyway or handle failure as needed
        } else {
            /* Example: explicitly route a destination node (e.g. node 3) via RS422 */
            /* In a real app, adjust routing tables accordingly if using multiple interfaces. */
            csp_rtable_set(3, 0, usart_iface, CSP_NO_VIA_ADDRESS);
        }

        /* CSP Router task - handles internal CSP routing */
        status = CFE_ES_CreateChildTask(
            &BUS_COMMS_CSP_RouterTaskId,
            "BC_CSP_ROUTER",
            BUS_COMMS_CSP_RouterTask,
            NULL,
            16384,
            60,
            0
        );
        if (status != CFE_SUCCESS) {
            CFE_ES_WriteToSysLog("BUS_COMMS: CreateChildTask Router failed RC=0x%08lX\n", (unsigned long)status);
            return status;
        }
        OS_TaskDelay(10);

        /* CSP Receiver task - receives from CAN and publishes responses to SBN */
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
            CFE_ES_WriteToSysLog("BUS_COMMS: CreateChildTask CSP RX failed RC=0x%08lX\n", (unsigned long)status);
            return status;
        }

        /* SBN Receiver task - receives requests from other apps and queues them for CAN TX */
        status = CFE_ES_CreateChildTask(
            &BUS_COMMS_SBN_RxTaskId,
            "BC_SBN_RX",
            BUS_COMMS_SBN_RxTask,
            NULL,
            16384,
            55,
            0
        );
        if (status != CFE_SUCCESS) {
            CFE_ES_WriteToSysLog("BUS_COMMS: CreateChildTask SBN RX failed RC=0x%08lX\n", (unsigned long)status);
            return status;
        }

        /* TX child task - periodic heartbeat processing based on table */
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

    } while (0);

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

        case BUS_COMMS_TEST_CAN_CC: {
            /* Test command: publish a CAN request message on the SBN pipe
             * This simulates what GPS/ADCS/Radio apps would do */
            BUS_COMMS_TestCanCmd_t *cmd = (BUS_COMMS_TestCanCmd_t *)SBBufPtr;
            
            /* Build a CAN request message and publish it */
            BUS_COMMS_CanRequest_t req;
            CFE_MSG_Init(CFE_MSG_PTR(req.Header), CFE_SB_ValueToMsgId(BUS_COMMS_CAN_REQUEST_MID),
                         sizeof(BUS_COMMS_CanRequest_t));
            
            req.direction = BUS_COMMS_DIR_REQUEST;
            req.subsystem = BUS_COMMS_SUBSYS_GROUND;  /* From ground test */
            req.dest_node = cmd->dest_node;
            req.dest_port = cmd->dest_port;
            req.sequence  = (uint16_t)(BUS_COMMS_AppData.CmdCounter & 0xFFFF);
            req.data_len  = cmd->data_len;
            if (req.data_len > BUS_COMMS_CAN_MAX_DATA_LEN) {
                req.data_len = BUS_COMMS_CAN_MAX_DATA_LEN;
            }
            memcpy(req.data, cmd->data, req.data_len);
            
            /* Publish to the SBN - our SBN RX task will pick it up and process it */
            CFE_SB_TransmitMsg(CFE_MSG_PTR(req.Header), true);
            
            CFE_EVS_SendEvent(BUS_COMMS_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "BUS_COMMS: TEST_CAN published request to node=%u port=%u len=%u",
                              cmd->dest_node, cmd->dest_port, req.data_len);
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
    BUS_COMMS_AppData.HkTlm.Payload.CommandCounter      = BUS_COMMS_AppData.CmdCounter;
    BUS_COMMS_AppData.HkTlm.Payload.CommandErrorCounter = BUS_COMMS_AppData.ErrCounter;
    BUS_COMMS_AppData.HkTlm.Payload.CanTxCount          = g_can_tx_count;
    BUS_COMMS_AppData.HkTlm.Payload.CanRxCount          = g_can_rx_count;
    BUS_COMMS_AppData.HkTlm.Payload.SbnRequestCount     = g_sbn_request_count;
    BUS_COMMS_AppData.HkTlm.Payload.SbnResponseCount    = g_sbn_response_count;
    BUS_COMMS_AppData.HkTlm.Payload.QueueDepth          = g_request_queue.count;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(BUS_COMMS_AppData.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(BUS_COMMS_AppData.HkTlm.TelemetryHeader), true);

    return CFE_SUCCESS;
}

/* CSP Router task - internal CSP routing */
static void BUS_COMMS_CSP_RouterTask(void)
{
    for (;;)
    {
        csp_route_work();
    }
    CFE_ES_ExitChildTask();
}

/*
 * CSP Receiver task - receives data from CAN bus and publishes to SBN.
 * Hardware responses come in here and get forwarded to requesting apps.
 */
static void BUS_COMMS_CSP_ReceiverTask(void)
{
    csp_socket_t sock;
    memset(&sock, 0, sizeof(sock));

    csp_bind(&sock, CSP_ANY);
    csp_listen(&sock, 10);

    for (;;)
    {
        csp_conn_t *conn = csp_accept(&sock, 1000);
        if (conn == NULL)
            continue;

        csp_packet_t *packet = csp_read(conn, 1000);
        if (packet) {
            uint8_t  src_node = csp_conn_src(conn);
            uint16_t src_port = csp_conn_sport(conn);

            BUS_COMMS_RouteUpdateRx(src_node, src_port);
            g_can_rx_count++;

            CFE_ES_WriteToSysLog("BUS_COMMS: CAN RX from node %u port %u len %u\n",
                                 src_node, src_port, packet->length);

            /*
             * Publish this response on the SBN for subscribing apps.
             * For now we use SUBSYS_UNKNOWN and sequence 0 since we don't have
             * request correlation yet. A more complete implementation would
             * track pending requests and match responses.
             */
            BUS_COMMS_PublishResponse(
                BUS_COMMS_SUBSYS_UNKNOWN,
                0,
                src_node,
                (uint8_t)src_port,
                BUS_COMMS_ERR_NONE,
                packet->data,
                packet->length
            );

            csp_buffer_free(packet);
        }

        csp_close(conn);
    }

    CFE_ES_ExitChildTask();
}

/*
 * SBN Receiver task - receives CAN requests from other apps via software bus.
 * Processes them FIFO by sending to CAN and publishing responses.
 */
static void BUS_COMMS_SBN_RxTask(void)
{
    int32            status;
    CFE_SB_Buffer_t *buf_ptr;

    CFE_ES_WriteToSysLog("BUS_COMMS: SBN RX task started\n");

    for (;;)
    {
        status = CFE_SB_ReceiveBuffer(&buf_ptr, g_sbn_request_pipe, 100);

        if (status == CFE_SUCCESS)
        {
            CFE_SB_MsgId_t msg_id = CFE_SB_INVALID_MSG_ID;
            CFE_MSG_GetMsgId(&buf_ptr->Msg, &msg_id);

            if (CFE_SB_MsgIdToValue(msg_id) == BUS_COMMS_CAN_REQUEST_MID)
            {
                BUS_COMMS_CanRequest_t *req = (BUS_COMMS_CanRequest_t *)buf_ptr;

                g_sbn_request_count++;

                CFE_ES_WriteToSysLog("BUS_COMMS: SBN request from subsys %u to node %u port %u len %u\n",
                                     req->subsystem, req->dest_node, req->dest_port, req->data_len);

                /* Process the request immediately (FIFO - this task handles them in order) */
                BUS_COMMS_ProcessCanRequest(req);
            }
        }
        else if (status != CFE_SB_TIME_OUT && status != CFE_SB_NO_MESSAGE)
        {
            CFE_ES_WriteToSysLog("BUS_COMMS: SBN pipe error 0x%08lX\n", (unsigned long)status);
        }
    }

    CFE_ES_ExitChildTask();
}

/*
 * CSP periodic transmitter - pulls from Configuration Table to determine 
 * if a heartbeat ping should be sent and how often.
 */
static void BUS_COMMS_CSP_TxTask(void)
{
    const char payload[] = "BUS_COMMS periodic ping";
    int        delay_ticks = 0;

    CFE_ES_WriteToSysLog("BUS_COMMS: TX task started\n");

    while (1)
    {
        /* Lock the table pointers automatically or read values defensively. 
         * Since TblPtr is updated by AppMain, we just check if it's safe. */
        bool heartbeat_enabled = false;
        uint32_t interval_ms = 25000;

        if (BUS_COMMS_AppData.TblPtr != NULL) {
            heartbeat_enabled = (BUS_COMMS_AppData.TblPtr->HeartbeatEnabled > 0);
            interval_ms = BUS_COMMS_AppData.TblPtr->HeartbeatInterval_ms;
        }

        if (heartbeat_enabled)
        {
            if (BUS_COMMS_CSP_Send(g_csp_dest_addr, BUS_COMMS_CSP_PORT, payload, (uint16_t)(sizeof(payload) - 1)) != 0)
            {
                CFE_ES_WriteToSysLog("BUS_COMMS: periodic CSP send failed\n");
            }
        }

        if (interval_ms < 1000) {
            interval_ms = 1000; /* safeguard to not flood/starve the CPU */
        }

        if (OS_Milli2Ticks(interval_ms, &delay_ticks) != OS_SUCCESS || delay_ticks <= 0)
        {
            delay_ticks = 25000;
        }

        OS_TaskDelay((uint32)delay_ticks);
    }

    CFE_ES_ExitChildTask();
}

/*
 * Process a CAN request and send data over CAN to the destination node.
 * Publishes a response (success or error) back to the requesting app.
 */
static void BUS_COMMS_ProcessCanRequest(const BUS_COMMS_CanRequest_t *req)
{
    uint8_t error_code = BUS_COMMS_ERR_NONE;

    if (req->data_len > BUS_COMMS_CAN_MAX_DATA_LEN) {
        error_code = BUS_COMMS_ERR_PAYLOAD_SIZE;
        CFE_ES_WriteToSysLog("BUS_COMMS: request payload too large %u\n", req->data_len);
    }
    else if (req->dest_node == 0) {
        error_code = BUS_COMMS_ERR_INVALID_DEST;
        CFE_ES_WriteToSysLog("BUS_COMMS: invalid dest node 0\n");
    }
    else {
        int rc = BUS_COMMS_CSP_Send(req->dest_node, req->dest_port, req->data, req->data_len);
        if (rc != 0) {
            error_code = BUS_COMMS_ERR_CAN_TX_FAIL;
        }
    }

    /*
     * Publish an acknowledgment response. For actual hardware responses,
     * the CSP RX task will publish them when they arrive.
     */
    BUS_COMMS_PublishResponse(
        req->subsystem,
        req->sequence,
        req->dest_node,
        req->dest_port,
        error_code,
        NULL,
        0
    );
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

/* Send data over CSP/CAN to a destination node */
static int BUS_COMMS_CSP_Send(uint8_t dest, uint8_t port, const void *data, uint16_t len) {
    if (len == 0 || data == NULL) {
        CFE_ES_WriteToSysLog("BUS_COMMS: CSP send invalid params\n");
        return -1;
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

    csp_send(conn, packet);

    BUS_COMMS_RouteUpdateTx(dest, port);
    g_can_tx_count++;
    csp_close(conn);
    return 0;
}

/* Select CSP node IDs based on CPU ID */
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

    CFE_ES_WriteToSysLog("BUS_COMMS: CPU=%lu my_addr=%u dest_addr=%u\n",
                         (unsigned long)cpu_id,
                         (unsigned)g_csp_my_addr,
                         (unsigned)g_csp_dest_addr);
}

/* Initialize the request queue */
static void BUS_COMMS_RequestQueueInit(void)
{
    memset(&g_request_queue, 0, sizeof(g_request_queue));
    g_request_queue.head = 0;
    g_request_queue.tail = 0;
    g_request_queue.count = 0;

    int32 status = OS_MutSemCreate(&g_request_queue.mutex, "BC_Q_MTX", 0);
    if (status != OS_SUCCESS) {
        CFE_ES_WriteToSysLog("BUS_COMMS: Failed to create queue mutex\n");
    }
}

/* Push a request onto the queue (available for future async processing) */
__attribute__((unused))
static int BUS_COMMS_RequestQueuePush(const BUS_COMMS_CanRequest_t *req)
{
    int rc = -1;

    OS_MutSemTake(g_request_queue.mutex);

    if (g_request_queue.count < BUS_COMMS_REQUEST_QUEUE_DEPTH) {
        memcpy(&g_request_queue.entries[g_request_queue.tail], req, sizeof(*req));
        g_request_queue.tail = (g_request_queue.tail + 1) % BUS_COMMS_REQUEST_QUEUE_DEPTH;
        g_request_queue.count++;
        rc = 0;
    }

    OS_MutSemGive(g_request_queue.mutex);
    return rc;
}

/* Pop a request from the queue (available for future async processing) */
__attribute__((unused))
static int BUS_COMMS_RequestQueuePop(BUS_COMMS_CanRequest_t *req)
{
    int rc = -1;

    OS_MutSemTake(g_request_queue.mutex);

    if (g_request_queue.count > 0) {
        memcpy(req, &g_request_queue.entries[g_request_queue.head], sizeof(*req));
        g_request_queue.head = (g_request_queue.head + 1) % BUS_COMMS_REQUEST_QUEUE_DEPTH;
        g_request_queue.count--;
        rc = 0;
    }

    OS_MutSemGive(g_request_queue.mutex);
    return rc;
}

/*
 * Publish a CAN response message on the software bus.
 * Other apps subscribed to BUS_COMMS_CAN_RESPONSE_MID will receive this.
 */
static void BUS_COMMS_PublishResponse(uint8_t subsystem, uint16_t sequence,
                                       uint8_t src_node, uint8_t src_port,
                                       uint8_t error_code,
                                       const uint8_t *data, uint16_t data_len)
{
    BUS_COMMS_CanResponse_t resp;

    CFE_MSG_Init(CFE_MSG_PTR(resp.Header), CFE_SB_ValueToMsgId(BUS_COMMS_CAN_RESPONSE_MID), sizeof(resp));

    resp.direction  = BUS_COMMS_DIR_RESPONSE;
    resp.subsystem  = subsystem;
    resp.src_node   = src_node;
    resp.src_port   = src_port;
    resp.sequence   = sequence;
    resp.error_code = error_code;
    resp.spare      = 0;

    if (data != NULL && data_len > 0) {
        if (data_len > BUS_COMMS_CAN_MAX_DATA_LEN) {
            data_len = BUS_COMMS_CAN_MAX_DATA_LEN;
        }
        memcpy(resp.data, data, data_len);
        resp.data_len = data_len;
    } else {
        resp.data_len = 0;
    }

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(resp.Header));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(resp.Header), true);

    g_sbn_response_count++;
}
