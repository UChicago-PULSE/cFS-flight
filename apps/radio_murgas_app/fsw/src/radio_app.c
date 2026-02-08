/************************************************************************
 * NASA Docket No. GSC-18,719-1, and identified as "core Flight System: Bootes"
 *
 * Copyright (c) 2020 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

#include "radio_app_events.h"
#include "radio_app_version.h"
#include "radio_app.h"
#include "radio_app_table.h"
#include "bus_comms_msg.h"
#include "bus_comms_msgids.h"
#include "osapi-file.h"
#include "cf_msgids.h"
#include "cf_msgstruct.h"
#include "cf_fcncodes.h"
#include "cf_extern_typedefs.h"
#include "osapi.h"
#include <string.h>

#define RADIO_APP_RADIO_NODE_ADDR       2   /* CSP addr of the radio (used for non-CFDP traffic) */
#define RADIO_APP_GROUND_STATION_ADDR   3   /* CSP addr of the ground station (CFDP destination) */
#define RADIO_APP_CONFIG_PORT           7
#define RADIO_APP_CFDP_DEST_EID         2   /* CFDP entity ID of the ground station */
#define RADIO_APP_CFDP_CSP_PORT   15  /* Dedicated CSP port for CFDP PDU traffic (must be <= CSP_PORT_MAX_BIND) */

/*
 * Offset from SB message start to the raw CFDP PDU payload.
 * CF builds outgoing PDUs using CF_PduTlmMsg_t layout (cf_cfdp_sbintf.c:117-119),
 * where the PDU starts immediately after CFE_MSG_TelemetryHeader_t.
 * This is sizeof(CFE_MSG_TelemetryHeader_t) = offsetof(CF_PduTlmMsg_t, ph).
 * We compute it here to avoid pulling in CF's heavy internal headers.
 */
#define RADIO_APP_CFDP_PDU_OFFSET  sizeof(CFE_MSG_TelemetryHeader_t)

RADIO_APP_Data_t RADIO_APP_Data;

void RADIO_APP_Main(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    CFE_ES_PerfLogEntry(RADIO_APP_PERF_ID);

    status = RADIO_APP_Init();
    if (status != CFE_SUCCESS)
    {
        RADIO_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    while (CFE_ES_RunLoop(&RADIO_APP_Data.RunStatus) == true)
    {
        CFE_ES_PerfLogExit(RADIO_APP_PERF_ID);

        status = CFE_SB_ReceiveBuffer(&SBBufPtr, RADIO_APP_Data.CommandPipe, CFE_SB_PEND_FOREVER);

        CFE_ES_PerfLogEntry(RADIO_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            RADIO_APP_ProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(RADIO_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "RADIO APP: SB Pipe Read Error, App Will Exit");

            RADIO_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    CFE_ES_PerfLogExit(RADIO_APP_PERF_ID);

    CFE_ES_ExitApp(RADIO_APP_Data.RunStatus);
}

int32 RADIO_APP_Init(void)
{
    int32 status;

    RADIO_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    RADIO_APP_Data.CmdCounter = 0;
    RADIO_APP_Data.ErrCounter = 0;

    RADIO_APP_Data.PipeDepth = RADIO_APP_PIPE_DEPTH;

    strncpy(RADIO_APP_Data.PipeName, "RADIO_APP_CMD_PIPE", sizeof(RADIO_APP_Data.PipeName));
    RADIO_APP_Data.PipeName[sizeof(RADIO_APP_Data.PipeName) - 1] = 0;

    status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Radio App: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    CFE_MSG_Init(CFE_MSG_PTR(RADIO_APP_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(RADIO_APP_HK_TLM_MID),
                 sizeof(RADIO_APP_Data.HkTlm));

    status = CFE_SB_CreatePipe(&RADIO_APP_Data.CommandPipe, RADIO_APP_Data.PipeDepth, RADIO_APP_Data.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Radio App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(RADIO_APP_SEND_HK_MID), RADIO_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Radio App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(RADIO_APP_CMD_MID), RADIO_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Radio App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    status = CFE_TBL_Register(&RADIO_APP_Data.TblHandles[0], "RadioAppTable", sizeof(RADIO_APP_Table_t),
                              CFE_TBL_OPT_DEFAULT, RADIO_APP_TblValidationFunc);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Radio App: Error Registering Table, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }
    else
    {
        status = CFE_TBL_Load(RADIO_APP_Data.TblHandles[0], CFE_TBL_SRC_FILE, RADIO_APP_TABLE_FILE);
    }

    /* Subscribe to CFDP outgoing PDUs so we can forward them through BUS_COMMS/CSP */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(CF_CH0_TX_MID), RADIO_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Radio App: Error Subscribing to CF_CH0_TX_MID, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /* Subscribe to incoming CSP data from BUS_COMMS (uplink PDUs from ground station) */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BUS_COMMS_CSP_RX_DATA_MID), RADIO_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Radio App: Error Subscribing to BUS_COMMS_CSP_RX_DATA_MID, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    CFE_EVS_SendEvent(RADIO_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "Radio App Initialized.%s",
                      RADIO_APP_VERSION_STRING);

    /* NOTE: Startup auto-transfer removed.  File transfers are triggered via
     * ground command (RADIO_APP_TRANSMIT_FILE_CC) after all apps are up. */

    return CFE_SUCCESS;
}

void RADIO_APP_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case RADIO_APP_CMD_MID:
            RADIO_APP_ProcessGroundCommand(SBBufPtr);
            break;

        case RADIO_APP_SEND_HK_MID:
            RADIO_APP_ReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case CF_CH0_TX_MID:
        {
            /*
             * TX path: Forward CFDP PDUs from CF to BUS_COMMS -> radio -> ground station.
             *
             * CRITICAL: CF builds outgoing PDUs using CF_PduTlmMsg_t layout
             * (cf_cfdp_sbintf.c:117-119). The raw CFDP PDU starts at
             * offsetof(CF_PduTlmMsg_t, ph) = 12 bytes (TLM header size),
             * regardless of the MID type.
             */
            CFE_MSG_Size_t msg_size = 0;

            CFE_MSG_GetSize(&SBBufPtr->Msg, &msg_size);

            if (msg_size > RADIO_APP_CFDP_PDU_OFFSET)
            {
                const uint8 *pdu_data = ((const uint8 *)SBBufPtr) + RADIO_APP_CFDP_PDU_OFFSET;
                uint16       pdu_len  = (uint16)(msg_size - RADIO_APP_CFDP_PDU_OFFSET);

                RADIO_APP_SendFileChunkToBusComms(pdu_data, pdu_len,
                    RADIO_APP_GROUND_STATION_ADDR, RADIO_APP_CFDP_CSP_PORT);
            }
            break;
        }

        case BUS_COMMS_CSP_RX_DATA_MID:
        {
            /*
             * Uplink path: Forward CFDP ACK/NAK/FIN PDUs from ground station
             * (via BUS_COMMS CSP receiver) to CF app.
             *
             * CRITICAL: CF_CH0_RX_MID is a TLM MID (CFE_PLATFORM_CF_TLM_MIDVAL).
             * CF's receive path checks CFE_MSG_GetType() and uses
             * offsetof(CF_PduTlmMsg_t, ph) for TLM-type messages.
             * We MUST use CF_PduTlmMsg_t (TelemetryHeader), NOT CF_PduCmdMsg_t.
             */
            const BUS_COMMS_CspRxData_t *rx = (const BUS_COMMS_CspRxData_t *)SBBufPtr;

            if (rx->src_port == RADIO_APP_CFDP_CSP_PORT && rx->data_len > 0)
            {
                uint8              buf[RADIO_APP_CFDP_PDU_OFFSET + BUS_COMMS_MAX_SEND_LEN];
                CFE_MSG_Message_t *msg   = (CFE_MSG_Message_t *)buf;
                size_t             total = RADIO_APP_CFDP_PDU_OFFSET + rx->data_len;

                memset(buf, 0, sizeof(buf));
                CFE_MSG_Init(msg, CFE_SB_ValueToMsgId(CF_CH0_RX_MID), total);
                memcpy(buf + RADIO_APP_CFDP_PDU_OFFSET, rx->data, rx->data_len);
                CFE_SB_TransmitMsg(msg, true);

                CFE_EVS_SendEvent(RADIO_APP_FILE_TX_INF_EID, CFE_EVS_EventType_DEBUG,
                                  "RADIO: Uplink PDU forwarded to CF, len=%u", (unsigned)rx->data_len);
            }
            break;
        }

        default:
            CFE_EVS_SendEvent(RADIO_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "RADIO: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }
}

void RADIO_APP_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    switch (CommandCode)
    {
        case RADIO_APP_NOOP_CC:
            if (RADIO_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(RADIO_APP_NoopCmd_t)))
            {
                RADIO_APP_Noop((RADIO_APP_NoopCmd_t *)SBBufPtr);
            }
            break;

        case RADIO_APP_RESET_COUNTERS_CC:
            if (RADIO_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(RADIO_APP_ResetCountersCmd_t)))
            {
                RADIO_APP_ResetCounters((RADIO_APP_ResetCountersCmd_t *)SBBufPtr);
            }
            break;

        case RADIO_APP_CONFIGURE_CC:
            if (RADIO_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(RADIO_APP_ConfigureCmd_t)))
            {
                RADIO_APP_ConfigureRadio((RADIO_APP_ConfigureCmd_t *)SBBufPtr);
            }
            break;

        case RADIO_APP_TRANSMIT_CC:
            if (RADIO_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(RADIO_APP_TransmitCmd_t)))
            {
                RADIO_APP_TransmitData((RADIO_APP_TransmitCmd_t *)SBBufPtr);
            }
            break;

        case RADIO_APP_HK_REQUEST_CC:
            if (RADIO_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(RADIO_APP_HkRequestCmd_t)))
            {
                RADIO_APP_RequestHousekeeping((RADIO_APP_HkRequestCmd_t *)SBBufPtr);
            }
            break;

        case RADIO_APP_TRANSMIT_FILE_CC:
            if (RADIO_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(RADIO_APP_TransmitFileCmd_t)))
            {
                RADIO_APP_TransmitFile((RADIO_APP_TransmitFileCmd_t *)SBBufPtr);
            }
            break;

        default:
            CFE_EVS_SendEvent(RADIO_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }
}

int32 RADIO_APP_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{
    int i;

    RADIO_APP_Data.HkTlm.Payload.CommandErrorCounter = RADIO_APP_Data.ErrCounter;
    RADIO_APP_Data.HkTlm.Payload.CommandCounter      = RADIO_APP_Data.CmdCounter;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(RADIO_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(RADIO_APP_Data.HkTlm.TelemetryHeader), true);

    for (i = 0; i < RADIO_APP_NUMBER_OF_TABLES; i++)
    {
        CFE_TBL_Manage(RADIO_APP_Data.TblHandles[i]);
    }

    RADIO_APP_ProcessTableUpdate();

    return CFE_SUCCESS;
}

int32 RADIO_APP_Noop(const RADIO_APP_NoopCmd_t *Msg)
{
    RADIO_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(RADIO_APP_COMMANDNOOP_INF_EID, CFE_EVS_EventType_INFORMATION, "RADIO: NOOP command %s",
                      RADIO_APP_VERSION);

    return CFE_SUCCESS;
}

int32 RADIO_APP_ResetCounters(const RADIO_APP_ResetCountersCmd_t *Msg)
{
    RADIO_APP_Data.CmdCounter = 0;
    RADIO_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(RADIO_APP_COMMANDRESET_INF_EID, CFE_EVS_EventType_INFORMATION, "RADIO: RESET command");

    return CFE_SUCCESS;
}

int32 RADIO_APP_ConfigureRadio(const RADIO_APP_ConfigureCmd_t *Msg)
{
    int32 status;

    RADIO_APP_Data.CmdCounter++;

    if (Msg->CommandLength > 256)
    {
        CFE_EVS_SendEvent(RADIO_APP_CONFIG_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RADIO: Command length exceeds maximum (256)");
        RADIO_APP_Data.ErrCounter++;
        return CFE_SB_BAD_ARGUMENT;
    }

    status = RADIO_APP_SendConfigToBusComms(Msg);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RADIO_APP_CONFIG_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RADIO: Failed to send config to bus_comms, RC = 0x%08lX", (unsigned long)status);
        RADIO_APP_Data.ErrCounter++;
        return status;
    }

    CFE_EVS_SendEvent(RADIO_APP_CONFIGURE_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "RADIO: Configuration command sent - Len=%u", Msg->CommandLength);

    return CFE_SUCCESS;
}

int32 RADIO_APP_TransmitData(const RADIO_APP_TransmitCmd_t *Msg)
{
    int32 status;

    RADIO_APP_Data.CmdCounter++;

    if (Msg->DataLength > 256)
    {
        CFE_EVS_SendEvent(RADIO_APP_TX_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RADIO: Data length exceeds maximum (256)");
        RADIO_APP_Data.ErrCounter++;
        return CFE_SB_BAD_ARGUMENT;
    }

    status = RADIO_APP_SendTxToBusComms(Msg);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RADIO_APP_TX_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RADIO: Failed to send transmit command to bus_comms, RC = 0x%08lX", (unsigned long)status);
        RADIO_APP_Data.ErrCounter++;
        return status;
    }

    CFE_EVS_SendEvent(RADIO_APP_TRANSMIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "RADIO: Transmit command sent - Dest=%u, Port=%u, Len=%u", Msg->DestAddress, Msg->DestPort,
                      Msg->DataLength);

    return CFE_SUCCESS;
}

int32 RADIO_APP_RequestHousekeeping(const RADIO_APP_HkRequestCmd_t *Msg)
{
    return RADIO_APP_ReportHousekeeping((CFE_MSG_CommandHeader_t *)Msg);
}

int32 RADIO_APP_TransmitFile(const RADIO_APP_TransmitFileCmd_t *Msg)
{
    int32                  status;
    size_t                 FilenameLen;
    const char            *basename_ptr;
    CF_TxFileCmd_t         CfCmd;

    RADIO_APP_Data.CmdCounter++;

    /* Validate filename is not empty */
    FilenameLen = strlen(Msg->Filename);
    if (FilenameLen == 0)
    {
        CFE_EVS_SendEvent(RADIO_APP_FILE_OPEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RADIO: Filename is empty");
        RADIO_APP_Data.ErrCounter++;
        return CFE_SB_BAD_ARGUMENT;
    }

    if (FilenameLen >= OS_MAX_PATH_LEN)
    {
        CFE_EVS_SendEvent(RADIO_APP_FILE_OPEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RADIO: Filename too long: %lu bytes (max %lu)", (unsigned long)FilenameLen,
                          (unsigned long)(OS_MAX_PATH_LEN - 1));
        RADIO_APP_Data.ErrCounter++;
        return CFE_SB_BAD_ARGUMENT;
    }

    /* Send CF_TX_FILE to CF app -- PDUs will flow through the SB back to us
     * (CF_CH0_TX_MID case in ProcessCommandPacket), then out via BUS_COMMS/CSP
     * to the radio-mock and ground-station. */
    basename_ptr = strrchr(Msg->Filename, '/');
    basename_ptr = (basename_ptr != NULL) ? basename_ptr + 1 : Msg->Filename;

    memset(&CfCmd, 0, sizeof(CfCmd));
    CFE_MSG_Init(CFE_MSG_PTR(CfCmd.CommandHeader), CFE_SB_ValueToMsgId(CF_CMD_MID), sizeof(CfCmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(CfCmd.CommandHeader), CF_TX_FILE_CC);

    CfCmd.Payload.cfdp_class = (uint8)CF_CFDP_CLASS_2;
    CfCmd.Payload.keep       = 1;
    CfCmd.Payload.chan_num   = 0;
    CfCmd.Payload.priority   = 0;
    CfCmd.Payload.dest_id    = (CF_EntityId_t)RADIO_APP_CFDP_DEST_EID;

    strncpy(CfCmd.Payload.src_filename, Msg->Filename, CF_FILENAME_MAX_LEN - 1);
    CfCmd.Payload.src_filename[CF_FILENAME_MAX_LEN - 1] = '\0';

    CfCmd.Payload.dst_filename[0] = '/';
    strncpy(&CfCmd.Payload.dst_filename[1], basename_ptr, CF_FILENAME_MAX_LEN - 2);
    CfCmd.Payload.dst_filename[CF_FILENAME_MAX_LEN - 1] = '\0';

    status = CFE_SB_TransmitMsg(CFE_MSG_PTR(CfCmd.CommandHeader), true);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RADIO_APP_FILE_TX_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RADIO: Failed to send CF_TX_FILE, RC = 0x%08lX", (unsigned long)status);
        RADIO_APP_Data.ErrCounter++;
        return status;
    }

    CFE_EVS_SendEvent(RADIO_APP_FILE_TX_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "RADIO: CFDP transfer requested - File='%s' -> '%s', DestEID=%u",
                      Msg->Filename, CfCmd.Payload.dst_filename, (unsigned int)RADIO_APP_CFDP_DEST_EID);

    return CFE_SUCCESS;
}

int32 RADIO_APP_SendConfigToBusComms(const RADIO_APP_ConfigureCmd_t *ConfigCmd)
{
    BUS_COMMS_SendCspCmd_t BusCommsCmd;
    int32                  status;

    if (ConfigCmd->CommandLength > BUS_COMMS_MAX_SEND_LEN)
    {
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    memset(&BusCommsCmd, 0, sizeof(BusCommsCmd));

    memcpy(BusCommsCmd.data, ConfigCmd->Command, ConfigCmd->CommandLength);
    BusCommsCmd.dest = RADIO_APP_RADIO_NODE_ADDR;
    BusCommsCmd.port = RADIO_APP_CONFIG_PORT;
    BusCommsCmd.len  = ConfigCmd->CommandLength;

    CFE_MSG_Init(CFE_MSG_PTR(BusCommsCmd.CmdHdr), CFE_SB_ValueToMsgId(BUS_COMMS_CMD_MID), sizeof(BusCommsCmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(BusCommsCmd.CmdHdr), BUS_COMMS_SEND_CSP_CC);

    status = CFE_SB_TransmitMsg(CFE_MSG_PTR(BusCommsCmd.CmdHdr), true);
    if (status == CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RADIO_APP_BUS_COMMS_SEND_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "RADIO: Config command sent to bus_comms - Dest=%u, Port=%u, Len=%u",
                          BusCommsCmd.dest, BusCommsCmd.port, BusCommsCmd.len);
    }
    return status;
}

int32 RADIO_APP_SendTxToBusComms(const RADIO_APP_TransmitCmd_t *TxCmd)
{
    BUS_COMMS_SendCspCmd_t BusCommsCmd;
    int32                  status;

    if (TxCmd->DataLength > BUS_COMMS_MAX_SEND_LEN)
    {
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    memset(&BusCommsCmd, 0, sizeof(BusCommsCmd));

    memcpy(BusCommsCmd.data, TxCmd->Data, TxCmd->DataLength);
    BusCommsCmd.dest = TxCmd->DestAddress;
    BusCommsCmd.port = TxCmd->DestPort;
    BusCommsCmd.len  = TxCmd->DataLength;

    CFE_MSG_Init(CFE_MSG_PTR(BusCommsCmd.CmdHdr), CFE_SB_ValueToMsgId(BUS_COMMS_CMD_MID), sizeof(BusCommsCmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(BusCommsCmd.CmdHdr), BUS_COMMS_SEND_CSP_CC);

    status = CFE_SB_TransmitMsg(CFE_MSG_PTR(BusCommsCmd.CmdHdr), true);
    if (status == CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RADIO_APP_BUS_COMMS_SEND_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "RADIO: Transmit data sent to bus_comms - Dest=%u, Port=%u, Len=%u",
                          BusCommsCmd.dest, BusCommsCmd.port, BusCommsCmd.len);
    }
    return status;
}

int32 RADIO_APP_SendFileChunkToBusComms(const uint8 *ChunkData, uint16 ChunkSize, uint8 DestAddr, uint8 DestPort)
{
    BUS_COMMS_SendCspCmd_t BusCommsCmd;
    int32                  status;

    if (ChunkSize > BUS_COMMS_MAX_SEND_LEN)
    {
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    /* CFE_MSG_Init zeroes the entire struct, so it MUST be called first */
    CFE_MSG_Init(CFE_MSG_PTR(BusCommsCmd.CmdHdr), CFE_SB_ValueToMsgId(BUS_COMMS_CMD_MID), sizeof(BusCommsCmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(BusCommsCmd.CmdHdr), BUS_COMMS_SEND_CSP_CC);

    memcpy(BusCommsCmd.data, ChunkData, ChunkSize);
    BusCommsCmd.dest = DestAddr;
    BusCommsCmd.port = DestPort;
    BusCommsCmd.len  = ChunkSize;

    status = CFE_SB_TransmitMsg(CFE_MSG_PTR(BusCommsCmd.CmdHdr), true);
    if (status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(RADIO_APP_FILE_TX_ERR_EID, CFE_EVS_EventType_ERROR,
                          "RADIO: Failed to send file chunk to bus_comms, RC = 0x%08lX", (unsigned long)status);
    }
    return status;
}

bool RADIO_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(RADIO_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        RADIO_APP_Data.ErrCounter++;
    }

    return result;
}

int32 RADIO_APP_TblValidationFunc(void *TblData)
{
    int32              ReturnCode = CFE_SUCCESS;
    RADIO_APP_Table_t *TblDataPtr = (RADIO_APP_Table_t *)TblData;

    /* Validate power levels (0-100%) */
    if (TblDataPtr->PowerData > 100 || TblDataPtr->PowerMorse > 100)
    {
        ReturnCode = RADIO_APP_TABLE_OUT_OF_RANGE_ERR_CODE;
    }

    /* Validate RF frequency (1 MHz to 10 GHz) */
    if (TblDataPtr->RfFrequency < 1000000 || TblDataPtr->RfFrequency > 10000000000)
    {
        ReturnCode = RADIO_APP_TABLE_OUT_OF_RANGE_ERR_CODE;
    }

    /* Validate Morse speed (5-50 WPM) */
    if (TblDataPtr->MorseSpeed < 5 || TblDataPtr->MorseSpeed > 50)
    {
        ReturnCode = RADIO_APP_TABLE_OUT_OF_RANGE_ERR_CODE;
    }

    /* Validate Max PA temp (0-150 C) */
    if (TblDataPtr->MaxPaTemp > 150)
    {
        ReturnCode = RADIO_APP_TABLE_OUT_OF_RANGE_ERR_CODE;
    }

    /* Validate PA voltage (0-5000 mV) */
    if (TblDataPtr->PaVoltage > 5000)
    {
        ReturnCode = RADIO_APP_TABLE_OUT_OF_RANGE_ERR_CODE;
    }

    /* Validate baudrates (reasonable ranges) */
    if (TblDataPtr->I2cBaudrate > 3400000 || TblDataPtr->Rs485Baudrate > 115200 ||
        TblDataPtr->UartBaudrate > 115200 || TblDataPtr->TrxBaudrate > 115200)
    {
        ReturnCode = RADIO_APP_TABLE_OUT_OF_RANGE_ERR_CODE;
    }

    /* Validate beacon period (1-3600 seconds) */
    if (TblDataPtr->BeaconPeriod < 1 || TblDataPtr->BeaconPeriod > 3600)
    {
        ReturnCode = RADIO_APP_TABLE_OUT_OF_RANGE_ERR_CODE;
    }

    /* Validate CSP address (1-255) */
    if (TblDataPtr->CspAddress == 0 || TblDataPtr->CspAddress > 255)
    {
        ReturnCode = RADIO_APP_TABLE_OUT_OF_RANGE_ERR_CODE;
    }

    return ReturnCode;
}

void RADIO_APP_ProcessTableUpdate(void)
{
    int32              status;
    RADIO_APP_Table_t *TblPtr;

    status = CFE_TBL_GetAddress((void *)&TblPtr, RADIO_APP_Data.TblHandles[0]);

    if (status == CFE_TBL_INFO_UPDATED)
    {
        /* TODO: Add your logic here to handle table updates */
        /* Example: reconfigure radio with new parameters */
        /* TblPtr->RfFrequency, TblPtr->PowerData, TblPtr->PowerMorse, etc. */
    }

    if (status >= CFE_SUCCESS)
    {
        CFE_TBL_ReleaseAddress(RADIO_APP_Data.TblHandles[0]);
    }
}

