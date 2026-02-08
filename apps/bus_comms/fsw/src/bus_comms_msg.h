/************************************************************************
 * Bus Communications App - messages
 ************************************************************************/
#ifndef BUS_COMMS_MSG_H
#define BUS_COMMS_MSG_H

#include "cfe_msg.h"

#define BUS_COMMS_NOOP_CC           0
#define BUS_COMMS_RESET_COUNTERS_CC 1
#define BUS_COMMS_SEND_CSP_CC       0x10

#define BUS_COMMS_MAX_SEND_LEN      220

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    struct {
        uint8 CommandCounter;
        uint8 CommandErrorCounter;
        uint8 Spare[2];
    } Payload;
} BUS_COMMS_HkTlm_t;

typedef struct {
    CFE_MSG_CommandHeader_t CmdHdr;
    uint8_t  dest;
    uint8_t  port;
    uint16_t len;
    uint8_t  data[BUS_COMMS_MAX_SEND_LEN];
} BUS_COMMS_SendCspCmd_t;

/* Telemetry message published when a CSP packet is received from an external node */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHdr;
    uint8_t  src_addr;     /* CSP source address */
    uint8_t  src_port;     /* CSP source port */
    uint16_t data_len;     /* payload length */
    uint8_t  data[BUS_COMMS_MAX_SEND_LEN];
} BUS_COMMS_CspRxData_t;

#endif /* BUS_COMMS_MSG_H */
