/************************************************************************
 * Bus Communications App - header file
 ************************************************************************/
#ifndef BUS_COMMS_APP_H
#define BUS_COMMS_APP_H

#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"
#include "cfe_msg.h"

#include "bus_comms_perfids.h"
#include "bus_comms_msgids.h"
#include "bus_comms_msg.h"
#include "bus_comms_table.h"

#define BUS_COMMS_APP_PIPE_DEPTH 32

typedef struct
{
    uint8 CmdCounter;
    uint8 ErrCounter;

    BUS_COMMS_HkTlm_t HkTlm;

    uint32 RunStatus;

    CFE_SB_PipeId_t CmdPipe;

    char   PipeName[CFE_MISSION_MAX_API_LEN];
    uint16 PipeDepth;

    CFE_TBL_Handle_t   TblHandle;
    BUS_COMMS_Table_t *TblPtr;
} BUS_COMMS_AppData_t;

extern BUS_COMMS_AppData_t BUS_COMMS_AppData;

void  BUS_COMMS_AppMain(void);
int32 BUS_COMMS_AppInit(void);
void  BUS_COMMS_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr);
void  BUS_COMMS_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr);
int32 BUS_COMMS_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg);

#endif /* BUS_COMMS_APP_H */
