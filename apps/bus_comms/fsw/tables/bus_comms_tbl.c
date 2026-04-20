#include "cfe.h"
#include "cfe_tbl_filedef.h"
#include "bus_comms_table.h"

BUS_COMMS_Table_t BUS_COMMS_Table =
{
    .HeartbeatEnabled = 1,
    .HeartbeatInterval_ms = 25000,
    .entries = { {0} }
};

CFE_TBL_FILEDEF(BUS_COMMS_Table, bus_comms.ConfigTbl, Bus Comms Config Table, bus_comms_tbl.tbl)
