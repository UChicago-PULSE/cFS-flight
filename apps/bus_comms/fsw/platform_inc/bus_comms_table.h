/************************************************************************
 * Bus Communications App - routing table definitions (minimal stub)
 ************************************************************************/
#ifndef BUS_COMMS_TABLE_H
#define BUS_COMMS_TABLE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Minimal device entry and table definition to satisfy includes.
 * Expand as needed when you add real routing entries.
 */
typedef struct
{
    uint8_t device_id;   /* Device identifier */
    uint8_t port;        /* Logical port on the bus/radio */
    bool    active;      /* Whether this route is active */
} BUS_COMMS_DeviceEntry_t;

typedef struct
{
    uint8_t  HeartbeatEnabled;
    uint32_t HeartbeatInterval_ms;
    BUS_COMMS_DeviceEntry_t entries[16];
} BUS_COMMS_Table_t;

/* Optional table constants (not required unless you register tables) */
#define BUS_COMMS_NUMBER_OF_TABLES 0
#define BUS_COMMS_TABLE_FILE       "/cf/bus_comms_tbl.tbl"

#endif /* BUS_COMMS_TABLE_H */
