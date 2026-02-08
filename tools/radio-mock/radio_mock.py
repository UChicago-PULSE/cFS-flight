#!/usr/bin/env python3
"""
radio-mock: CSP store-and-forward proxy that simulates a satellite radio.

CSP address 2 on vcan0.
Relays packets between BUS_COMMS (addr 1) and ground-station (addr 3).

Usage:
    source ~/venv/bin/activate
    LD_LIBRARY_PATH=<libcsp_build_dir> PYTHONPATH=<libcsp_build_dir> \
        python3 tools/radio-mock/radio_mock.py
"""

import logging
import sys
import time
import threading

import libcsp_py3 as libcsp

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
RADIO_ADDR = 2          # This process's CSP address
SATELLITE_ADDR = 1      # BUS_COMMS (cFS)
GROUND_ADDR = 3         # Ground-station mock
CAN_INTERFACE = "vcan0"
CFDP_PORT = 15          # Must match BUS_COMMS_CSP_CFDP_PORT and <= CSP_PORT_MAX_BIND (16)
MAX_RETRIES = 3
RETRY_DELAY_S = 0.05    # 50 ms between retries

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [radio-mock] %(levelname)s %(message)s",
)
log = logging.getLogger("radio-mock")

# ---------------------------------------------------------------------------
# Forwarding logic
# ---------------------------------------------------------------------------
_stats = {"fwd_ok": 0, "fwd_fail": 0, "dropped": 0}


def forward_packet(data, length, src, dport):
    """Forward a CSP packet payload to the opposite side of the link."""
    if src == SATELLITE_ADDR:
        fwd_addr = GROUND_ADDR
    elif src == GROUND_ADDR:
        fwd_addr = SATELLITE_ADDR
    else:
        log.warning("Unknown source addr %d -- dropping", src)
        _stats["dropped"] += 1
        return

    for attempt in range(1, MAX_RETRIES + 1):
        try:
            fwd_conn = libcsp.connect(
                libcsp.CSP_PRIO_NORM, fwd_addr, dport, 1000, libcsp.CSP_O_NONE
            )
            fwd_pkt = libcsp.buffer_get(0)
            libcsp.packet_set_data(fwd_pkt, data[:length])
            libcsp.send(fwd_conn, fwd_pkt)
            libcsp.close(fwd_conn)
            _stats["fwd_ok"] += 1
            log.debug(
                "FWD addr %d:%d -> addr %d:%d  len=%d",
                src, dport, fwd_addr, dport, length,
            )
            return
        except Exception as exc:
            log.warning(
                "Forward attempt %d/%d failed (src=%d -> dst=%d port=%d): %s",
                attempt, MAX_RETRIES, src, fwd_addr, dport, exc,
            )
            if attempt < MAX_RETRIES:
                time.sleep(RETRY_DELAY_S)

    log.error("Giving up forwarding packet from %d to %d port %d", src, fwd_addr, dport)
    _stats["fwd_fail"] += 1


def stats_printer():
    """Periodically log forwarding statistics."""
    while True:
        time.sleep(10)
        log.info(
            "Stats: fwd_ok=%d  fwd_fail=%d  dropped=%d",
            _stats["fwd_ok"], _stats["fwd_fail"], _stats["dropped"],
        )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    log.info("Initializing CSP -- addr=%d iface=%s", RADIO_ADDR, CAN_INTERFACE)
    libcsp.init("radio-mock", "mock", "1.0")
    libcsp.can_socketcan_init(CAN_INTERFACE, RADIO_ADDR)
    libcsp.rtable_load("0/0 CAN")
    libcsp.route_start_task()

    log.info("Interfaces:")
    libcsp.print_interfaces()
    log.info("Routes:")
    libcsp.print_routes()

    # Background stats thread
    threading.Thread(target=stats_printer, daemon=True).start()

    # Listen on ALL ports
    sock = libcsp.socket()
    libcsp.bind(sock, libcsp.CSP_ANY)
    libcsp.listen(sock, 10)
    log.info("Listening on CSP_ANY -- relaying between addr %d and addr %d",
             SATELLITE_ADDR, GROUND_ADDR)

    try:
        while True:
            conn = libcsp.accept(sock, libcsp.CSP_MAX_TIMEOUT)
            if not conn:
                continue

            src = libcsp.conn_src(conn)
            dport = libcsp.conn_dport(conn)

            while True:
                packet = libcsp.read(conn, 100)
                if packet is None:
                    break
                data = bytearray(libcsp.packet_get_data(packet))
                length = libcsp.packet_get_length(packet)
                forward_packet(data, length, src, dport)

            libcsp.close(conn)
    except KeyboardInterrupt:
        log.info("Shutting down.")
        sys.exit(0)


if __name__ == "__main__":
    main()
