#!/usr/bin/env python3
"""
ground-station: Mock ground station that receives CFDP file transfers over CSP.

CSP address 3 on vcan0.
Receives CFDP PDUs relayed by radio-mock (addr 2) from BUS_COMMS (addr 1).
Uses the Python ``cfdp`` library (v0.2.0) to reassemble files.

Usage:
    source ~/venv/bin/activate
    LD_LIBRARY_PATH=<libcsp_build_dir> PYTHONPATH=<libcsp_build_dir> \\
        python3 tools/ground-station/ground_station.py
"""

import logging
import os
import sys
import threading
import time

import libcsp_py3 as libcsp

import cfdp
from cfdp.transport.base import Transport
from cfdp.filestore import NativeFileStore

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
GROUND_ADDR = 3         # This process's CSP address
RADIO_ADDR = 2          # Radio-mock relay address
CFDP_PORT = 15          # Dedicated CSP port for CFDP PDU traffic (must be <= CSP_PORT_MAX_BIND)
CAN_INTERFACE = "vcan0"
ENTITY_ID = 2           # Must match RADIO_APP_CFDP_DEST_EID in cFS
DEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "received_files")

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [ground-station] %(levelname)s %(message)s",
)
log = logging.getLogger("ground-station")


# ---------------------------------------------------------------------------
# CspTransport -- bridges CSP with the cfdp library
# ---------------------------------------------------------------------------
class CspTransport(Transport):
    """CFDP transport over CSP (Cubesat Space Protocol).

    Inbound : CSP packets on ``cfdp_port`` are fed to the cfdp entity.
    Outbound: ACK / NAK / FIN PDUs are sent back through CSP to the
              radio-mock relay.

    Thread-safety note:
        CfdpEntity.__init__ overrides ``transport.indication`` with
        ``lambda x: self.pdu_received(x)``.  The existing UdpTransport
        already calls ``self.indication(pdu)`` from its own receive thread,
        so this threading pattern is established and accepted by the library.
    """

    def __init__(self, relay_addr, cfdp_port):
        super().__init__()
        self.relay_addr = relay_addr
        self.cfdp_port = cfdp_port
        self._sock = None
        self._thread = None
        self._running = False
        self._rx_count = 0
        self._tx_count = 0

    # -- lifecycle -----------------------------------------------------------

    def bind(self):
        """Create a CSP socket bound to *cfdp_port* and start the RX loop."""
        self._sock = libcsp.socket()
        libcsp.bind(self._sock, self.cfdp_port)
        libcsp.listen(self._sock, 10)
        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()
        log.info("CspTransport bound to port %d", self.cfdp_port)

    def unbind(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        log.info("CspTransport unbound  (rx=%d tx=%d)", self._rx_count, self._tx_count)

    def shutdown(self):
        self.unbind()

    # -- outbound (called by cfdp entity for ACK / NAK / FIN) ---------------

    def request(self, pdu):
        """Send a raw PDU back through CSP to the radio-mock relay."""
        try:
            conn = libcsp.connect(
                libcsp.CSP_PRIO_NORM, self.relay_addr, self.cfdp_port,
                1000, libcsp.CSP_O_NONE,
            )
            pkt = libcsp.buffer_get(0)
            libcsp.packet_set_data(pkt, pdu)
            libcsp.send(conn, pkt)
            libcsp.close(conn)
            self._tx_count += 1
            log.debug("TX PDU -> relay addr %d port %d  len=%d",
                      self.relay_addr, self.cfdp_port, len(pdu))
        except Exception as exc:
            log.error("Failed to send PDU to relay: %s", exc)

    # -- inbound (CSP receive loop) -----------------------------------------

    def _rx_loop(self):
        log.info("RX loop started -- waiting for CFDP PDUs on port %d", self.cfdp_port)
        while self._running:
            conn = libcsp.accept(self._sock, 1000)  # 1-second timeout
            if not conn:
                continue
            src = libcsp.conn_src(conn)
            while True:
                packet = libcsp.read(conn, 100)
                if packet is None:
                    break
                length = libcsp.packet_get_length(packet)
                data = bytes(libcsp.packet_get_data(packet)[:length])
                self._rx_count += 1
                log.debug("RX PDU from addr %d  len=%d  (total rx=%d)",
                          src, length, self._rx_count)
                # ``self.indication`` is overwritten by CfdpEntity to call
                # ``pdu_received()`` -- see CfdpEntity.__init__.
                try:
                    self.indication(data)
                except Exception as exc:
                    log.error("cfdp indication error: %s", exc)
            libcsp.close(conn)
        log.info("RX loop stopped")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    # Ensure destination directory exists
    os.makedirs(DEST_DIR, exist_ok=True)

    # -- CSP init -----------------------------------------------------------
    log.info("Initializing CSP -- addr=%d iface=%s", GROUND_ADDR, CAN_INTERFACE)
    libcsp.init("ground-station", "gs", "1.0")
    libcsp.can_socketcan_init(CAN_INTERFACE, GROUND_ADDR)
    libcsp.rtable_load("0/0 CAN")
    libcsp.route_start_task()

    log.info("Interfaces:")
    libcsp.print_interfaces()
    log.info("Routes:")
    libcsp.print_routes()

    # -- CFDP entity --------------------------------------------------------
    transport = CspTransport(relay_addr=RADIO_ADDR, cfdp_port=CFDP_PORT)
    filestore = NativeFileStore(DEST_DIR)
    entity = cfdp.CfdpEntity(
        entity_id=ENTITY_ID,
        filestore=filestore,
        transport=transport,
        transaction_finished_indication_required=True,
    )
    transport.bind()

    log.info("Ground station ready -- entity_id=%d  dest_dir=%s", ENTITY_ID, DEST_DIR)
    log.info("Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    log.info("Shutting down...")
    entity.shutdown()
    transport.shutdown()
    log.info("Done.")


if __name__ == "__main__":
    main()
