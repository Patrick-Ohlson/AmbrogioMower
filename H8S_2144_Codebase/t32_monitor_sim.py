"""
t32_monitor_sim.py — Trace32 ROM Monitor Protocol Simulator for H8S/2144

Two modes of operation:

  Simulator mode (default):
    Emulates the monitor protocol over a virtual COM port so Trace32 can
    connect without real hardware.

    python t32_monitor_sim.py --port COM5 [--mot main.mot] [--verbose]

  Proxy mode (--target):
    Sits between Trace32 and real hardware, forwarding all bytes while
    parsing and annotating the protocol in real time.  Detects sync loss,
    checksum failures, and timeouts.

    python t32_monitor_sim.py --port COM5 --target COM3 [--verbose]

Requires: pyserial  (pip install pyserial)
Virtual COM pair: com0com on Windows (e.g. COM4 ↔ COM5)
"""

import argparse
import logging
import os
import select
import sys
import threading
import time

import serial

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log = logging.getLogger("t32sim")

# Coloured helpers for hex dumps (if terminal supports it)
def _hex(data):
    """Format bytes/list as hex string."""
    if isinstance(data, int):
        return f"{data:02X}"
    return " ".join(f"{b:02X}" for b in data)


# ---------------------------------------------------------------------------
# S-Record Parser
# ---------------------------------------------------------------------------

def load_srec(path):
    """Parse a Motorola S-record file into a dict {addr: byte_value}.

    Supports S0 (header), S1 (16-bit addr), S2 (24-bit addr),
    S5 (record count), S9 (entry point, ignored).
    """
    memory = {}
    records = 0
    with open(path, "r") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if len(line) < 4:
                log.warning("srec line %d too short: %s", lineno, line)
                continue
            rtype = line[0:2]
            byte_count = int(line[2:4], 16)
            raw = bytes.fromhex(line[4:])
            # Verify checksum: 1's complement of sum of byte_count + address + data
            chk = byte_count
            for b in raw[:-1]:
                chk += b
            chk = (~chk) & 0xFF
            if chk != raw[-1]:
                log.warning("srec line %d checksum mismatch (got %02X, expected %02X)",
                            lineno, raw[-1], chk)
            if rtype == "S0":
                continue  # header — skip
            elif rtype == "S1":
                addr = (raw[0] << 8) | raw[1]
                data = raw[2:-1]
            elif rtype == "S2":
                addr = (raw[0] << 16) | (raw[1] << 8) | raw[2]
                data = raw[3:-1]
            elif rtype == "S3":
                addr = (raw[0] << 24) | (raw[1] << 16) | (raw[2] << 8) | raw[3]
                data = raw[4:-1]
            elif rtype in ("S5", "S7", "S8", "S9"):
                continue  # terminator / count — skip
            else:
                log.warning("srec line %d unknown record type %s", lineno, rtype)
                continue
            for i, b in enumerate(data):
                memory[addr + i] = b
            records += 1
    log.info("Loaded %d S-records, %d bytes from %s", records, len(memory), path)
    return memory


# ---------------------------------------------------------------------------
# Checksum (16-bit accumulator, matches monitor.S E3 register)
# ---------------------------------------------------------------------------

class Checksum:
    """16-bit checksum accumulator matching the monitor protocol."""

    def __init__(self):
        self.value = 0

    def reset(self):
        self.value = 0

    def add(self, byte_val):
        self.value = (self.value + (byte_val & 0xFF)) & 0xFFFF

    @property
    def high(self):
        return (self.value >> 8) & 0xFF

    @property
    def low(self):
        return self.value & 0xFF


# ---------------------------------------------------------------------------
# CPU Register State
# ---------------------------------------------------------------------------

# Protocol register numbering (matches monitor.S writereg jump table):
#   0=ER7/SP  1=MACH  2=MACL  3=ER6  4=ER5  5=ER4
#   6=ER3     7=ER2   8=ER1   9=ER0  10=EXR 11=CCR+PC
REG_NAMES = ["ER7", "MACH", "MACL", "ER6", "ER5", "ER4",
             "ER3", "ER2",  "ER1",  "ER0", "EXR", "CCR+PC"]


class H8SRegisters:
    """H8S/2144 CPU register file (shadow for simulator)."""

    def __init__(self):
        self.regs = [0] * 12  # indexed by protocol numbering

    def get(self, idx):
        if 0 <= idx < 12:
            return self.regs[idx] & 0xFFFFFFFF
        return 0

    def set(self, idx, value):
        if 0 <= idx < 12:
            self.regs[idx] = value & 0xFFFFFFFF
            log.info("  REG[%d] %s = 0x%08X", idx, REG_NAMES[idx], self.regs[idx])

    def dump(self):
        for i, name in enumerate(REG_NAMES):
            log.info("  %6s = 0x%08X", name, self.regs[i])


# ---------------------------------------------------------------------------
# Monitor Simulator
# ---------------------------------------------------------------------------

class MonitorSim:
    """Emulates the Trace32 ROM monitor serial protocol (V4.3, serial mode)."""

    def __init__(self, ser, memory, verbose=False):
        self.ser = ser
        self.memory = memory       # dict {addr: byte}
        self.regs = H8SRegisters()
        self.msg_id = 0x00         # r3h: message ID / entry reason
        self.cksum = Checksum()
        self.verbose = verbose
        self._tx_buf = []          # for batch logging
        self._rx_buf = []

    # --- Low-level serial I/O with logging ---

    def tx(self, byte_val):
        """Transmit one byte to host."""
        byte_val &= 0xFF
        self.ser.write(bytes([byte_val]))
        self._tx_buf.append(byte_val)

    def rx(self):
        """Receive one byte from host. Blocks until available."""
        while True:
            data = self.ser.read(1)
            if data:
                b = data[0]
                self._rx_buf.append(b)
                return b

    def flush_log(self, label=""):
        """Flush accumulated TX/RX bytes to log."""
        if self._tx_buf:
            log.debug("[TX] %s  %s", _hex(self._tx_buf), label)
            self._tx_buf = []
        if self._rx_buf:
            log.debug("[RX] %s  %s", _hex(self._rx_buf), label)
            self._rx_buf = []

    def tx_with_checksum(self, byte_val):
        """Transmit one byte and add to checksum."""
        self.cksum.add(byte_val)
        self.tx(byte_val)

    def rx_with_checksum(self):
        """Receive one byte and add to checksum."""
        b = self.rx()
        self.cksum.add(b)
        return b

    # --- Memory access ---

    def mem_read(self, addr):
        return self.memory.get(addr & 0xFFFFFF, 0xFF)

    def mem_write(self, addr, val):
        self.memory[addr & 0xFFFFFF] = val & 0xFF

    # --- Protocol building blocks ---

    def send_long(self, value):
        """Send a 32-bit value big-endian, adding each byte to checksum."""
        self.tx_with_checksum((value >> 24) & 0xFF)
        self.tx_with_checksum((value >> 16) & 0xFF)
        self.tx_with_checksum((value >> 8) & 0xFF)
        self.tx_with_checksum(value & 0xFF)

    def recv_long(self):
        """Receive a 32-bit value big-endian, adding each byte to checksum."""
        b0 = self.rx_with_checksum()
        b1 = self.rx_with_checksum()
        b2 = self.rx_with_checksum()
        b3 = self.rx_with_checksum()
        return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3

    def verify_data_checksum(self):
        """Read 2 checksum bytes and XOR-verify against running checksum.

        Returns True if valid.  Assembly: xor.w e3,r0 then cmp 0.
        """
        chk_hi = self.rx()
        chk_lo = self.rx()
        ok = ((chk_hi ^ self.cksum.high) == 0) and ((chk_lo ^ self.cksum.low) == 0)
        if not ok:
            log.warning("  DATA CHECKSUM FAIL: got %02X %02X, expected %02X %02X",
                        chk_hi, chk_lo, self.cksum.high, self.cksum.low)
        return ok

    def send_checksum_footer(self):
        """Send 2-byte checksum (mon_mainlp1 path)."""
        self.tx(self.cksum.high)
        self.tx(self.cksum.low)

    def do_sync_send(self):
        """Monitor→Host sync: TX 0x20, RX expect 0x3F. Returns success."""
        self.tx(0x20)
        b = self.rx()
        self.flush_log("sync_send")
        if b != 0x3F:
            log.warning("  Sync fail: expected 0x3F, got 0x%02X", b)
            return False
        return True

    def checkandreply(self, data_len):
        """Verify data checksum, then do sync handshake and send header.

        Matches monitor.S 'checkandreply' subroutine.
        Returns True on success.
        """
        if not self.verify_data_checksum():
            return False
        self.flush_log("checkandreply chk")

        if not self.do_sync_send():
            return False

        self.cksum.reset()
        self.tx_with_checksum(data_len & 0xFF)
        self.tx_with_checksum(self.msg_id)
        self.flush_log("checkandreply hdr")
        return True

    # --- Startup ---

    def send_startup_burst(self):
        """Send 256 × 0xFF to flush host receiver (mon_start1)."""
        log.info("Sending startup burst (256 x 0xFF)...")
        self.ser.write(b'\xFF' * 256)
        self.ser.flush()
        log.debug("[TX] FF x256  (startup burst)")

    # --- Main loop ---

    def send_status(self):
        """mon_mainlp2: send status message to host.

        TX: 0x00, 0x20 (sync)
        RX: expect 0x3F
        TX: size(0), msgID, checksum_hi, checksum_lo
        Returns True if host responded correctly.
        """
        self.tx(0x00)
        self.tx(0x20)
        self.flush_log("status sync")

        b = self.rx()
        self.flush_log("host sync")
        if b != 0x3F:
            log.warning("  Status: expected 0x3F, got 0x%02X — retry", b)
            return False

        self.cksum.reset()
        size = 0x00
        self.cksum.add(size)
        self.tx(size)
        self.cksum.add(self.msg_id)
        self.tx(self.msg_id)
        self.tx(self.cksum.high)
        self.tx(self.cksum.low)
        self.flush_log("status msg (id=0x%02X)" % self.msg_id)
        return True

    def receive_command(self):
        """mon_mainlp: wait for and parse host command.

        Returns dict with command info, or None on sync/checksum failure.
        """
        # Wait for host sync byte 0x20
        self.cksum.reset()
        while True:
            b = self.rx()
            if b == 0x20:
                break
            # Assembly loops on mon_mainlp waiting for 0x20
            if self.verbose:
                log.debug("  mainlp: skip byte 0x%02X (waiting for 0x20)", b)

        self.tx(0x3F)  # acknowledge
        self.flush_log("cmd sync")

        # Read header: size, cmd_id, cmd_type
        size_byte = self.rx_with_checksum()
        cmd_byte = self.rx_with_checksum()
        cmd_type = self.rx_with_checksum()
        self.msg_id = cmd_byte & 0x0F

        self.flush_log("cmd hdr (size=%d cmd=0x%02X type=%d id=%d)" %
                        (size_byte, cmd_byte, cmd_type, self.msg_id))

        # NOP or GoRAM — no further parameters
        if cmd_type == 0 or cmd_type == 7:
            return {"type": cmd_type, "msg_id": self.msg_id}

        # Out of range
        if cmd_type > 7:
            log.warning("  cmd_type %d out of range — ignore", cmd_type)
            return None

        # Read standard parameters (cmd_type 1-6)
        access_class = self.rx_with_checksum()
        access_width = self.rx_with_checksum()

        addr = 0
        for _ in range(4):
            b = self.rx_with_checksum()
            addr = (addr << 8) | b

        data_len = self.rx_with_checksum()
        self.flush_log("cmd params (class=%d width=%d addr=0x%06X len=%d)" %
                        (access_class, access_width, addr, data_len))

        if data_len == 0xFF:
            log.warning("  data_len=0xFF — invalid, ignore")
            return None

        # Parameter checksum verification
        param_chk = self.rx()
        expected = self.cksum.low
        if param_chk != expected:
            log.warning("  PARAM CHECKSUM FAIL: got 0x%02X, expected 0x%02X",
                        param_chk, expected)
            self.flush_log("param_chk FAIL")
            return None
        self.cksum.add(param_chk)
        self.flush_log("param_chk OK")

        return {
            "type": cmd_type,
            "msg_id": self.msg_id,
            "access_class": access_class,
            "access_width": access_width,
            "addr": addr,
            "data_len": data_len,
        }

    # --- Command Handlers ---

    def handle_read_memory(self, cmd):
        """Cmd 1: Read Memory (rd path in monitor.S)."""
        addr = cmd["addr"]
        length = cmd["data_len"]
        log.info("READ MEM: addr=0x%06X len=%d", addr, length)

        # Verify data checksum (empty data section — just 2 checksum bytes)
        if not self.verify_data_checksum():
            return

        # Sync handshake
        if not self.do_sync_send():
            return

        # Send response header
        self.cksum.reset()
        self.tx_with_checksum(length & 0xFF)    # data length
        self.tx_with_checksum(self.msg_id)       # message ID

        # Send memory data (byte mode — simplest, always works)
        for i in range(length):
            b = self.mem_read(addr + i)
            self.tx_with_checksum(b)

        self.flush_log("read_mem data (%d bytes from 0x%06X)" % (length, addr))

        # Checksum footer (mon_mainlp1)
        self.send_checksum_footer()
        self.flush_log("read_mem chk footer")

    def handle_write_memory(self, cmd):
        """Cmd 2: Write Memory (wr path in monitor.S)."""
        addr = cmd["addr"]
        length = cmd["data_len"]
        log.info("WRITE MEM: addr=0x%06X len=%d", addr, length)

        # Read data bytes (byte mode)
        for i in range(length):
            b = self.rx_with_checksum()
            self.mem_write(addr + i, b)

        self.flush_log("write_mem data (%d bytes to 0x%06X)" % (length, addr))

        # Verify data checksum (mon_mainlp3)
        if not self.verify_data_checksum():
            log.warning("  write_mem checksum fail")
            return
        self.flush_log("write_mem chk OK")

        # Return to mon_mainlp2 (send status on next iteration)

    def handle_read_registers(self, cmd):
        """Cmd 3: Read Registers (readreg path in monitor.S)."""
        data_len = cmd["data_len"]
        log.info("READ REGS: len=%d", data_len)

        # checkandreply: verify data checksum, sync, send header
        if not self.checkandreply(data_len):
            return

        if data_len != 36:
            # readreg01: non-destructive — send ER7, MACH(0), MACL(0) = 12 bytes
            log.info("  readreg01: ER7 + dummy MACH + dummy MACL")
            self.send_long(self.regs.get(0))   # ER7/SP
            self.send_long(self.regs.get(1))   # MACH (dummy 0 for 2144)
            self.send_long(self.regs.get(2))   # MACL (dummy 0 for 2144)
        else:
            # readregpc: send ER6,ER5,ER4,ER3,ER2,ER1,ER0,EXR(dummy),CCR+PC = 36 bytes
            log.info("  readregpc: all registers (36 bytes)")
            self.send_long(self.regs.get(3))   # ER6
            self.send_long(self.regs.get(4))   # ER5
            self.send_long(self.regs.get(5))   # ER4
            self.send_long(self.regs.get(6))   # ER3
            self.send_long(self.regs.get(7))   # ER2
            self.send_long(self.regs.get(8))   # ER1
            self.send_long(self.regs.get(9))   # ER0
            self.send_long(self.regs.get(10))  # EXR (dummy 0 for 2144)
            self.send_long(self.regs.get(11))  # CCR+PC

        self.flush_log("read_regs data")

        # Checksum footer (mon_mainlp1)
        self.send_checksum_footer()
        self.flush_log("read_regs chk footer")

    def handle_write_register(self, cmd):
        """Cmd 4: Write Register (writereg path in monitor.S)."""
        data_len = cmd["data_len"]
        reg_num = cmd["addr"]  # ER2 = register number in protocol
        log.info("WRITE REG: reg=%d (%s) len=%d",
                 reg_num, REG_NAMES[reg_num] if reg_num < 12 else "?", data_len)

        if data_len == 0:
            # No data — just verify checksum (mon_mainlp3)
            if not self.verify_data_checksum():
                return
            self.flush_log("write_reg len=0 chk OK")
            return

        if data_len != 4:
            # Illegal packet length — resync
            log.warning("  write_reg: illegal length %d (expected 4)", data_len)
            return

        # Read 4-byte register value
        value = self.recv_long()
        self.flush_log("write_reg value=0x%08X" % value)

        # Store if in range
        if reg_num < 12:
            self.regs.set(reg_num, value)
        else:
            log.warning("  write_reg: reg %d out of range, ignoring", reg_num)

        # Verify data checksum (mon_mainlp3)
        if not self.verify_data_checksum():
            log.warning("  write_reg checksum fail")
            return
        self.flush_log("write_reg chk OK")

    def handle_go(self, cmd):
        """Cmd 5/6: Go or Step (mon_go path in monitor.S).

        Receives register values, then "executes" — we simulate an
        immediate breakpoint and return to the monitor main loop.
        """
        cmd_type = cmd["type"]
        cmd_name = "GO" if cmd_type == 5 else "STEP"
        log.info("%s command", cmd_name)

        # Read 9 register values (CCR+PC, EXR, ER0-ER6) via readlong
        ccr_pc = self.recv_long()
        exr_val = self.recv_long()
        er0 = self.recv_long()
        er1 = self.recv_long()
        er2 = self.recv_long()
        er3 = self.recv_long()
        er4 = self.recv_long()
        er5 = self.recv_long()
        er6 = self.recv_long()

        self.flush_log("go reg data (9 longs)")

        # Verify data checksum
        if not self.verify_data_checksum():
            log.warning("  %s checksum fail — error", cmd_name)
            return
        self.flush_log("go chk OK")

        # Store register values
        self.regs.set(11, ccr_pc)   # CCR+PC
        self.regs.set(10, exr_val)  # EXR
        self.regs.set(9, er0)       # ER0
        self.regs.set(8, er1)       # ER1
        self.regs.set(7, er2)       # ER2
        self.regs.set(6, er3)       # ER3
        self.regs.set(5, er4)       # ER4
        self.regs.set(4, er5)       # ER5
        self.regs.set(3, er6)       # ER6

        pc = ccr_pc & 0xFFFFFF
        ccr = (ccr_pc >> 24) & 0xFF
        log.info("  %s → PC=0x%06X CCR=0x%02X", cmd_name, pc, ccr)

        # Send acknowledgement (sync + status)
        if not self.do_sync_send():
            return

        self.cksum.reset()
        self.tx_with_checksum(0x00)           # size = 0
        self.tx_with_checksum(self.msg_id)    # message ID
        self.tx(self.cksum.high)              # checksum high
        self.tx(self.cksum.low)               # checksum low
        self.flush_log("go ack")

        # Simulate immediate breakpoint — set msg_id to breakpoint reason
        log.info("  Simulating immediate breakpoint (returning to monitor)")
        self.msg_id = 0x10  # breakpoint reason code

    def handle_nop_goram(self, cmd):
        """Cmd 0 (NOP) / Cmd 7 (GoRAM) — checkandreply, then return."""
        cmd_type = cmd["type"]
        cmd_name = "NOP" if cmd_type == 0 else "GoRAM"
        log.info("%s command", cmd_name)

        # For NOP/GoRAM, data_len = 0 (no parameters read yet)
        # The goram path does: checkandreply with r1=0, then sends checksum footer
        if not self.checkandreply(0):
            return

        # Send checksum footer
        self.send_checksum_footer()
        self.flush_log("%s done" % cmd_name)

    # --- Top-level loop ---

    def run(self):
        """Main monitor loop.

        Matches the control flow of monitor.S exactly:

        mon_mainlp2 (status) → mon_mainlp1 (chk footer) → mon_mainlp (cmd wait)

        After read commands (1,3) and NOP/GoRAM (0,7):
            → mon_mainlp1 → mon_mainlp  (NO status before next cmd)
        After write commands (2,4):
            → mon_mainlp3 → mon_mainlp2 (status, THEN next cmd)
        After Go/Step (5,6):
            → breakpoint re-entry → mon_mainlp2 (status with reason)
        """
        log.info("=" * 60)
        log.info("Trace32 Monitor Simulator starting")
        log.info("Port: %s  Baud: %d", self.ser.port, self.ser.baudrate)
        log.info("Memory: %d bytes loaded", len(self.memory))
        log.info("=" * 60)

        # Startup burst
        self.send_startup_burst()

        # Flush any pending input
        self.ser.reset_input_buffer()

        # State: "status" = mon_mainlp2 (send status first)
        #        "command" = mon_mainlp (wait for host command directly)
        next_state = "status"

        while True:
            try:
                # mon_mainlp2: send status message (only if state requires it)
                if next_state == "status":
                    if not self.send_status():
                        # Sync failed — real monitor goes to mon_mainlp
                        next_state = "command"
                        continue
                    # Status sent; falls through to mon_mainlp1 (checksum)
                    # then to mon_mainlp (command wait) — handled inline above

                # mon_mainlp: receive and parse command
                next_state = "command"  # default: stay in command loop
                cmd = self.receive_command()
                if cmd is None:
                    log.warning("Command parse failed — back to mainlp")
                    continue

                cmd_type = cmd["type"]

                # Dispatch and determine next state
                if cmd_type == 0 or cmd_type == 7:
                    self.handle_nop_goram(cmd)
                    next_state = "command"   # → mon_mainlp
                elif cmd_type == 1:
                    self.handle_read_memory(cmd)
                    next_state = "command"   # → mon_mainlp1 → mon_mainlp
                elif cmd_type == 2:
                    self.handle_write_memory(cmd)
                    next_state = "status"    # → mon_mainlp3 → mon_mainlp2
                elif cmd_type == 3:
                    self.handle_read_registers(cmd)
                    next_state = "command"   # → mon_mainlp1 → mon_mainlp
                elif cmd_type == 4:
                    self.handle_write_register(cmd)
                    next_state = "status"    # → mon_mainlp3 → mon_mainlp2
                elif cmd_type in (5, 6):
                    self.handle_go(cmd)
                    next_state = "status"    # breakpoint → mon_mainlp2
                else:
                    log.warning("Unknown command type %d — ignoring", cmd_type)

            except serial.SerialTimeoutException:
                # Read timeout — just loop back
                continue
            except KeyboardInterrupt:
                log.info("Interrupted — exiting")
                break
            except Exception:
                log.exception("Unhandled exception in main loop")
                time.sleep(0.5)


# ---------------------------------------------------------------------------
# Protocol-Aware Proxy
# ---------------------------------------------------------------------------

# Command type names for logging
CMD_NAMES = {0: "NOP", 1: "ReadMem", 2: "WriteMem", 3: "ReadRegs",
             4: "WriteRegs", 5: "Go", 6: "Step", 7: "GoRAM"}


class MonitorProxy:
    """Transparent proxy between Trace32 (host) and real hardware (target).

    Forwards all bytes bidirectionally while parsing the protocol state
    machine.  Logs decoded commands, detects checksum mismatches, sync
    loss, and timeouts.

    Architecture:
      Trace32 ←(host_ser)→ [MonitorProxy] ←(target_ser)→ Real H8S Hardware

    The proxy NEVER injects or modifies bytes — it only observes.
    """

    def __init__(self, host_ser, target_ser, verbose=False):
        self.host = host_ser       # Trace32 side (via com0com virtual pair)
        self.target = target_ser   # Real hardware side
        self.verbose = verbose
        self.cksum = Checksum()

        # Protocol state machine
        self.state = "idle"        # see _state_map
        self.msg_id = 0
        self.cmd_type = 0
        self.data_len = 0
        self.addr = 0
        self.access_width = 0
        self._param_bytes = []     # accumulate parameter bytes
        self._data_count = 0       # bytes remaining in data phase

        # Desync detection
        self._last_activity = time.time()
        self._host_byte_count = 0
        self._target_byte_count = 0
        self._sync_errors = 0
        self._checksum_errors = 0
        self._timeout_count = 0
        self._idle_warned = False
        self._breakin_count = 0    # consecutive 0xFF break-in bytes from host
        self._breakin_rounds = 0   # complete break-in sequences (burst + reset)
        self._fw_traffic_count = 0 # firmware serial bytes received during protocol
        self._fw_traffic_buf = []  # small buffer for logging first bytes

    # --- Logging helpers ---

    def _log_byte(self, direction, byte_val, annotation=""):
        """Log a single byte with direction and optional annotation."""
        arrow = "H→T" if direction == "host" else "T→H"
        ann = f"  ({annotation})" if annotation else ""
        if self.verbose:
            log.debug("[%s] %02X%s", arrow, byte_val, ann)

    def _log_event(self, msg, *args, level=logging.INFO):
        """Log a protocol event."""
        log.log(level, "PROXY: " + msg, *args)

    def _log_desync(self, msg, *args):
        """Log a desync/error condition prominently."""
        self._sync_errors += 1
        log.warning("*** DESYNC #%d: " + msg, self._sync_errors, *args)

    def _flush_fw_traffic(self):
        """Log accumulated firmware serial traffic and reset the counter."""
        if self._fw_traffic_count > 0:
            preview = " ".join("%02X" % b for b in self._fw_traffic_buf[:16])
            if self._fw_traffic_count > 16:
                preview += " ..."
            self._log_event("Firmware traffic: %d bytes [%s]",
                            self._fw_traffic_count, preview)
            self._fw_traffic_count = 0
            self._fw_traffic_buf = []

    def _accum_fw_traffic(self, b):
        """Accumulate a firmware serial traffic byte for consolidated logging."""
        self._fw_traffic_count += 1
        if len(self._fw_traffic_buf) < 16:
            self._fw_traffic_buf.append(b)

    # --- Byte forwarding ---

    def _forward_host_to_target(self, b):
        """Forward one byte from Trace32 to hardware."""
        self.target.write(bytes([b]))
        self._host_byte_count += 1
        self._last_activity = time.time()

    def _forward_target_to_host(self, b):
        """Forward one byte from hardware to Trace32."""
        self.host.write(bytes([b]))
        self._target_byte_count += 1
        self._last_activity = time.time()

    # --- Protocol state machine ---
    #
    # States track where we are in the protocol conversation.
    # The proxy observes bytes flowing in BOTH directions and
    # follows the expected protocol sequence.
    #
    # State names match monitor.S labels where possible:
    #
    #   "idle"              — waiting for first activity (startup)
    #   "startup_burst"     — target sending 0xFF burst
    #   "status_byte"       — expect target TX 0x00 (mon_mainlp2)
    #   "status_sync"       — expect target TX 0x20
    #   "status_host_sync"  — expect host TX 0x3F
    #   "status_size"       — expect target TX size byte
    #   "status_msgid"      — expect target TX msgID
    #   "status_chk_hi"     — expect target TX checksum high
    #   "status_chk_lo"     — expect target TX checksum low
    #   "cmd_wait_sync"     — expect host TX 0x20 (mon_mainlp)
    #   "cmd_target_ack"    — expect target TX 0x3F
    #   "cmd_header"        — reading 3-byte command header from host
    #   "cmd_params"        — reading parameters from host (cmd 1-6)
    #   "cmd_param_chk"     — reading parameter checksum from host
    #   "cmd_data_host"     — host sending data (WriteMem, WriteRegs, Go)
    #   "cmd_data_chk_h"    — host data checksum high
    #   "cmd_data_chk_l"    — host data checksum low
    #   "resp_sync"         — expect target TX 0x20 (response sync)
    #   "resp_host_ack"     — expect host TX 0x3F
    #   "resp_header"       — target sending response header (size, msgID)
    #   "resp_data"         — target sending response data
    #   "resp_chk_hi"       — target TX response checksum high
    #   "resp_chk_lo"       — target TX response checksum low
    #   "go_running"        — target executing user code after Go/Step

    def _transition(self, new_state, msg=""):
        """Transition to a new protocol state."""
        old = self.state
        self.state = new_state
        if self.verbose:
            log.debug("PROXY: state %s → %s  %s", old, new_state, msg)

    # States where ONLY host bytes are expected (target should be silent).
    # Any target bytes in these states are firmware serial traffic on SCI1.
    _HOST_EXPECT_STATES = frozenset({
        "cmd_wait_sync",      # waiting for host 0x20 command sync
        "cmd_header",         # host sending 3-byte command header
        "cmd_params",         # host sending parameters
        "cmd_param_chk",      # host sending parameter checksum
        "cmd_data_host",      # host sending data (WriteMem/WriteRegs/Go)
        "cmd_data_chk_h",     # host sending data checksum high
        "cmd_data_chk_l",     # host sending data checksum low
        "go_data_chk_h",      # host sending Go data checksum high
        "go_data_chk_l",      # host sending Go data checksum low
        "wr_data_chk_hi",     # host sending write data checksum high
        "wr_data_chk_lo",     # host sending write data checksum low
        "status_host_sync",   # waiting for host 0x3F after status sync
        "resp_host_ack",      # waiting for host 0x3F after response sync
    })

    def on_target_byte(self, b):
        """Process one byte sent FROM target (hardware) TO host (Trace32)."""
        self._forward_target_to_host(b)
        self._idle_warned = False

        # --- Firmware serial traffic detection ---
        # In states where only the host should be sending, any target byte
        # is firmware SCI1 output leaking through.  Accumulate and log later.
        if self.state in self._HOST_EXPECT_STATES:
            self._accum_fw_traffic(b)
            return

        # If we were accumulating firmware traffic and now get a valid
        # protocol byte, flush the accumulated count first.
        if self._fw_traffic_count > 0:
            self._flush_fw_traffic()

        if self.state == "idle":
            if b == 0xFF:
                self._log_event("Startup burst detected (first 0xFF)")
                self._transition("startup_burst")
                self._data_count = 1
            else:
                self._log_event("Target sent 0x%02X in idle (expected 0xFF burst)", b)

        elif self.state == "startup_burst":
            self._data_count += 1
            if b != 0xFF:
                self._log_event("Startup burst ended after %d bytes (non-0xFF: 0x%02X)",
                                self._data_count - 1, b)
                # Could be the 0x00 of first status
                if b == 0x00:
                    self._transition("status_sync", "got 0x00, expect 0x20 next")
                else:
                    self._transition("cmd_wait_sync", "unexpected byte after burst")
            elif self._data_count >= 256:
                self._log_event("Startup burst complete (256 bytes)")
                self._transition("status_byte")

        elif self.state == "status_byte":
            if b == 0x00:
                self._log_byte("target", b, "status byte OK")
                self._transition("status_sync")
            else:
                self._log_desync("Expected status 0x00, got 0x%02X", b)
                self._transition("cmd_wait_sync")

        elif self.state == "status_sync":
            if b == 0x20:
                self._log_byte("target", b, "monitor sync 0x20")
                self._transition("status_host_sync")
            else:
                self._log_desync("Expected sync 0x20 from target, got 0x%02X", b)
                self._transition("cmd_wait_sync")

        elif self.state == "status_size":
            self.cksum.reset()
            self.cksum.add(b)
            self._log_byte("target", b, "status size=%d" % b)
            self._transition("status_msgid")

        elif self.state == "status_msgid":
            self.cksum.add(b)
            self.msg_id = b
            reason = ""
            if b == 0x00:
                reason = "initial/NOP"
            elif b == 0x10:
                reason = "BREAKPOINT"
            elif b == 0x30:
                reason = "TRAP"
            self._log_event("Status: msgID=0x%02X %s", b, reason)
            self._transition("status_chk_hi")

        elif self.state == "status_chk_hi":
            expected = self.cksum.high
            if b != expected:
                self._log_desync("Status checksum high: got 0x%02X, expected 0x%02X",
                                 b, expected)
            self._transition("status_chk_lo")

        elif self.state == "status_chk_lo":
            expected = self.cksum.low
            if b != expected:
                self._log_desync("Status checksum low: got 0x%02X, expected 0x%02X",
                                 b, expected)
            self._log_event("Status complete → waiting for host command")
            self._transition("cmd_wait_sync")

        elif self.state == "cmd_target_ack":
            if b == 0x3F:
                self._log_byte("target", b, "target ack 0x3F")
                self._transition("cmd_header")
                self._param_bytes = []
            else:
                self._log_desync("Expected target ack 0x3F, got 0x%02X", b)
                self._transition("cmd_wait_sync")

        elif self.state == "resp_sync":
            if b == 0x20:
                self._log_byte("target", b, "response sync 0x20")
                self._transition("resp_host_ack")
            else:
                self._log_desync("Expected response sync 0x20, got 0x%02X", b)
                self._transition("cmd_wait_sync")

        elif self.state == "resp_header":
            self._param_bytes.append(b)
            self.cksum.add(b)
            if len(self._param_bytes) == 1:
                self._log_byte("target", b, "resp size=%d" % b)
                self._data_count = b  # how many data bytes to expect
            elif len(self._param_bytes) == 2:
                self._log_byte("target", b, "resp msgID=0x%02X" % b)
                if self._data_count > 0:
                    self._param_bytes = []
                    self._transition("resp_data", "expecting %d data bytes" % self._data_count)
                else:
                    # No data — go straight to checksum
                    self._transition("resp_chk_hi")

        elif self.state == "resp_data":
            self.cksum.add(b)
            self._data_count -= 1
            if self._data_count <= 0:
                self._log_event("Response data complete")
                self._transition("resp_chk_hi")

        elif self.state == "resp_chk_hi":
            ok = (b == self.cksum.high)
            if not ok:
                self._log_desync("Response checksum high: got 0x%02X, expected 0x%02X",
                                 b, self.cksum.high)
                self._checksum_errors += 1
            self._transition("resp_chk_lo")

        elif self.state == "resp_chk_lo":
            ok = (b == self.cksum.low)
            if not ok:
                self._log_desync("Response checksum low: got 0x%02X, expected 0x%02X",
                                 b, self.cksum.low)
                self._checksum_errors += 1
            if self.cmd_type in (5, 6):
                # Go/Step ack complete — target is now running user code
                self._log_event("Go/Step ack complete → TARGET RUNNING (user code)")
                self._transition("go_running")
            else:
                # After read response → mon_mainlp (no status)
                self._log_event("Response checksum complete → waiting for host command")
                self._transition("cmd_wait_sync")

        elif self.state == "go_running":
            # Target re-entered monitor (breakpoint/trap/reset) — startup burst
            if b == 0xFF:
                self._log_event("Target re-entered monitor (0xFF burst detected)")
                self._transition("startup_burst")
                self._data_count = 1
            elif b == 0x00:
                # Skip burst, went straight to status (fast re-entry)
                self._log_event("Target re-entered monitor (status byte, no burst)")
                self._transition("status_sync")
            else:
                self._log_event("Target sent 0x%02X while running (unexpected)", b)

        else:
            if self.verbose:
                self._log_byte("target", b, "UNTRACKED in state=%s" % self.state)

    def on_host_byte(self, b):
        """Process one byte sent FROM host (Trace32) TO target (hardware)."""
        self._forward_host_to_target(b)
        self._idle_warned = False

        if self.state == "status_host_sync":
            if b == 0x3F:
                self._flush_fw_traffic()
                self._log_byte("host", b, "host sync 0x3F")
                self._transition("status_size")
            else:
                self._flush_fw_traffic()
                self._log_desync("Expected host sync 0x3F, got 0x%02X", b)
                self._transition("cmd_wait_sync")

        elif self.state == "cmd_wait_sync":
            if b == 0x20:
                self._flush_fw_traffic()
                self._log_byte("host", b, "host command sync 0x20")
                self._transition("cmd_target_ack")
            else:
                if self.verbose:
                    self._log_byte("host", b, "waiting for 0x20 (ignored)")

        elif self.state == "cmd_target_ack":
            # Host sending while we wait for target ack — target is unresponsive.
            # Trace32 break-in pattern: 256 × 0xFF, then 0x00 (flush), repeat.
            # After several rounds, it may send 0x20 to retry the command sync.
            if b == 0xFF:
                self._breakin_count += 1
                if self._breakin_count == 1 and self._breakin_rounds == 0:
                    self._log_event("Host sending break-in (0xFF) — target unresponsive")
            elif b == 0x00:
                # 0x00 after 0xFF burst = Trace32 flush/reset between break-in rounds
                if self._breakin_count > 0:
                    self._breakin_rounds += 1
                    if self._breakin_rounds <= 3 or self._breakin_rounds % 10 == 0:
                        self._log_event("  Break-in round %d: %d x 0xFF + flush",
                                        self._breakin_rounds, self._breakin_count)
                    self._breakin_count = 0
                # Stay in cmd_target_ack — still waiting for target 0x3F
            elif b == 0x20:
                # Trace32 retrying command sync
                if self._breakin_count > 0 or self._breakin_rounds > 0:
                    self._log_event("  Break-in ended after %d rounds, "
                                    "host retrying sync (0x20)",
                                    self._breakin_rounds)
                    self._breakin_count = 0
                    self._breakin_rounds = 0
                # Stay in cmd_target_ack — still waiting for target 0x3F
            else:
                if self._breakin_count > 0 or self._breakin_rounds > 0:
                    self._log_event("  Break-in: %d rounds + %d x 0xFF, "
                                    "then host sent 0x%02X",
                                    self._breakin_rounds, self._breakin_count, b)
                    self._breakin_count = 0
                    self._breakin_rounds = 0

        elif self.state == "go_running":
            # Host sending while target runs — typically break-in attempt.
            # Same pattern: 256 × 0xFF + 0x00 flush, repeated.
            if b == 0xFF:
                self._breakin_count += 1
                if self._breakin_count == 1 and self._breakin_rounds == 0:
                    self._log_event("Host sending break-in (0xFF) — target running")
            elif b == 0x00:
                if self._breakin_count > 0:
                    self._breakin_rounds += 1
                    if self._breakin_rounds <= 3 or self._breakin_rounds % 10 == 0:
                        self._log_event("  Break-in round %d: %d x 0xFF + flush",
                                        self._breakin_rounds, self._breakin_count)
                    self._breakin_count = 0
            elif b == 0x20:
                if self._breakin_count > 0 or self._breakin_rounds > 0:
                    self._log_event("  Break-in ended after %d rounds, "
                                    "host trying cmd sync (0x20)",
                                    self._breakin_rounds)
                    self._breakin_count = 0
                    self._breakin_rounds = 0
                # Transition: Trace32 gave up on break-in, trying command
                self._transition("cmd_target_ack")
            else:
                if self._breakin_count > 0 or self._breakin_rounds > 0:
                    self._log_event("  Break-in: %d rounds + %d x 0xFF, then 0x%02X",
                                    self._breakin_rounds, self._breakin_count, b)
                    self._breakin_count = 0
                    self._breakin_rounds = 0

        elif self.state == "cmd_header":
            self._param_bytes.append(b)
            self.cksum.add(b)
            if len(self._param_bytes) == 1:
                self._log_byte("host", b, "cmd size=%d" % b)
            elif len(self._param_bytes) == 2:
                self.msg_id = b & 0x0F
                self._log_byte("host", b, "cmd byte=0x%02X (id=%d)" % (b, self.msg_id))
            elif len(self._param_bytes) == 3:
                self.cmd_type = b
                name = CMD_NAMES.get(b, "UNKNOWN(%d)" % b)
                self._log_event("Command: type=%d (%s) msgID=%d", b, name, self.msg_id)

                if b == 0 or b == 7:
                    # NOP/GoRAM: no parameters, next is data checksum
                    self._transition("cmd_data_chk_h",
                                     "NOP/GoRAM → expect data checksum")
                elif b > 7:
                    self._log_desync("cmd_type %d out of range", b)
                    self._transition("cmd_wait_sync")
                else:
                    # cmd 1-6: read more parameters
                    self._param_bytes = []
                    self._transition("cmd_params")

        elif self.state == "cmd_params":
            self._param_bytes.append(b)
            self.cksum.add(b)
            # Parameters: access_class(1) + access_width(1) + addr(4) + data_len(1) = 7 bytes
            if len(self._param_bytes) == 7:
                self.access_width = self._param_bytes[1]
                self.addr = ((self._param_bytes[2] << 24) | (self._param_bytes[3] << 16) |
                             (self._param_bytes[4] << 8) | self._param_bytes[5])
                self.data_len = self._param_bytes[6]
                name = CMD_NAMES.get(self.cmd_type, "?")
                self._log_event("  %s: addr=0x%06X len=%d width=%d",
                                name, self.addr, self.data_len, self.access_width)
                if self.data_len == 0xFF:
                    self._log_desync("data_len=0xFF (invalid)")
                    self._transition("cmd_wait_sync")
                else:
                    self._transition("cmd_param_chk")

        elif self.state == "cmd_param_chk":
            expected = self.cksum.low
            if b != expected:
                self._log_desync("Param checksum: got 0x%02X, expected 0x%02X", b, expected)
                self._checksum_errors += 1
                self._transition("cmd_wait_sync")
                return
            self.cksum.add(b)
            self._log_event("  Param checksum OK (0x%02X)", b)
            self._dispatch_after_param_chk()

        elif self.state == "cmd_data_chk_h":
            # Data checksum high byte from host (NOP/GoRAM or read commands)
            ok = (b ^ self.cksum.high) == 0
            if not ok:
                self._log_desync("Data checksum high: 0x%02X XOR 0x%02X != 0",
                                 b, self.cksum.high)
                self._checksum_errors += 1
            self._transition("cmd_data_chk_l")

        elif self.state == "cmd_data_chk_l":
            ok = (b ^ self.cksum.low) == 0
            if not ok:
                self._log_desync("Data checksum low: 0x%02X XOR 0x%02X != 0",
                                 b, self.cksum.low)
                self._checksum_errors += 1
            # After data checksum verified, target will send response
            if self.cmd_type in (0, 7):
                # NOP/GoRAM → checkandreply → target sends sync
                self._log_event("  Data checksum OK → expect response sync from target")
                self._transition("resp_sync")
            elif self.cmd_type == 1:
                # ReadMem → target sends sync + data
                self._log_event("  Data checksum OK → expect ReadMem response from target")
                self._transition("resp_sync")
            elif self.cmd_type == 3:
                # ReadRegs → checkandreply → target sends sync
                self._log_event("  Data checksum OK → expect ReadRegs response from target")
                self._transition("resp_sync")
            else:
                self._log_event("  Data checksum OK")
                self._transition("cmd_wait_sync")

        elif self.state == "cmd_data_host":
            # Host sending data bytes (WriteMem, WriteRegs value, Go regs)
            self.cksum.add(b)
            self._data_count -= 1
            if self._data_count <= 0:
                name = CMD_NAMES.get(self.cmd_type, "?")
                self._log_event("  %s: all data bytes received from host", name)
                if self.cmd_type == 2:
                    # WriteMem → data checksum verified by target
                    # Target reads checksum bytes then goes to mon_mainlp2
                    self._transition("wr_data_chk_hi")
                elif self.cmd_type == 4:
                    # WriteRegs → mon_mainlp3 → data checksum
                    self._transition("wr_data_chk_hi")
                elif self.cmd_type in (5, 6):
                    # Go/Step → data checksum, then target sends ack
                    self._transition("go_data_chk_h")
                else:
                    self._transition("cmd_wait_sync")

        elif self.state == "go_data_chk_h":
            ok = (b ^ self.cksum.high) == 0
            if not ok:
                self._log_desync("Go data checksum high: 0x%02X XOR 0x%02X != 0",
                                 b, self.cksum.high)
                self._checksum_errors += 1
            self._transition("go_data_chk_l")

        elif self.state == "go_data_chk_l":
            ok = (b ^ self.cksum.low) == 0
            if not ok:
                self._log_desync("Go data checksum low: 0x%02X XOR 0x%02X != 0",
                                 b, self.cksum.low)
                self._checksum_errors += 1
            # Go/Step: target sends sync+ack, then executes
            self._log_event("  Go/Step checksum OK → expect ack from target")
            self._transition("resp_sync")

        elif self.state == "wr_data_chk_hi":
            # Write data checksum high — HOST sends to TARGET (mon_mainlp3)
            ok = (b ^ self.cksum.high) == 0
            if not ok:
                self._log_desync("Write data checksum high: 0x%02X XOR 0x%02X != 0",
                                 b, self.cksum.high)
                self._checksum_errors += 1
            self._transition("wr_data_chk_lo")

        elif self.state == "wr_data_chk_lo":
            ok = (b ^ self.cksum.low) == 0
            if not ok:
                self._log_desync("Write data checksum low: 0x%02X XOR 0x%02X != 0",
                                 b, self.cksum.low)
                self._checksum_errors += 1
            # After write → mon_mainlp2 (target sends status)
            self._log_event("Write checksum verified → expect status from target")
            self._transition("status_byte")

        elif self.state == "resp_host_ack":
            if b == 0x3F:
                self._log_byte("host", b, "host response ack 0x3F")
                self.cksum.reset()
                self._param_bytes = []
                self._transition("resp_header")
            else:
                self._log_desync("Expected host ack 0x3F, got 0x%02X", b)
                self._transition("cmd_wait_sync")

        else:
            if self.verbose:
                self._log_byte("host", b, "UNTRACKED in state=%s" % self.state)

    def _dispatch_after_param_chk(self):
        """After parameter checksum verified, set up state for command-specific data."""
        if self.cmd_type == 1:
            # ReadMem: host sends data checksum (empty data), then target responds
            self._transition("cmd_data_chk_h", "ReadMem → expect data checksum from host")

        elif self.cmd_type == 2:
            # WriteMem: host sends data_len bytes, then data checksum
            self._data_count = self.data_len
            self._log_event("  WriteMem: expecting %d data bytes from host", self.data_len)
            if self.data_len == 0:
                self._transition("wr_data_chk_hi")
            else:
                self._transition("cmd_data_host")

        elif self.cmd_type == 3:
            # ReadRegs: host sends data checksum (empty), target responds via checkandreply
            self._transition("cmd_data_chk_h", "ReadRegs → expect data checksum from host")

        elif self.cmd_type == 4:
            # WriteRegs: host sends data_len bytes (0 or 4), then data checksum
            self._data_count = self.data_len
            if self.data_len == 0:
                self._log_event("  WriteRegs: len=0 → expect data checksum")
                self._transition("wr_data_chk_hi")
            elif self.data_len == 4:
                self._log_event("  WriteRegs: reg=%d (%s), expecting 4 value bytes",
                                self.addr,
                                REG_NAMES[self.addr] if self.addr < 12 else "?")
                self._transition("cmd_data_host")
            else:
                self._log_desync("WriteRegs: invalid data_len=%d (expected 0 or 4)",
                                 self.data_len)
                self._transition("cmd_wait_sync")

        elif self.cmd_type in (5, 6):
            # Go/Step: host sends 36 bytes (9 regs), then data checksum
            self._data_count = 36
            name = "Go" if self.cmd_type == 5 else "Step"
            self._log_event("  %s: expecting 36 register bytes from host", name)
            self._transition("cmd_data_host")

        else:
            self._log_event("  cmd_type=%d: no data expected", self.cmd_type)
            self._transition("cmd_wait_sync")

    # --- Idle/timeout detection ---

    def check_timeout(self, timeout=3.0):
        """Check if communication has stalled. Call periodically."""
        elapsed = time.time() - self._last_activity
        if elapsed > timeout and not self._idle_warned:
            self._flush_fw_traffic()
            self._idle_warned = True
            self._timeout_count += 1
            log.warning("*** TIMEOUT #%d: No activity for %.1fs (state=%s) — possible hang",
                        self._timeout_count, elapsed, self.state)
            return True
        return False

    def print_stats(self):
        """Print summary statistics."""
        self._flush_fw_traffic()  # flush any remaining
        log.info("=" * 60)
        log.info("Proxy session summary:")
        log.info("  Host→Target bytes:  %d", self._host_byte_count)
        log.info("  Target→Host bytes:  %d", self._target_byte_count)
        log.info("  Sync errors:        %d", self._sync_errors)
        log.info("  Checksum errors:    %d", self._checksum_errors)
        log.info("  Timeouts:           %d", self._timeout_count)
        log.info("  Break-in rounds:    %d", self._breakin_rounds)
        log.info("  Final state:        %s", self.state)
        log.info("=" * 60)

    # --- Main proxy loop ---

    def run(self):
        """Run the proxy, forwarding bytes and tracking protocol."""
        self._log_event("=" * 55)
        self._log_event("Proxy mode: %s (host) ↔ %s (target)", self.host.port, self.target.port)
        self._log_event("Waiting for target startup burst...")
        self._log_event("=" * 55)

        self._last_activity = time.time()

        try:
            while True:
                # Poll both ports with short timeout
                # Read any available bytes from target → forward to host
                target_data = self.target.read(self.target.in_waiting or 1)
                if target_data:
                    for b in target_data:
                        self.on_target_byte(b)

                # Read any available bytes from host → forward to target
                host_data = self.host.read(self.host.in_waiting or 0)
                if host_data:
                    for b in host_data:
                        self.on_host_byte(b)

                # Check for communication timeout
                self.check_timeout()

                # Small sleep to prevent busy-wait when idle
                if not target_data and not host_data:
                    time.sleep(0.001)

        except KeyboardInterrupt:
            self._log_event("Interrupted")
        except Exception:
            log.exception("Proxy exception")
        finally:
            self.print_stats()


# ---------------------------------------------------------------------------
# CLI Entry Point
# ---------------------------------------------------------------------------

def _open_serial(port, timeout=5.0):
    """Open a serial port with standard monitor settings.

    DTR and RTS are held LOW to prevent resetting the target board.
    Many USB-serial adapters toggle these lines on open, which can
    trip the H8S reset if DTR/RTS is wired to the reset circuit.
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 57600
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = timeout
    ser.write_timeout = 5.0
    ser.dsrdtr = False    # don't let pyserial manage DTR via DSR
    ser.rtscts = False    # don't let pyserial manage RTS via CTS
    ser.dtr = False       # hold DTR low BEFORE open
    ser.rts = False       # hold RTS low BEFORE open
    ser.open()
    return ser


def main():
    parser = argparse.ArgumentParser(
        description="Trace32 ROM Monitor Protocol Simulator / Proxy for H8S/2144",
        epilog="""
Modes:
  Simulator:  --port COM5 [--mot main.mot]
    Emulates the monitor on a virtual COM port.

  Proxy:      --port COM5 --target COM3
    Sits between Trace32 (--port, via com0com) and real hardware (--target).
    Forwards all bytes transparently while tracking protocol state and
    detecting sync loss, checksum errors, and timeouts.
""",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True,
                        help="Host-side serial port (Trace32 connects here, e.g. COM5)")
    parser.add_argument("--target", default=None,
                        help="Target hardware serial port (e.g. COM3). Enables proxy mode.")
    parser.add_argument("--mot", default=None,
                        help="Motorola S-record file to load (simulator mode only)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable verbose protocol logging")
    args = parser.parse_args()

    # Logging setup
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    if args.target:
        # ===== Proxy mode =====
        log.info("Starting in PROXY mode")
        try:
            host_ser = _open_serial(args.port, timeout=0.01)
        except serial.SerialException as e:
            log.error("Cannot open host port %s: %s", args.port, e)
            sys.exit(1)
        log.info("Host port %s opened (Trace32 side)", args.port)

        try:
            target_ser = _open_serial(args.target, timeout=0.01)
        except serial.SerialException as e:
            log.error("Cannot open target port %s: %s", args.target, e)
            host_ser.close()
            sys.exit(1)
        log.info("Target port %s opened (hardware side)", args.target)

        proxy = MonitorProxy(host_ser, target_ser, verbose=args.verbose)
        try:
            proxy.run()
        finally:
            host_ser.close()
            target_ser.close()
            log.info("Both ports closed")

    else:
        # ===== Simulator mode =====
        log.info("Starting in SIMULATOR mode")

        # Determine .mot file
        mot_path = args.mot
        if mot_path is None:
            mot_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "main.mot")
        if not os.path.exists(mot_path):
            log.error("S-record file not found: %s", mot_path)
            sys.exit(1)

        # Load memory
        memory = load_srec(mot_path)

        # Open serial port
        try:
            ser = _open_serial(args.port)
        except serial.SerialException as e:
            log.error("Cannot open serial port %s: %s", args.port, e)
            sys.exit(1)
        log.info("Serial port %s opened", args.port)

        # Run simulator
        sim = MonitorSim(ser, memory, verbose=args.verbose)
        try:
            sim.run()
        finally:
            ser.close()
            log.info("Serial port closed")


if __name__ == "__main__":
    main()
