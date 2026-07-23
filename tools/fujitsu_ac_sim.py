#!/usr/bin/env python3
"""
fujitsu_ac_sim.py - Simulate a Fujitsu (UTY-TFSXW1) indoor unit on a PC.

Play the role of the air conditioner so you can test the esphome-fujitsu dongle
firmware over a plain 3.3 V serial link - no real AC, no 12 V bus.

It answers the two init frames, serves register reads with a realistic AC state,
and applies register writes (so Home Assistant commands round-trip and show up).

WIRING (USB-UART TTL adapter  <->  ESP8266 running the dongle firmware)
  !!! Set the adapter to 3.3 V. A 5 V adapter over-volts the ESP8266 RX pin. !!!
    adapter TX  -> ESP RX (GPIO3)
    adapter RX  -> ESP TX (GPIO1)
    adapter GND -> ESP GND        (common ground is required)
  Power the ESP from its own USB. Do NOT connect the adapter's VCC to the ESP.
  In the ESP YAML use non-inverted UART for this test:
      tx_pin: {number: GPIO1, inverted: false}
      rx_pin: {number: GPIO3, inverted: false}
      baud_rate: 9600   data_bits: 8   parity: NONE   stop_bits: 1

USAGE
    pip install pyserial
    python fujitsu_ac_sim.py --list                # find your COM port
    python fujitsu_ac_sim.py --port COM5           # run the simulator
    python fujitsu_ac_sim.py --selftest            # no hardware; verify logic

Interactive commands while running (type + Enter):
    show                 print the current simulated AC state
    power on|off
    mode auto|cool|dry|fan|heat
    fan  auto|quiet|low|medium|high
    temp <c>             set CURRENT room temperature (e.g. temp 21.5)
    outdoor <c>          set outdoor temperature
    set <AAAA> <VVVV>    set any register by hex address/value
    help
    quit
"""

import argparse
import sys
import threading
import queue
import time

# --- Register map (subset we name; everything else is shown in hex) -----------
R_POWER = 0x1000
R_MODE = 0x1001
R_SETPOINT = 0x1002
R_FAN = 0x1003
R_VSWING = 0x1011
R_VAIRFLOW = 0x10A0
R_HSWING = 0x1023
R_ACTUAL = 0x1033
R_ECONOMY = 0x1100
R_MINHEAT = 0x1101
R_POWERFUL = 0x1120
R_COILDRY = 0x1144
R_OUTDOOR = 0x2020

REG_NAMES = {
    R_POWER: "Power", R_MODE: "Mode", R_SETPOINT: "SetpointTemp", R_FAN: "FanSpeed",
    R_VSWING: "VerticalSwing", R_VAIRFLOW: "VerticalAirflow", R_HSWING: "HorizontalSwing",
    R_ACTUAL: "ActualTemp", R_ECONOMY: "EconomyMode", R_MINHEAT: "MinimumHeat",
    R_POWERFUL: "Powerful", R_COILDRY: "CoilDry", R_OUTDOOR: "OutdoorTemp",
    0x0130: "VerticalAirflowDirCount", 0x0131: "VerticalSwingSupported",
    0x0142: "HorizontalAirflowDirCount", 0x0143: "HorizontalSwingSupported",
    0x0150: "EconomyModeSupported", 0x0151: "MinimumHeatSupported",
    0x0170: "PowerfulSupported", 0x0193: "CoilDrySupported",
}

MODE_NAMES = {0x0000: "auto", 0x0001: "cool", 0x0002: "dry", 0x0003: "fan", 0x0004: "heat"}
FAN_NAMES = {0x0000: "auto", 0x0002: "quiet", 0x0005: "low", 0x0008: "medium", 0x000B: "high"}

# Realistic default AC state: powered on, cooling, setpoint 25.0, room 20.0,
# outdoor 14.75, all features supported. Anything not listed reads back as 0x0000.
DEFAULT_REGS = {
    0x0130: 0x0006, 0x0131: 0x0001, 0x0142: 0x0015, 0x0143: 0x0001,
    0x0150: 0x0001, 0x0151: 0x0001, 0x0152: 0x0001, 0x0153: 0x0001,
    0x0170: 0x0001, 0x0171: 0x0001, 0x0193: 0x0001,
    R_POWER: 0x0001, R_MODE: 0x0001, R_SETPOINT: 0x00FA, R_FAN: 0x0002,
    R_VSWING: 0x0000, R_VAIRFLOW: 0x0001, R_HSWING: 0x0000,
    R_ACTUAL: 0x1B71, R_ECONOMY: 0x0000, R_MINHEAT: 0x0000,
    R_POWERFUL: 0x0000, R_COILDRY: 0x0000, R_OUTDOOR: 0x1964,
}

TEMP_OFFSET = 5025  # actual/outdoor: C = (raw - 5025) / 100


def checksum(data: bytes) -> int:
    cs = 0xFFFF
    for b in data:
        cs = (cs - b) & 0xFFFF
    return cs


def frame_valid(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    got = (frame[-2] << 8) | frame[-1]
    return got == checksum(frame[:-2])


def with_checksum(body: bytearray) -> bytes:
    cs = checksum(bytes(body))
    body.append((cs >> 8) & 0xFF)
    body.append(cs & 0xFF)
    return bytes(body)


def hexs(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def _stamp() -> str:
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1000):03d}"


def name(addr: int) -> str:
    return REG_NAMES.get(addr, f"0x{addr:04X}")


def decode_value(addr: int, val: int) -> str:
    if addr == R_MODE:
        return MODE_NAMES.get(val, f"0x{val:04X}")
    if addr == R_FAN:
        return FAN_NAMES.get(val, f"0x{val:04X}")
    if addr == R_POWER:
        return "on" if val == 1 else "off"
    if addr in (R_VSWING, R_HSWING, R_POWERFUL, R_ECONOMY, R_COILDRY, R_MINHEAT):
        return "on" if val == 1 else ("off" if val == 0 else f"0x{val:04X}")
    if addr == R_SETPOINT:
        return f"{val / 10:.1f}C"
    if addr in (R_ACTUAL, R_OUTDOOR):
        return f"{(val - TEMP_OFFSET) / 100:.2f}C"
    return f"0x{val:04X}"


class FujitsuAcSim:
    def __init__(self, transport, verbose=False):
        self.regs = dict(DEFAULT_REGS)
        self.transport = transport  # object with .write(bytes); None for selftest capture
        self.verbose = verbose
        self.reads_since_init1 = 0
        self.handshake_start = None

    def send(self, frame: bytes, note=""):
        if self.transport is not None:
            self.transport.write(frame)
        print(f"{_stamp()}  [TX] {hexs(frame)}   {note}")

    def handle_frame(self, frame: bytes):
        if not frame_valid(frame):
            print(f"{_stamp()}  [RX] {hexs(frame)}   !! bad checksum, ignored")
            return
        t = frame[0]
        if t == 0x00:
            # Going back to init1 after we'd started reading registers means the
            # dongle reset its whole state machine -> the ESP8266 rebooted.
            if self.reads_since_init1 > 0 and self.handshake_start is not None:
                elapsed = time.time() - self.handshake_start
                print(f"  *** DONGLE RESTARTED after {elapsed:.1f}s "
                      f"({self.reads_since_init1} reads, then back to init1 = it REBOOTED mid-handshake) ***")
            self.reads_since_init1 = 0
            self.handshake_start = time.time()
            print(f"{_stamp()}  [RX] {hexs(frame)}   init1")
            self.send(with_checksum(bytearray([0x00, 0, 0, 0, 0x01, 0x01])), "init1 ack")
        elif t == 0x01:
            print(f"{_stamp()}  [RX] {hexs(frame)}   init2")
            self.send(with_checksum(bytearray([0x01, 0, 0, 0, 0x01, 0x01])), "init2 ack")
            print("  --- handshake complete, dongle should now show 'Running' ---")
        elif t == 0x02:
            self._apply_write(frame)
        elif t == 0x03:
            self.reads_since_init1 += 1
            self._respond_read(frame)
        else:
            print(f"{_stamp()}  [RX] {hexs(frame)}   ?? unknown type 0x{t:02X}")

    def _apply_write(self, frame: bytes):
        count = frame[4] // 4
        changes = []
        for i in range(count):
            idx = 5 + i * 4
            addr = (frame[idx] << 8) | frame[idx + 1]
            val = (frame[idx + 2] << 8) | frame[idx + 3]
            self.regs[addr] = val
            changes.append(f"{name(addr)}={decode_value(addr, val)}")
        print(f"{_stamp()}  [RX] {hexs(frame)}   WRITE: {', '.join(changes)}")
        self.send(with_checksum(bytearray([0x02, 0, 0, 0, 0x01, 0x01])), "write ack")

    def _respond_read(self, frame: bytes):
        count = frame[4] // 2
        addrs = [(frame[5 + i * 2] << 8) | frame[5 + i * 2 + 1] for i in range(count)]
        body = bytearray([0x03, 0, 0, 0, (4 * count + 1) & 0xFF, 0x01])
        for addr in addrs:
            val = self.regs.get(addr, 0x0000)
            body += bytes([(addr >> 8) & 0xFF, addr & 0xFF, (val >> 8) & 0xFF, val & 0xFF])
        print(f"{_stamp()}  [RX] {hexs(frame)}   read {count} regs")
        self.send(with_checksum(body), f"{count} values")

    # -- interactive state helpers --------------------------------------------
    def set_temp_current(self, celsius: float):
        self.regs[R_ACTUAL] = int(round(celsius * 100)) + TEMP_OFFSET

    def set_temp_outdoor(self, celsius: float):
        self.regs[R_OUTDOOR] = int(round(celsius * 100)) + TEMP_OFFSET

    def show(self):
        print("  --- simulated AC state ---")
        for addr in (R_POWER, R_MODE, R_SETPOINT, R_FAN, R_VSWING, R_HSWING,
                     R_POWERFUL, R_ECONOMY, R_ACTUAL, R_OUTDOOR):
            print(f"    {name(addr):16s} = {decode_value(addr, self.regs.get(addr, 0))}")


# --- Serial frame reader (length-based, with inter-frame gap resync) ----------
def serve_serial(port, baud, verbose):
    import serial  # imported here so --selftest works without pyserial

    ser = serial.Serial(port, baudrate=baud, bytesize=8, parity="N",
                        stopbits=1, timeout=0.02)
    print(f"Simulating a Fujitsu AC on {port} @ {baud} 8N1.")
    print("Waiting for the dongle... (Ctrl+C to stop). Type 'help' for commands.\n")

    sim = FujitsuAcSim(ser, verbose=verbose)

    cmd_q = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(cmd_q,), daemon=True).start()

    buf = bytearray()
    last_rx = time.monotonic()
    GAP = 0.10  # >100 ms with no bytes => start of a new frame
    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            now = time.monotonic()
            if data:
                if buf and (now - last_rx) > GAP:
                    buf.clear()  # stale partial frame, resync
                buf += data
                last_rx = now
                _drain_frames(buf, sim)
            _process_commands(cmd_q, sim)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


def _drain_frames(buf: bytearray, sim: FujitsuAcSim):
    while len(buf) >= 7:
        size = buf[4] + 7
        if buf[4] > 121:            # implausible length => garbage, slide by 1
            del buf[:1]
            continue
        if len(buf) < size:
            break                   # wait for the rest of the frame
        frame = bytes(buf[:size])
        if frame_valid(frame):
            del buf[:size]
            sim.handle_frame(frame)
        else:
            del buf[:1]             # misaligned/corrupt, slide by 1 and retry


def _stdin_reader(cmd_q: "queue.Queue[str]"):
    for line in sys.stdin:
        cmd_q.put(line.strip())


def _process_commands(cmd_q, sim: FujitsuAcSim):
    try:
        line = cmd_q.get_nowait()
    except queue.Empty:
        return
    if not line:
        return
    parts = line.split()
    cmd = parts[0].lower()
    try:
        if cmd == "help":
            print(__doc__[__doc__.index("Interactive"):])
        elif cmd == "show":
            sim.show()
        elif cmd == "quit":
            raise KeyboardInterrupt
        elif cmd == "power":
            sim.regs[R_POWER] = 1 if parts[1] == "on" else 0
        elif cmd == "mode":
            inv = {v: k for k, v in MODE_NAMES.items()}
            sim.regs[R_MODE] = inv[parts[1]]
        elif cmd == "fan":
            inv = {v: k for k, v in FAN_NAMES.items()}
            sim.regs[R_FAN] = inv[parts[1]]
        elif cmd == "temp":
            sim.set_temp_current(float(parts[1]))
        elif cmd == "outdoor":
            sim.set_temp_outdoor(float(parts[1]))
        elif cmd == "set":
            sim.regs[int(parts[1], 16)] = int(parts[2], 16)
        else:
            print(f"  ?? unknown command '{cmd}' (type 'help')")
            return
        print(f"  (ok: {line})")
    except (IndexError, KeyError, ValueError):
        print(f"  ?? bad command '{line}' (type 'help')")


# --- Self test (no serial hardware needed) ------------------------------------
class _Capture:
    def __init__(self):
        self.last = None

    def write(self, data):
        self.last = data


def selftest() -> int:
    cap = _Capture()
    sim = FujitsuAcSim(cap)
    fails = 0

    def check(cond, msg):
        nonlocal fails
        if not cond:
            fails += 1
            print(f"  FAIL: {msg}")

    # init1 -> ack
    sim.handle_frame(bytes([0x00, 0, 0, 0, 0x04, 0, 0, 0, 0, 0xFF, 0xFB]))
    check(cap.last == bytes([0x00, 0, 0, 0, 0x01, 0x01, 0xFF, 0xFD]), "init1 ack")

    # init2 -> ack
    sim.handle_frame(bytes([0x01, 0, 0, 0, 0x04, 0, 0x04, 0, 0x01, 0xFF, 0xF5]))
    check(cap.last == bytes([0x01, 0, 0, 0, 0x01, 0x01, 0xFF, 0xFC]), "init2 ack")

    # read Power(0x1000)+Mode(0x1001) -> valid response carrying 0001 / 0001
    req = bytearray([0x03, 0, 0, 0, 0x04, 0x10, 0x00, 0x10, 0x01])
    sim.handle_frame(with_checksum(req))
    resp = cap.last
    check(frame_valid(resp), "read response checksum")
    check(resp[0] == 0x03 and resp[4] == 0x09 and resp[5] == 0x01, "read response header")
    check(resp[6:10] == bytes([0x10, 0x00, 0x00, 0x01]), "Power value == 0x0001")
    check(resp[10:14] == bytes([0x10, 0x01, 0x00, 0x01]), "Mode value == 0x0001")

    # write Power=0 -> ack + state updated
    wr = bytearray([0x02, 0, 0, 0, 0x04, 0x10, 0x00, 0x00, 0x00])
    sim.handle_frame(with_checksum(wr))
    check(cap.last == bytes([0x02, 0, 0, 0, 0x01, 0x01, 0xFF, 0xFB]), "write ack")
    check(sim.regs[R_POWER] == 0x0000, "power turned off")

    # temperature helpers round-trip through the documented encoding
    sim.set_temp_current(22.5)
    check(sim.regs[R_ACTUAL] == 0x1C6B, "22.5C -> 0x1C6B")

    print("selftest: " + ("ALL PASSED" if fails == 0 else f"{fails} FAILURE(S)"))
    return 1 if fails else 0


def list_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit("pyserial not installed. Run: pip install pyserial")
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device:10s} {p.description}")


def main():
    ap = argparse.ArgumentParser(description="Fujitsu AC simulator over serial.")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--selftest", action="store_true", help="run logic self-test, no hardware")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if args.list:
        list_ports()
        return
    if not args.port:
        ap.error("--port is required (or use --list / --selftest)")
    try:
        import serial  # noqa: F401
    except ImportError:
        sys.exit("pyserial not installed. Run: pip install pyserial")
    serve_serial(args.port, args.baud, args.verbose)


if __name__ == "__main__":
    main()
