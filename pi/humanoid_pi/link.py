"""Serial link to the Arduino Mega (text protocol, 115200 baud; see firmware/protocol.md).

The same class talks to the digital twin: pass  socket://localhost:5555  as the port while
sim/firmware_sim.py --tcp 5555 is running (pyserial opens both kinds of URL).
"""
import collections
import logging
import threading
import time

import serial

log = logging.getLogger("link")


def parse_state(line):
    """'STATE st=IDLE x=12 ...' -> {'st': 'IDLE', 'x': 12, ...}"""
    out = {}
    for item in line.split()[1:]:
        if "=" in item:
            k, v = item.split("=", 1)
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
    return out


class MegaLink:
    def __init__(self, url, stream_hz=5):
        self.url, self.stream_hz = url, stream_hz
        self.state, self.state_time = {}, 0.0
        self.history = collections.deque(maxlen=200)   # (time, line) of everything but STATE
        self._seq = 0                                  # lines emitted so far (history is bounded)
        self.connected = False
        self._ser = None
        self._wlock = threading.Lock()
        self._cond = threading.Condition()
        self._listeners = []
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="mega-link", daemon=True)

    # ---- public API
    def start(self):
        self._thread.start()
        return self

    def close(self):
        self._stop = True

    def add_listener(self, fn):
        """fn(line) is called from the reader thread for every OK / ERR / DONE / EVT line."""
        self._listeners.append(fn)

    def send(self, line):
        line = line.strip()
        with self._wlock:
            if not self._ser:
                raise ConnectionError("Mega not connected")
            self._ser.write((line + "\n").encode())
        log.debug("> %s", line)

    def run(self, cmd, timeout=30.0):
        """Send a command and wait for its DONE (or ERR). Returns (ok, reply line)."""
        name = cmd.split()[0].upper()
        with self._cond:
            mark = self._seq
        self.send(cmd)
        end = time.time() + timeout
        with self._cond:
            while time.time() < end:
                new = self._seq - mark
                for _, ln in list(self.history)[-new:] if new else []:
                    p = ln.split()
                    if len(p) >= 2 and p[0] in ("DONE", "ERR") and p[1] == name:
                        return p[0] == "DONE" and (len(p) < 3 or p[2] == "ok"), ln
                self._cond.wait(0.2)
        return False, "TIMEOUT"

    def state_age(self):
        return time.time() - self.state_time if self.state_time else float("inf")

    # ---- reader thread
    def _open(self):
        ser = serial.serial_for_url(self.url, baudrate=115200, timeout=0.2)
        # Opening the USB port resets the Mega: wait for its READY line (up to 4 s).
        t0 = time.time()
        while time.time() - t0 < 4.0:
            ln = ser.readline().decode(errors="replace").strip()
            if ln.startswith("READY"):
                break
        ser.write(f"STREAM {self.stream_hz}\n".encode())
        return ser

    def _run(self):
        while not self._stop:
            try:
                ser = self._open()
                with self._wlock:
                    self._ser = ser
                self.connected = True
                log.info("connected to %s", self.url)
                self._emit("EVT LINK_UP")
                while not self._stop:
                    raw = ser.readline()
                    if not raw:
                        continue
                    self._handle(raw.decode(errors="replace").strip())
            except (serial.SerialException, OSError) as e:
                if self.connected:
                    log.warning("link lost: %s", e)
                    self._emit("EVT LINK_DOWN")
                self.connected = False
                with self._wlock:
                    self._ser = None
                time.sleep(1.0)

    def _handle(self, line):
        if not line:
            return
        if line.startswith("STATE"):
            self.state = parse_state(line)
            self.state_time = time.time()
            return
        self._emit(line)

    def _emit(self, line):
        with self._cond:
            self.history.append((time.time(), line))
            self._seq += 1
            self._cond.notify_all()
        for fn in self._listeners:
            try:
                fn(line)
            except Exception:  # a listener bug must not kill the link
                log.exception("listener failed on %r", line)
