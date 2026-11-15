"""Behaviour layer: gives the robot its character from the Mega's events.

It only reacts and decorates. Every safety decision (obstacle stop, E-stop, battery) is taken by
the Mega before this code hears about it.
"""
import logging
import threading
import time

log = logging.getLogger("behaviour")


class Behaviour:
    IDLE_SLEEP_S = 90

    def __init__(self, link, eyes):
        self.link, self.eyes = link, eyes
        self.last_activity = time.time()
        self.sleeping = False
        self._last_arg = ""
        link.add_listener(self.on_line)
        threading.Thread(target=self._idle_watch, name="behaviour", daemon=True).start()

    def on_line(self, line):
        p = line.split()
        if not p:
            return
        kind, what = p[0], (p[1] if len(p) > 1 else "")
        if kind in ("OK", "EVT") and what not in ("STREAM",):
            self._wake()
        e = self.eyes
        if kind == "EVT":
            if what == "BLOCKED":
                e.show("surprised", 1.5)
            elif what == "FAULT":
                e.show("fault")
            elif what in ("LINK_DOWN",):
                e.show("fault")
            elif what in ("LINK_UP", "ESTOP_RELEASED", "CLEAR"):
                e.show("neutral")
            elif what == "SLIP":
                e.show("look_down", 1.0)
        elif kind == "OK":
            if what in ("WAVE", "NOD", "DEMO"):
                e.show("happy", 3.0)
            elif what == "SHAKE":
                e.show("look_left", 0.6, then="look_right")
            elif what == "POINT":
                e.show("look_right", 2.0)
            elif what == "TURN":
                try:
                    deg = float(self._last_arg)
                except ValueError:
                    deg = 0
                e.show("look_left" if deg >= 0 else "look_right", 1.5)
            elif what == "RESET":
                e.show("neutral")
        elif kind == "DONE" and what == "SHAKE":
            e.show("neutral")

    def command(self, cmd):
        """Commands from the web page go through here so the eyes can anticipate them."""
        p = cmd.split()
        self._last_arg = p[1] if len(p) > 1 else ""
        self.link.send(cmd)

    def _wake(self):
        self.last_activity = time.time()
        if self.sleeping:
            self.sleeping = False
            self.eyes.show("neutral")

    def _idle_watch(self):
        while True:
            time.sleep(1.0)
            if not self.sleeping and time.time() - self.last_activity > self.IDLE_SLEEP_S:
                self.sleeping = True
                self.eyes.show("sleepy")
