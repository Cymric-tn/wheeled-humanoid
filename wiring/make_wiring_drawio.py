#!/usr/bin/env python3
"""Generate wheeled_humanoid_wiring.drawio (RAS Build Challenge, Project 2: Wheeled Humanoid).

Content follows Wiring_Diagram_Wheeled_Humanoid.pdf (sheets 1 to 4). Every wire is a
real draw.io connector between two pin cells, and every component is a container,
so moving a component in draw.io keeps its wires attached.

    python3 wiring/make_wiring_drawio.py
Then open wiring/wheeled_humanoid_wiring.drawio in draw.io (desktop or app.diagrams.net).
"""
import os
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "wheeled_humanoid_wiring.drawio")
REV = "rev. 15/11/2026"

RED, BLACK = "#C62828", "#1A1A1A"
BLUE, SDA, SCL, USB = "#1565C0", "#2E7D32", "#EF6C00", "#5E35B1"
GREY = "#757575"
W14, W16, W26, WSIG = 4, 2.5, 1.2, 1.2

FILL = {
    "power": ("#FDECEA", "#B71C1C"), "conv": ("#FFFFFF", "#333333"),
    "ctrl": ("#F3E5F5", "#6A1B9A"), "load": ("#FFF8E1", "#8D6E63"),
    "module": ("#F5F5F5", "#424242"), "board": ("#E8F5E9", "#2E7D32"),
}


def q(s):
    return escape(str(s), {'"': "&quot;"})


class Page:
    def __init__(self, name, w, h):
        self.name, self.w, self.h = name, w, h
        self.cells, self.n, self.abs = [], 0, {}

    def _id(self, p="c"):
        self.n += 1
        return f"{p}{self.n}"

    # ------------------------------------------------------------ shapes
    def box(self, x, y, w, h, label, kind="module", align="center", valign="top", size=11):
        fill, stroke = FILL[kind]
        i = self._id("b")
        st = (f"rounded=1;arcSize=4;whiteSpace=wrap;html=1;container=1;collapsible=0;recursiveResize=0;"
              f"fillColor={fill};strokeColor={stroke};strokeWidth=1.5;align={align};verticalAlign={valign};"
              f"spacingLeft=6;spacingRight=6;spacingTop=3;fontSize={size};")
        self.cells.append(f'<mxCell id="{i}" value="{q(label)}" style="{st}" vertex="1" parent="1">'
                          f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>')
        self.abs[i] = (x, y, w, h)
        return i

    def pin(self, parent, side, pos, label, w=96, h=16, size=10, color="#000000"):
        """side: L, R (pos = y offset of the pin centre), T, B (pos = x offset of the pin centre)."""
        px, py, pw, ph = self.abs[parent]
        if side == "L":
            rx, ry, al, pt = 0, pos - h / 2, "left", (0, 0.5)
        elif side == "R":
            rx, ry, al, pt = pw - w, pos - h / 2, "right", (1, 0.5)
        elif side == "T":
            rx, ry, al, pt = pos - w / 2, 0, "center", (0.5, 0)
        else:
            rx, ry, al, pt = pos - w / 2, ph - h, "center", (0.5, 1)
        i = self._id("p")
        st = (f"text;html=1;align={al};verticalAlign=middle;fontSize={size};spacingLeft=4;spacingRight=4;"
              f"fontColor={color};points=[[0,0.5],[1,0.5],[0.5,0],[0.5,1]];")
        self.cells.append(f'<mxCell id="{i}" value="{q(label)}" style="{st}" vertex="1" parent="{parent}">'
                          f'<mxGeometry x="{rx}" y="{ry}" width="{w}" height="{h}" as="geometry"/></mxCell>')
        ax, ay = px + rx + pt[0] * w, py + ry + pt[1] * h
        self.abs[i] = (ax, ay, side)
        return i

    def anchor(self, i):
        """Absolute connection point of a pin / dot / tag."""
        a = self.abs[i]
        return a[0], a[1]

    def dot(self, x, y, r=4, color=BLACK):
        i = self._id("d")
        self.cells.append(f'<mxCell id="{i}" value="" style="ellipse;html=1;fillColor={color};strokeColor={color};'
                          f'points=[[0.5,0.5]];" vertex="1" parent="1">'
                          f'<mxGeometry x="{x - r}" y="{y - r}" width="{2 * r}" height="{2 * r}" as="geometry"/></mxCell>')
        self.abs[i] = (x, y, "D")
        return i

    def tag(self, x, y, label, color=RED, side="L", w=40):
        """Small net tag (5V, GND, n.c., ...). side = which end the wire attaches to."""
        i = self._id("t")
        al = "left" if side == "L" else "right"
        self.cells.append(f'<mxCell id="{i}" value="{q(label)}" style="text;html=1;align={al};verticalAlign=middle;'
                          f'fontSize=9;fontStyle=1;fontColor={color};spacingLeft=2;spacingRight=2;points=[[0,0.5],[1,0.5]];" '
                          f'vertex="1" parent="1"><mxGeometry x="{x if side == "L" else x - w}" y="{y - 8}" width="{w}" '
                          f'height="16" as="geometry"/></mxCell>')
        self.abs[i] = (x, y, "TL" if side == "L" else "TR")
        return i

    def text(self, x, y, w, h, label, size=10, color="#000000", align="left", style=""):
        i = self._id("x")
        self.cells.append(f'<mxCell id="{i}" value="{q(label)}" style="text;html=1;whiteSpace=wrap;align={align};'
                          f'verticalAlign=top;fontSize={size};fontColor={color};{style}" vertex="1" parent="1">'
                          f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>')
        return i

    def frame(self, title, subtitle):
        self.text(30, 14, self.w - 60, 30, f"<b>{title}</b>", size=20, color="#AD1457")
        self.text(30, 46, self.w - 60, 36, subtitle, size=11, color="#333333")
        self.text(self.w - 430, self.h - 34, 400, 20,
                  f"Wheeled Humanoid · RAS Build Challenge, Project 2 · {REV}", size=9, color=GREY, align="right")

    # ------------------------------------------------------------ wires
    def wire(self, a, b, color=BLUE, width=WSIG, points=(), label=None, lpos=0.0, dashed=False):
        i = self._id("w")

        def pt(cell, src):
            s = self.abs[cell][2]
            m = {"L": (0, 0.5), "R": (1, 0.5), "T": (0.5, 0), "B": (0.5, 1),
                 "D": (0.5, 0.5), "TL": (0, 0.5), "TR": (1, 0.5)}[s]
            k = "exit" if src else "entry"
            return f"{k}X={m[0]};{k}Y={m[1]};{k}Dx=0;{k}Dy=0;{k}Perimeter=0;"

        st = (f"edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;html=1;endArrow=none;startArrow=none;"
              f"strokeColor={color};strokeWidth={width};fontSize=9;fontStyle=1;fontColor={color};"
              f"labelBackgroundColor=#FFFFFF;jumpStyle=arc;jumpSize=6;" + ("dashed=1;" if dashed else "")
              + pt(a, True) + pt(b, False))
        pts = "".join(f'<mxPoint x="{x}" y="{y}"/>' for x, y in points)
        geo = (f'<mxGeometry relative="1" x="{lpos}" as="geometry">'
               + (f'<Array as="points">{pts}</Array>' if pts else "") + "</mxGeometry>")
        self.cells.append(f'<mxCell id="{i}" value="{q(label or "")}" style="{st}" edge="1" parent="1" '
                          f'source="{a}" target="{b}">{geo}</mxCell>')
        return i

    def line(self, x1, y1, x2, y2, color, width):
        i = self._id("l")
        self.cells.append(f'<mxCell id="{i}" value="" style="endArrow=none;html=1;strokeColor={color};strokeWidth={width};" '
                          f'edge="1" parent="1"><mxGeometry relative="1" as="geometry">'
                          f'<mxPoint x="{x1}" y="{y1}" as="sourcePoint"/><mxPoint x="{x2}" y="{y2}" as="targetPoint"/>'
                          f'</mxGeometry></mxCell>')

    def xml(self, pid):
        body = "".join(self.cells)
        return (f'<diagram name="{q(self.name)}" id="{pid}"><mxGraphModel dx="1400" dy="900" grid="1" gridSize="10" '
                f'guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" '
                f'pageWidth="{self.w}" pageHeight="{self.h}" math="0" shadow="0"><root><mxCell id="0"/>'
                f'<mxCell id="1" parent="0"/>{body}</root></mxGraphModel></diagram>')


# ====================================================================== sheet 1
def sheet_power():
    P = Page("1 Power wiring", 1654, 900)
    P.frame("Sheet 1: power wiring",
            "Every terminal from the battery to each load. Thick lines 14 AWG, medium 16 AWG, thin 26 AWG. "
            "Red = positive, black = negative. Labels W1 to W28 match the power wire schedule.")
    RAIL_P, RAIL_N = 150, 230   # y of the +SW/+ACT rail and of the GND rail

    bat = P.box(30, 110, 130, 150, "<b>LiPo 3S</b><br>11.1 V<br>5200 mAh<br>35C", "power", valign="middle")
    bp = P.pin(bat, "R", 40, "+", w=24, color=RED)
    bn = P.pin(bat, "R", 120, "−", w=24)
    xt = P.box(200, 110, 80, 150, "<b>XT60</b><br>pair", "conv")
    xpi = P.pin(xt, "L", 40, "+", w=22, color=RED)
    xni = P.pin(xt, "L", 120, "−", w=22)
    xpo = P.pin(xt, "R", 40, "+", w=22, color=RED)
    xno = P.pin(xt, "R", 120, "−", w=22)
    P.wire(bp, xpi, RED, W14)
    P.wire(bn, xni, BLACK, W14)

    fuse = P.box(340, RAIL_P - 20, 110, 40, "<b>Fuse 20 A</b><br>5×20 mm glass", "conv", valign="middle", size=10)
    fi, fo = P.pin(fuse, "L", 20, "", w=6), P.pin(fuse, "R", 20, "", w=6)
    sw = P.box(500, RAIL_P - 20, 110, 40, "<b>KCD4 switch</b>", "conv", valign="middle", size=10)
    si, so = P.pin(sw, "L", 20, "", w=6), P.pin(sw, "R", 20, "", w=6)
    P.wire(xpo, fi, RED, W14, label="W1", lpos=0.2)
    P.wire(fo, si, RED, W14, label="W2")

    SPL_SW = P.dot(640, RAIL_P, 5)
    P.wire(so, SPL_SW, RED, W14, label="W3")
    estop = P.box(925, RAIL_P - 22, 120, 44, "<b>E-stop</b> Ø22<br>NC contact", "power", valign="middle", size=10)
    ei, eo = P.pin(estop, "L", 22, "", w=6), P.pin(estop, "R", 22, "", w=6)
    P.wire(SPL_SW, ei, RED, W14, label="W4", lpos=0.4)
    SPL_ACT = P.dot(1070, RAIL_P, 5)
    P.wire(eo, SPL_ACT, RED, W14, label="W5")
    SPL_GND = P.dot(640, RAIL_N, 5)
    P.wire(xno, SPL_GND, BLACK, W14, label="W6", lpos=-0.3)

    P.text(655, RAIL_P - 34, 260, 18, "<b>+SW</b> switched battery, 9.6 to 12.6 V", 10, RED)
    P.text(1085, RAIL_P - 34, 520, 18, "<b>+ACT</b> actuator bus: dead when the emergency stop is pressed", 10, RED)
    P.text(300, RAIL_N + 2, 300, 18, "<b>GND</b> common negative", 10, BLACK)

    # battery divider (W27) → Mega A0
    r1 = P.box(655, 300, 70, 30, "R1 10 kΩ", "conv", valign="middle", size=10)
    r1t, r1b = P.pin(r1, "T", 35, "", w=8, h=4), P.pin(r1, "B", 35, "", w=8, h=4)
    r2 = P.box(655, 380, 70, 30, "R2 3.3 kΩ", "conv", valign="middle", size=10)
    r2t, r2b = P.pin(r2, "T", 35, "", w=8, h=4), P.pin(r2, "B", 35, "", w=8, h=4)
    P.wire(SPL_SW, r1t, RED, W26, points=[(690, RAIL_P)], label="W27", lpos=0.6)
    mid = P.dot(690, 355, 3.5, BLUE)
    P.wire(r1b, mid, RED, W26)
    P.wire(mid, r2t, BLACK, W26)
    a0 = P.tag(575, 355, "to Mega A0<br>(sheet 2)", BLUE, side="R", w=70)
    P.wire(mid, a0, BLUE, W26)
    P.text(530, 372, 125, 30, "3.1 V at 12.6 V", 9, BLUE, align="right")
    g1 = P.tag(690, 445, "GND", BLACK, side="L", w=36)
    P.wire(r2b, g1, BLACK, W26, points=[(690, 445)])

    # converters and drivers (top pins = inputs, bottom pins = outputs)
    def unit(x, name, sub, kind="conv", inp=("IN+", "IN−"), outp=("OUT+", "OUT−"), w=130):
        b = P.box(x, 300, w, 100, f"<b>{name}</b><br>{sub}", kind, valign="middle", size=11)
        ip = P.pin(b, "T", 30, inp[0], w=44, color=RED)
        im = P.pin(b, "T", w - 30, inp[1], w=44)
        op = P.pin(b, "B", 30, outp[0], w=44, color=RED)
        om = P.pin(b, "B", w - 30, outp[1], w=44)
        return ip, im, op, om

    xy = unit(765, "XY-3606", "buck<br>5 V 5 A")
    xa = unit(1090, "XL4016 A", "buck<br>strip A", w=120)
    xb = unit(1230, "XL4016 B", "buck<br>strip B", w=120)
    bl = unit(1370, "BTS7960", "left", "conv", ("B+", "B−"), ("M+", "M−"), w=115)
    br = unit(1505, "BTS7960", "right", "conv", ("B+", "B−"), ("M+", "M−"), w=115)

    def feed(spl, pinid, color, label, rail):
        x, _ = P.anchor(pinid)
        _, py = P.anchor(pinid)
        P.dot(x, rail, 3.5, color)
        P.wire(spl, pinid, color, W16, points=[(x, rail)])
        P.text(x + 3, py - 40, 40, 16, f"<b>{label}</b>", 9, color)

    feed(SPL_SW, xy[0], RED, "W7", RAIL_P)
    feed(SPL_GND, xy[1], BLACK, "W8", RAIL_N)
    feed(SPL_ACT, xa[0], RED, "W11", RAIL_P)
    feed(SPL_GND, xa[1], BLACK, "W12", RAIL_N)
    feed(SPL_ACT, xb[0], RED, "W13", RAIL_P)
    feed(SPL_GND, xb[1], BLACK, "W14", RAIL_N)
    feed(SPL_ACT, bl[0], RED, "W19", RAIL_P)
    feed(SPL_GND, bl[1], BLACK, "W20", RAIL_N)
    feed(SPL_ACT, br[0], RED, "W21", RAIL_P)
    feed(SPL_GND, br[1], BLACK, "W22", RAIL_N)

    # loads
    def load(x, name, sub, kind, pl=("+", "−"), w=130):
        b = P.box(x, 500, w, 90, f"<b>{name}</b><br>{sub}", kind, valign="middle", size=10)
        return P.pin(b, "T", 30, pl[0], w=60, color=RED), P.pin(b, "T", w - 30, pl[1], w=60)

    pi = load(765, "Raspberry Pi 5", "GPIO header", "ctrl", ("pins 2, 4", "pin 6"))
    sa = load(1090, "Servo strip A", "+ 1000 µF<br>ch. 0, 1, 2, 6", "board", ("V+", "GND"), w=120)
    sb = load(1230, "Servo strip B", "+ 1000 µF<br>ch. 3, 4, 5, 7", "board", ("V+", "GND"), w=120)
    ml = load(1370, "Motor L", "M1, M2 of its<br>6-pin plug", "load", ("M1", "M2"), w=115)
    mr = load(1505, "Motor R", "M1, M2 of its<br>6-pin plug", "load", ("M1", "M2"), w=115)
    for (src, dst, labs, v, gauge) in [(xy, pi, ("W9", "W10"), "5.1 V", W16), (xa, sa, ("W15", "W16"), "6.8 V", W16),
                                       (xb, sb, ("W17", "W18"), "6.8 V", W16), (bl, ml, ("W23", "W24"), None, W16),
                                       (br, mr, ("W25", "W26"), None, W16)]:
        P.wire(src[2], dst[0], RED, gauge, label=labs[0] + (f"<br>{v}" if v else ""), lpos=0)
        P.wire(src[3], dst[1], BLACK, gauge, label=labs[1], lpos=0)

    # legend + notes
    lg = P.box(30, 470, 520, 300, "", "module")
    P.text(40, 476, 300, 18, "<b>Legend</b>", 11)
    y0 = 504
    for k, (col, wd, txt) in enumerate([(RED, W14, "positive, 14 AWG"), (BLACK, W14, "negative, 14 AWG"),
                                        (RED, W16, "positive, 16 AWG"), (BLACK, W16, "negative, 16 AWG"),
                                        (RED, W26, "26 AWG (divider)"), (BLUE, W26, "signal to the Mega")]):
        yy = y0 + k * 18
        P.line(45, yy, 90, yy, col, wd)
        P.text(98, yy - 8, 250, 16, txt, 10)
    P.dot(50, y0 + 6 * 18 + 2, 4)
    P.text(62, y0 + 6 * 18 - 7, 300, 18, "soldered splice with heat-shrink", 10)
    P.text(40, y0 + 7 * 18 + 6, 500, 140,
           "<b>Order on the positive lead:</b> battery, XT60, fuse, switch, the two taps before the "
           "emergency stop (divider and Pi supply), the emergency stop, then every actuator.<br>"
           "The +ACT and GND splices sit at the rear of the upper deck, next to the converters.<br>"
           "The converters are not isolated: IN− and OUT− are the same node.<br>"
           "The Mega has no power wire here: it is fed by the Pi over USB (sheet 2).<br>"
           "<b>Before connecting any load:</b> set XL4016 A and B to 6.8 V and the XY-3606 to 5.1 V "
           "with a multimeter, outputs open.", 10)
    return P


# ====================================================================== sheet 2
def sheet_signal():
    P = Page("2 Signal wiring", 1654, 1240)
    P.frame("Sheet 2: signal wiring",
            "Every module pin to its controller pin. Blue: digital signal. Green: I²C SDA. Orange: I²C SCL. "
            "Purple: USB. Red and black tags: 5 V and GND from the Mega (3.3 V and GND from the Pi for the OLEDs).")
    ROW, TOP = 18, 26
    LX, LW = 40, 270
    MX, MW = 610, 220
    SDA_X, SCL_X = 470, 500

    modules = [
        ("Left motor<br>encoder plug", [("VCC", "5V"), ("C1 (phase A)", "D2 (INT)"), ("C2 (phase B)", "D18 (INT)"), ("GND", "GND")]),
        ("Right motor<br>encoder plug", [("VCC", "5V"), ("C1 (phase A)", "D3 (INT)"), ("C2 (phase B)", "D19 (INT)"), ("GND", "GND")]),
        ("HC-SR04 left", [("VCC", "5V"), ("Trig", "D22"), ("Echo", "D23"), ("GND", "GND")]),
        ("HC-SR04 right", [("VCC", "5V"), ("Trig", "D24"), ("Echo", "D25"), ("GND", "GND")]),
        ("VL53L0X no. 1<br>(0x30 after boot)", [("VIN", "5V"), ("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA"), ("XSHUT", "D26")]),
        ("VL53L0X no. 2<br>(0x29)", [("VIN", "5V"), ("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA"), ("XSHUT", "D27")]),
        ("MPU-6050<br>(0x68)", [("VCC", "5V"), ("GND", "GND"), ("SCL", "D21 (SCL)"), ("SDA", "D20 (SDA)"), ("AD0", "GND")]),
        ("PCA9685 (0x40)<br>control header", [("GND", "GND"), ("OE", "D28"), ("SCL", "SCL"), ("SDA", "SDA"), ("VCC", "5V"), ("V+", "nc")]),
        ("Battery divider<br>(sheet 1)", [("mid-point", "A0")]),
    ]
    y = 100
    layout = []
    for name, pins in modules:
        h = max(TOP + len(pins) * ROW + 8, 44)
        b = P.box(LX, y, LW, h, name, "module", align="left", size=10)
        rows = []
        for k, (pn, dest) in enumerate(pins):
            py = TOP + k * ROW + ROW / 2 if len(pins) > 1 else h / 2
            rows.append((P.pin(b, "R", py, pn, w=110), y + py, dest))
        layout.append(rows)
        y += h + 10
    MEGA_TOP, MEGA_BOT = 90, y - 10 + 10
    mega = P.box(MX, MEGA_TOP, MW, MEGA_BOT - MEGA_TOP, "<b>Arduino Mega 2560 R3</b>", "ctrl", size=12)

    scl_rows, sda_rows, mega_pin = [], [], {}
    for rows in layout:
        for pid, ay, dest in rows:
            if dest in ("5V", "GND"):
                P.wire(pid, P.tag(LX + LW + 26, ay, dest, RED if dest == "5V" else BLACK), RED if dest == "5V" else BLACK, WSIG)
            elif dest == "nc":
                P.text(LX + LW + 6, ay - 8, 300, 16, "not connected: servo power is on the strips (sheet 3)", 8, GREY)
            elif dest == "SCL":
                scl_rows.append((pid, ay))
            elif dest == "SDA":
                sda_rows.append((pid, ay))
            else:
                mp = P.pin(mega, "L", ay - MEGA_TOP, dest, w=100)
                mega_pin[dest] = (mp, ay)
                color = SCL if "SCL" in dest else SDA if "SDA" in dest else BLUE
                if "SCL" in dest or "SDA" in dest:
                    bx = SCL_X if "SCL" in dest else SDA_X
                    P.wire(pid, mp, color, WSIG)
                    P.dot(bx, ay, 3, color)
                else:
                    P.wire(pid, mp, color, WSIG)
    for rows, bx, color, key in ((scl_rows, SCL_X, SCL, "D21 (SCL)"), (sda_rows, SDA_X, SDA, "D20 (SDA)")):
        mp, my = mega_pin[key]
        for pid, ay in rows:
            P.dot(bx, ay, 3, color)
            P.wire(pid, mp, color, WSIG, points=[(bx, ay), (bx, my)])

    # ---- right of the Mega: motor drivers, e-stop contact
    RX, RW = 1040, 260
    right_rows = []

    def header(y, name, pins):
        h = TOP + len(pins) * ROW + 8
        b = P.box(RX, y, RW, h, name, "module", align="right", size=10)
        out = []
        for k, (pn, src) in enumerate(pins):
            py = TOP + k * ROW + ROW / 2
            out.append((P.pin(b, "L", py, pn, w=90), y + py, src))
        return out, y + h

    bl, y2 = header(100, "<b>BTS7960 left</b><br>8-pin header",
                    [("1 RPWM", "D5 (PWM)"), ("2 LPWM", "D6 (PWM)"), ("3 R_EN", "D30"), ("4 L_EN", "=EN"),
                     ("5 R_IS", "n.c."), ("6 L_IS", "n.c."), ("7 VCC", "5V"), ("8 GND", "GND")])
    br, y3 = header(y2 + 14, "<b>BTS7960 right</b><br>8-pin header",
                    [("1 RPWM", "D7 (PWM)"), ("2 LPWM", "D8 (PWM)"), ("3 R_EN", "D31"), ("4 L_EN", "=EN"),
                     ("5 R_IS", "n.c."), ("6 L_IS", "n.c."), ("7 VCC", "5V"), ("8 GND", "GND")])
    es, y4 = header(y3 + 14, "<b>E-stop, spare<br>NO contact</b> (optional)", [("13", "D29 (pull-up)"), ("14", "GND")])
    EN_X = RX - 40
    for rows in (bl, br, es):
        en_dot = None
        for pid, ay, src in rows:
            if src in ("5V", "GND"):
                P.wire(P.tag(RX - 20, ay, src, RED if src == "5V" else BLACK, side="R", w=36), pid,
                       RED if src == "5V" else BLACK, WSIG)
            elif src == "n.c.":
                P.text(RX - 40, ay - 8, 36, 16, "n.c.", 8, GREY, align="right")
            elif src == "=EN":
                P.wire(en_dot, pid, BLUE, WSIG, points=[(EN_X, ay)])
            else:
                mp = P.pin(mega, "R", ay - MEGA_TOP, src, w=110)
                if src.startswith("D3"):  # enable line, joined R_EN + L_EN at the module
                    en_dot = P.dot(EN_X, ay, 3, BLUE)
                    P.wire(mp, en_dot, BLUE, WSIG)
                    P.wire(en_dot, pid, BLUE, WSIG)
                else:
                    P.wire(mp, pid, BLUE, WSIG)

    # Mega power + USB
    yy = y4 + 20
    m5 = P.pin(mega, "R", yy - MEGA_TOP, "5V pin", w=100)
    P.wire(m5, P.tag(MX + MW + 20, yy, "5V rail to all 5V tags", RED, w=150), RED, WSIG)
    mg = P.pin(mega, "R", yy + ROW - MEGA_TOP, "GND pins", w=100)
    P.wire(mg, P.tag(MX + MW + 20, yy + ROW, "GND to all GND tags", BLACK, w=150), BLACK, WSIG)
    usb_y = yy + 2.5 * ROW
    musb = P.pin(mega, "R", usb_y - MEGA_TOP, "USB-B", w=100, color=USB)

    # ---- Raspberry Pi 5
    PX, PY, PW = 900, usb_y + 40, 250
    pi_rows = [("pin 1 (3V3)", "oledL", "VCC", RED), ("pin 9 (GND)", "oledL", "GND", BLACK),
               ("pin 5 (SCL1)", "oledL", "SCL", SCL), ("pin 3 (SDA1)", "oledL", "SDA", SDA),
               ("pin 17 (3V3)", "oledR", "VCC", RED), ("pin 25 (GND)", "oledR", "GND", BLACK),
               ("pin 18 (GPIO24)", "oledR", "SCL", SCL), ("pin 16 (GPIO23)", "oledR", "SDA", SDA),
               ("CAM/DISP 0", "cam", "22-pin ribbon", GREY),
               ("pins 2, 4 (5V)", "xy", "OUT+", RED), ("pin 6 (GND)", "xy", "OUT−", BLACK)]
    gaps = {4: 40, 8: 36, 9: 36}
    ys, acc = [], PY + 40
    for k in range(len(pi_rows)):
        acc += gaps.get(k, 0)
        ys.append(acc)
        acc += ROW
    pi = P.box(PX, PY, PW, acc - PY + 10, "<b>Raspberry Pi 5</b> (2 GB)", "ctrl", size=12)
    pusb = P.pin(pi, "L", 40 - 4, "USB-A", w=70, color=USB)
    P.wire(musb, pusb, USB, 2, points=[(PX - 30, usb_y), (PX - 30, PY + 36)])
    P.text(PX - 20, usb_y + 4, 260, 30, "USB A–B cable: serial at 115200 baud<br>and 5 V supply of the Mega", 8, USB)

    OX, OW = 1280, 230
    groups = {}
    for k, (pn, grp, peer, col) in enumerate(pi_rows):
        groups.setdefault(grp, []).append((k, pn, peer, col))
    target = {}
    titles = {"oledL": "<b>OLED left</b><br>1.3\", I²C (0x3C)", "oledR": "<b>OLED right</b><br>1.3\", I²C (0x3C)",
              "cam": "<b>Pi camera</b> 5 MP", "xy": "<b>XY-3606</b><br>5.1 V output (sheet 1)"}
    for grp, items in groups.items():
        y_first, y_last = ys[items[0][0]], ys[items[-1][0]]
        top = y_first - (30 if len(items) > 1 else 14)
        h = max((y_last - top) + ROW / 2 + 8, 30)
        b = P.box(OX, top, OW, h, titles[grp], "module", align="right", size=10)
        for k, pn, peer, col in items:
            pp = P.pin(pi, "R", ys[k] - PY, pn, w=120)
            op = P.pin(b, "L", ys[k] - top, peer, w=100)
            P.wire(pp, op, col, WSIG if col != GREY else 3)
    P.text(PX, acc + 20, 560, 70,
           "Right OLED on a software I²C bus, enabled in <font face=\"monospace\">config.txt</font>:<br>"
           "<font face=\"monospace\">dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24</font><br>"
           "Pi fan on the FAN header. Both OLEDs are 0x3C, so they need separate buses.", 9, "#333333")
    P.text(40, MEGA_BOT + 20, 540, 80,
           "<b>I²C bus of the Mega</b> (D20 SDA, D21 SCL): VL53L0X ×2, MPU-6050, PCA9685. "
           "VL53L0X no. 1 is held off with XSHUT at boot and re-addressed to 0x30; no. 2 keeps 0x29. "
           "Never drive XSHUT high: drive it low or leave it as an input.<br>"
           "The Mega pulls SDA and SCL to 5 V through 10 kΩ; if the bus is unreliable, add a level shifter.", 9, "#333333")
    return P


# ====================================================================== sheet 3
def sheet_servos():
    P = Page("3 Servo harness", 1654, 1180)
    P.frame("Sheet 3: servo harness",
            "PCA9685 signal channels to the servo headers on the perfboard strips. Each header takes its signal from one "
            "PCA9685 channel and V+ / GND from its strip. Servo power never passes through the PCA9685.")
    ORANGE = "#E65100"
    ROW = 30
    servos = {0: ("Left shoulder pitch", "TD-8130MG, 30 kg·cm"), 1: ("Left shoulder roll", "TD-8130MG, 30 kg·cm"),
              2: ("Left elbow", "RCXAZ, 18 kg·cm"), 3: ("Right shoulder pitch", "TD-8130MG, 30 kg·cm"),
              4: ("Right shoulder roll", "TD-8130MG, 30 kg·cm"), 5: ("Right elbow", "RCXAZ, 18 kg·cm"),
              6: ("Head pan", "MG996, 13 kg·cm"), 7: ("Head tilt", "MG996, 13 kg·cm")}
    strips = {"A": [0, 1, 2, 6], "B": [3, 4, 5, 7]}

    # ---- PCA9685
    PXc, PYc, PWc = 170, 170, 250
    ctrl = [("GND", "Mega GND", BLACK), ("OE", "Mega D28", BLUE), ("SCL", "Mega D21", SCL), ("SDA", "Mega D20", SDA),
            ("VCC", "Mega 5V", RED), ("V+", None, GREY)]
    PH = 40 + 8 * ROW + 20 + len(ctrl) * 24 + 40
    pca = P.box(PXc, PYc, PWc, PH, "<b>PCA9685</b> 16-ch PWM (0x40)", "board", size=11)
    ch_pin = {k: P.pin(pca, "R", 45 + k * ROW, f"PWM {k}, signal pin", w=130, color=ORANGE) for k in range(8)}
    for k, (pn, dest, col) in enumerate(ctrl):
        y = 45 + 8 * ROW + 10 + k * 24
        p = P.pin(pca, "L", y, pn, w=50)
        if dest:
            P.wire(P.tag(PXc - 12, PYc + y, dest, col, side="R", w=70), p, col, WSIG)
        else:
            P.text(PXc - 60, PYc + y - 8, 52, 16, "n.c.", 9, GREY, align="right")
    W28_Y = 300 + 80 + 4 * 50 + 30          # just below strip A
    pgnd = P.pin(pca, "R", W28_Y - PYc, "GND (V+ terminal)", w=130)
    P.text(PXc, PYc + PH + 14, PWc, 40, "Control header wired on sheet 2.<br>OE high releases all servos.", 9, GREY)

    # ---- strips with their converters above them
    SX, SW, HROW = 720, 320, 50
    strip, header = {}, {}
    y = 300
    for s, chans in strips.items():
        h = 80 + len(chans) * HROW
        cb = P.box(SX + 20, y - 135, SW - 40, 70,
                   f"<b>XL4016 {s}</b> set to 6.8 V<br>IN+ / IN− from +ACT / GND (sheet 1)", "conv", valign="middle", size=10)
        op = P.pin(cb, "B", 60, "OUT+", w=44, color=RED)
        om = P.pin(cb, "B", SW - 100, "OUT−", w=44)
        b = P.box(SX, y, SW, h, f"<b>Servo strip {s}</b> (perfboard)<br>1000 µF across V+ and GND", "board", size=10)
        vp = P.pin(b, "T", 80, "V+", w=40, color=RED)
        vg = P.pin(b, "T", SW - 80, "GND", w=40)
        gb = P.pin(b, "B", SW - 80, "GND", w=40)
        n1, n2 = ("15", "16") if s == "A" else ("17", "18")
        P.wire(op, vp, RED, W16)
        P.wire(om, vg, BLACK, W16)
        P.text(SX + 84, y - 50, 40, 16, f"<b>W{n1}</b>", 9, RED)
        P.text(SX + SW - 76, y - 50, 40, 16, f"<b>W{n2}</b>", 9, BLACK)
        for k, c in enumerate(chans):
            py = 75 + k * HROW
            header[c] = (P.pin(b, "L", py, f"S{c}", w=40, color=ORANGE),
                         P.pin(b, "R", py, f"header ch {c}:  S · V+ · GND", w=190), y + py)
        strip[s] = (b, gb, y, h)
        y += h + 190

    # ---- PCA channel -> header signal pin; lane order keeps crossings to the unavoidable ch 6 ones
    order = [0, 1, 2, 6, 3, 4, 5, 7]
    for c in range(8):
        sp, _, hy = header[c]
        _, cy = P.anchor(ch_pin[c])
        lane = 650 - order.index(c) * 24
        P.wire(ch_pin[c], sp, ORANGE, 1.5, points=[(lane, cy), (lane, hy)])

    # ---- servos
    VX = 1260
    for c in range(8):
        _, hp, hy = header[c]
        joint, model = servos[c]
        sb = P.box(VX, hy - 21, 300, 42, f"<b>ch {c} · {joint}</b><br>{model}", "load", align="left", valign="middle", size=10)
        P.wire(hp, P.pin(sb, "L", 21, "", w=6), "#8D6E63", 3)
    P.text(1060, 245, 220, 30, "3-wire servo plug:<br>orange = signal, red = V+, brown = GND", 9, "#8D6E63")

    # ---- W28: PCA9685 GND to the GND rail of both strips
    gx, gy = P.anchor(pgnd)
    (_, gA, yA, hA), (_, gB, yB, hB) = strip["A"], strip["B"]
    ax, ay = P.anchor(gA)
    bx, by = P.anchor(gB)
    split = P.dot(gx + 20, gy, 4)
    P.wire(pgnd, split, BLACK, W26)
    P.wire(split, gA, BLACK, W26, points=[(ax, gy)])
    P.wire(split, gB, BLACK, W26, points=[(gx + 20, by + 30), (bx, by + 30)])
    P.text(gx + 26, gy - 18, 60, 16, "<b>W28</b>", 9, BLACK)
    P.text(60, 1080, 1100, 40,
           "<b>W28</b> ties the PCA9685 ground to both strip grounds, so the servo signals share a reference with the servo supply. "
           "Strip A carries the left arm and the head pan, strip B the right arm and the head tilt, so each XL4016 "
           "carries about half of the servo current.", 9, "#333333")
    return P


def main():
    pages = [sheet_power(), sheet_signal(), sheet_servos()]
    xml = ('<mxfile host="generator" agent="make_wiring_drawio.py" version="24.0.0">'
           + "".join(p.xml(f"sheet{k + 1}") for k, p in enumerate(pages)) + "</mxfile>")
    open(OUT, "w").write(xml)
    print("wrote", OUT, "pages:", [p.name for p in pages])


if __name__ == "__main__":
    main()
