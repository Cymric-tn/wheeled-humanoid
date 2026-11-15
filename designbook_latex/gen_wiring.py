"""Generates the two TikZ wiring sheets (wiring/power.tex, wiring/signal.tex)."""
P = 0.40
out = []
def w(s): out.append(s)

# =============================================================== SIGNAL SHEET
MX0, MX1 = 7.0, 9.6            # Mega box
LX0, LX1 = 0.0, 3.5            # left modules
RX0, RX1 = 13.5, 17.0          # right modules
BUS_SDA, BUS_SCL = 5.5, 5.95
COL = {"SIG": "sig", "SDA": "sda", "SCL": "scl", "5V": "pwr", "3V3": "pwr", "GND": "black"}
mega_lab = []                  # (y, text, side)
bus_y = {"SDA": [], "SCL": []}

def module(side, ytop, title, pins, x0=None, x1=None, tw=1.75):
    """pins: (label, kind, target). kind: SIG / SDA / SCL / 5V / 3V3 / GND / NC / TXT. Returns y after the module."""
    x0 = x0 if x0 is not None else (LX0 if side == "L" else RX0)
    x1 = x1 if x1 is not None else (LX1 if side == "L" else RX1)
    n = len(pins); ybot = ytop - (n - 1) * P
    w(f"\\draw[mod] ({x0},{ytop+0.24:.2f}) rectangle ({x1},{ybot-0.24:.2f});")
    if side == "L":
        w(f"\\node[anchor=north west,align=left,text width={tw}cm,font=\\scriptsize\\bfseries,inner sep=1.5pt] at ({x0+0.03},{ytop+0.22:.2f}) {{{title}}};")
    else:
        w(f"\\node[anchor=north east,align=right,text width={tw}cm,font=\\scriptsize\\bfseries,inner sep=1.5pt] at ({x1-0.03},{ytop+0.22:.2f}) {{{title}}};")
    ys = {}
    for i, (lab, kind, tgt) in enumerate(pins):
        y = round(ytop - i * P, 3); ys[lab] = y
        xe = x1 if side == "L" else x0; d = 1 if side == "L" else -1
        anc = "east" if side == "L" else "west"
        w(f"\\node[anchor={anc},font=\\scriptsize,inner sep=1.5pt] at ({xe - d*0.03},{y}) {{{lab}}};")
        w(f"\\fill ({xe},{y}) circle (1pt);")
        if kind == "SIG":
            xm = MX0 if side == "L" else MX1
            w(f"\\draw[wire,sig] ({xe},{y}) -- ({xm},{y});")
            mega_lab.append((y, tgt, side))
        elif kind in ("SDA", "SCL"):
            xb = BUS_SDA if kind == "SDA" else BUS_SCL
            w(f"\\draw[wire,{COL[kind]}] ({xe},{y}) -- ({xb},{y}); \\fill[{COL[kind]}] ({xb},{y}) circle (1.3pt);")
            bus_y[kind].append(y)
            if tgt:   # this row carries the Mega pin
                w(f"\\draw[wire,{COL[kind]}] ({xb},{y}) -- ({MX0},{y});"); mega_lab.append((y, tgt, side))
        elif kind in ("5V", "GND", "3V3"):
            w(f"\\draw[wire,{COL[kind]}] ({xe},{y}) -- ({xe + d*0.4},{y});")
            w(f"\\node[tag,{'text=pwr' if kind != 'GND' else ''},anchor={'west' if side=='L' else 'east'}] at ({xe + d*0.4},{y}) {{{tgt or kind}}};")
        elif kind == "NC":
            w(f"\\node[anchor={'west' if side=='L' else 'east'},font=\\tiny,text=gray,inner sep=1.5pt] at ({xe + d*0.05},{y}) {{{tgt or 'not connected'}}};")
        elif kind == "TXT":
            w(f"\\node[anchor={'west' if side=='L' else 'east'},font=\\tiny,inner sep=1.5pt] at ({xe + d*0.05},{y}) {{{tgt}}};")
    return ybot - P * 1.75, ys

w("\\begin{tikzpicture}[font=\\scriptsize, mod/.style={draw=black!75,rounded corners=1.5pt,fill=black!3}, wire/.style={line width=0.7pt}, "
  "tag/.style={font=\\tiny\\bfseries,inner sep=1.2pt}, sig/.style={draw=sig,text=sig}, sda/.style={draw=sda,text=sda,fill=sda}, scl/.style={draw=scl,text=scl,fill=scl}, pwr/.style={draw=pwr}]")
y = 0.0
y, _ = module("L", y, "Left motor\\\\encoder plug", [("VCC", "5V", ""), ("C1 (phase A)", "SIG", "D2 (INT)"), ("C2 (phase B)", "SIG", "D18 (INT)"), ("GND", "GND", "")])
y, _ = module("L", y, "Right motor\\\\encoder plug", [("VCC", "5V", ""), ("C1 (phase A)", "SIG", "D3 (INT)"), ("C2 (phase B)", "SIG", "D19 (INT)"), ("GND", "GND", "")])
y, _ = module("L", y, "HC-SR04 left", [("VCC", "5V", ""), ("Trig", "SIG", "D22"), ("Echo", "SIG", "D23"), ("GND", "GND", "")])
y, _ = module("L", y, "HC-SR04 right", [("VCC", "5V", ""), ("Trig", "SIG", "D24"), ("Echo", "SIG", "D25"), ("GND", "GND", "")])
y, _ = module("L", y, "VL53L0X no. 1 (0x30)", [("VIN", "5V", ""), ("GND", "GND", ""), ("SCL", "SCL", ""), ("SDA", "SDA", ""), ("XSHUT", "SIG", "D26")])
y, _ = module("L", y, "VL53L0X no. 2 (0x29)", [("VIN", "5V", ""), ("GND", "GND", ""), ("SCL", "SCL", ""), ("SDA", "SDA", ""), ("XSHUT", "SIG", "D27")])
y, _ = module("L", y, "MPU-6050 (0x68)", [("VCC", "5V", ""), ("GND", "GND", ""), ("SCL", "SCL", "D21 (SCL)"), ("SDA", "SDA", "D20 (SDA)"), ("AD0", "GND", "")])
y, pca = module("L", y, "PCA9685 (0x40)\\\\control header", [("GND", "GND", ""), ("OE", "SIG", "D28"), ("SCL", "SCL", ""), ("SDA", "SDA", ""), ("VCC", "5V", ""), ("V+", "NC", "not connected: servo power is on the strips")])
y, _ = module("L", y, "Battery divider", [("mid-point", "SIG", "A0")])
yl_end = y
for k in ("SDA", "SCL"):
    xb = BUS_SDA if k == "SDA" else BUS_SCL
    w(f"\\draw[wire,{COL[k]}] ({xb},{max(bus_y[k])}) -- ({xb},{min(bus_y[k])});")
# right side of the Mega
y = 0.0
def bts(y, name, a, b_, en):
    yb, ys = module("R", y, name, [("1 RPWM", "SIG", a), ("2 LPWM", "SIG", b_), ("3 R\\_EN", "SIG", en), ("4 L\\_EN", "TXT", ""), ("5 R\\_IS", "NC", "n.c."), ("6 L\\_IS", "NC", "n.c."), ("7 VCC", "5V", ""), ("8 GND", "GND", "")])
    w(f"\\draw[wire,sig] ({RX0},{ys['4 L\\_EN']}) -- ({RX0-0.45},{ys['4 L\\_EN']}) -- ({RX0-0.45},{ys['3 R\\_EN']}); \\fill[sig] ({RX0-0.45},{ys['3 R\\_EN']}) circle (1.3pt);")
    return yb
y = bts(y, "BTS7960 left, 8-pin header", "D5 (PWM)", "D6 (PWM)", "D30")
y = bts(y, "BTS7960 right, 8-pin header", "D7 (PWM)", "D8 (PWM)", "D31")
y, es = module("R", y, "E-stop, spare NO contact (optional)", [("13", "SIG", "D29 (pull-up)"), ("14", "GND", "")])
yusb = y
# Mega box
ytop, ybot = 0.55, yl_end + P * 1.75 - 0.3
w(f"\\draw[draw=rasred,line width=1pt,rounded corners=2pt,fill=rasred!8] ({MX0},{ytop}) rectangle ({MX1},{ybot:.2f});")
w(f"\\node[font=\\small\\bfseries,align=center] at ({(MX0+MX1)/2},{ytop+0.3}) {{Arduino Mega 2560}};")
for (yy, t, side) in mega_lab:
    if side == "L": w(f"\\node[anchor=west,font=\\scriptsize,inner sep=1.5pt] at ({MX0+0.03},{yy}) {{{t}}}; \\fill ({MX0},{yy}) circle (1pt);")
    else: w(f"\\node[anchor=east,font=\\scriptsize,inner sep=1.5pt] at ({MX1-0.03},{yy}) {{{t}}}; \\fill ({MX1},{yy}) circle (1pt);")
# Mega 5V / GND / USB pins on the right, lower part
y5 = yusb - 0.2
for i, (lab, tag, c) in enumerate((("5V pin", "5V rail to all 5V tags", "pwr"), ("GND pins", "GND to all GND tags", "black"))):
    yy = round(y5 - i * P, 3)
    w(f"\\node[anchor=east,font=\\scriptsize,inner sep=1.5pt] at ({MX1-0.03},{yy}) {{{lab}}}; \\fill ({MX1},{yy}) circle (1pt);")
    w(f"\\draw[wire,{'pwr' if c=='pwr' else 'black'}] ({MX1},{yy}) -- ({MX1+0.4},{yy}); \\node[tag,anchor=west,{'text=pwr' if c=='pwr' else ''}] at ({MX1+0.4},{yy}) {{{tag}}};")
yU = round(y5 - 2 * P - 0.3, 3)
w(f"\\node[anchor=east,font=\\scriptsize,inner sep=1.5pt] at ({MX1-0.03},{yU}) {{USB-B}}; \\fill ({MX1},{yU}) circle (1pt);")

# Raspberry Pi block (right, below)
PX0, PX1 = 11.5, 13.0
ypi = yU - 1.6
rows = []
def pi_mod(y, title, pins):
    yb, ys = module("R", y, title, [(l, "TXT", "") for l, _ in pins])
    for (l, pin) in pins:
        yy = ys[l]
        col = "pwr" if "3V3" in pin else ("black" if "GND" in pin else ("sda" if "SDA" in pin else "scl"))
        w(f"\\draw[wire,{col}] ({RX0},{yy}) -- ({PX1},{yy});")
        rows.append((yy, pin))
    return yb
y = pi_mod(ypi, "OLED left (0x3C)", [("VCC", "pin 1 (3V3)"), ("GND", "pin 9 (GND)"), ("SCL", "pin 5 (SCL1)"), ("SDA", "pin 3 (SDA1)")])
y = pi_mod(y, "OLED right (0x3C)", [("VCC", "pin 17 (3V3)"), ("GND", "pin 25 (GND)"), ("SCL", "pin 18 (GPIO24)"), ("SDA", "pin 16 (GPIO23)")])
yb, ys = module("R", y, "Pi camera", [("22-pin ribbon", "TXT", "")])
w(f"\\draw[wire,sig] ({RX0},{ys['22-pin ribbon']}) -- ({PX1},{ys['22-pin ribbon']});"); rows.append((ys['22-pin ribbon'], "CAM/DISP 0"))
y = yb
yb, ys = module("R", y, "XY-3606, 5.1 V output", [("OUT+", "TXT", ""), ("OUT$-$", "TXT", "")])
w(f"\\draw[wire,pwr] ({RX0},{ys['OUT+']}) -- ({PX1},{ys['OUT+']});"); rows.append((ys['OUT+'], "pins 2, 4 (5V)"))
w(f"\\draw[wire,black] ({RX0},{ys['OUT$-$']}) -- ({PX1},{ys['OUT$-$']});"); rows.append((ys['OUT$-$'], "pin 6 (GND)"))
ypb = yb + P * 1.75 - 0.3
w(f"\\draw[draw=raspurple,line width=1pt,rounded corners=2pt,fill=soft] ({PX0-1.25},{ypi+0.95}) rectangle ({PX1},{ypb:.2f});")
w(f"\\node[font=\\small\\bfseries,align=center] at ({(PX0-1.25+PX1)/2},{ypi+0.65}) {{Raspberry Pi 5}};")
for yy, t in rows:
    w(f"\\node[anchor=east,font=\\scriptsize,inner sep=1.5pt] at ({PX1-0.03},{yy}) {{{t}}}; \\fill ({PX1},{yy}) circle (1pt);")
# USB link Mega <-> Pi
yusbpi = round(ypi + 0.2, 3)
w(f"\\node[anchor=west,font=\\scriptsize,inner sep=1.5pt] at ({PX0-1.22},{yusbpi}) {{USB-A}}; \\fill ({PX0-1.25},{yusbpi}) circle (1pt);")
w(f"\\draw[wire,sig,line width=1.2pt] ({MX1},{yU}) -- ({MX1+0.5},{yU}) |- ({PX0-1.25},{yusbpi});")
w(f"\\node[font=\\tiny,text=sig,anchor=west,align=left] at ({MX1+0.55},{(yU+yusbpi)/2+0.25:.2f}) {{USB A--B cable: serial at 115200 baud\\\\and 5 V supply of the Mega}};")
# soft bus note
w(f"\\node[font=\\tiny,anchor=north west,align=left,text width=6.6cm] at ({PX0-1.25},{ypb-0.12:.2f}) {{Right OLED on a software I\\textsuperscript{{2}}C bus, enabled in \\texttt{{config.txt}}:\\\\ \\texttt{{dtoverlay=i2c-gpio,bus=3,}}\\\\ \\texttt{{i2c\\_gpio\\_sda=23,i2c\\_gpio\\_scl=24}}\\\\Pi fan on the FAN header.}};")
w("\\end{tikzpicture}")
open("wiring/signal.tex", "w").write("\n".join(out)); out.clear()

# =============================================================== POWER SHEET
T14, T16 = "line width=1.7pt", "line width=0.9pt"
w("\\begin{tikzpicture}[font=\\scriptsize, cmp/.style={draw=black!75,rounded corners=1.5pt,fill=white,align=center,inner sep=2pt}, "
  "p/.style={draw=pwr}, n/.style={draw=black}, t/.style={font=\\fontsize{6}{7}\\selectfont,inner sep=1pt}]")
w(f"\\draw[p,{T14}] (1.9,0) -- (17.25,0);")
w(f"\\draw[n,{T14}] (1.9,-1.0) -- (17.25,-1.0);")
w("\\node[cmp,fill=pwr!12,minimum width=1.9cm,minimum height=1.9cm] at (0.95,-0.5) {\\textbf{LiPo 3S}\\\\11.1\\,V\\\\5200\\,mAh\\\\35C};")
w("\\node[t,anchor=south west,text=pwr] at (1.93,0.03) {+}; \\node[t,anchor=north west] at (1.93,-1.03) {$-$};")
w("\\node[cmp,minimum width=0.85cm,minimum height=1.85cm] at (2.95,-0.5) {XT60\\\\pair};")
w("\\node[cmp,minimum width=1.3cm,minimum height=0.66cm] at (4.7,0) {Fuse 20\\,A};")
w("\\node[cmp,minimum width=1.4cm,minimum height=0.66cm] at (6.55,0) {KCD4 switch};")
w("\\node[cmp,fill=pwr!12,minimum width=1.5cm,minimum height=0.66cm] at (10.15,0) {\\textbf{E-stop} NC};")
w("\\node[t,text=pwr,anchor=south] at (8.3,0.42) {\\textbf{+SW} switched battery}; \\node[t,text=pwr,anchor=south] at (14.1,0.42) {\\textbf{+ACT} actuator bus, dead when the emergency stop is pressed};")
w("\\node[t,anchor=north west] at (3.5,-1.08) {\\textbf{GND} common negative};")
w(f"\\fill[pwr] (7.7,0) circle (1.6pt); \\draw[p,line width=0.5pt] (7.7,0) -- (7.7,-1.75);")
w("\\node[cmp,minimum width=0.8cm,minimum height=0.5cm] (r1) at (7.7,-2.0) {10\\,k$\\Omega$};")
w("\\node[cmp,minimum width=0.8cm,minimum height=0.5cm] (r2) at (7.7,-3.3) {3.3\\,k$\\Omega$};")
w("\\draw[line width=0.5pt] (r1) -- (r2); \\fill[sig] (7.7,-2.65) circle (1.4pt); \\draw[draw=sig,line width=0.6pt] (7.7,-2.65) -- (6.9,-2.65); \\node[t,text=sig,anchor=east,align=right] at (6.9,-2.65) {to Mega A0\\\\3.1\\,V at 12.6\\,V};")
w("\\draw[n,line width=0.5pt] (r2) -- (7.7,-3.9); \\node[t,anchor=north] at (7.7,-3.9) {\\textbf{GND}};")
def load(xc, wd, title, inlab=("IN+", "IN$-$"), outlab=("OUT+", "OUT$-$"), fill="white"):
    xp, xn = round(xc - wd * 0.24, 2), round(xc + wd * 0.24, 2)
    w(f"\\fill[pwr] ({xp},0) circle (1.6pt); \\draw[p,{T16}] ({xp},0) -- ({xp},-2.1);")
    w(f"\\fill ({xn},-1.0) circle (1.6pt); \\draw[n,{T16}] ({xn},-1.0) -- ({xn},-2.1);")
    w(f"\\node[cmp,fill={fill},minimum width={wd}cm,minimum height=1.7cm] at ({xc},-2.95) {{{title}}};")
    w(f"\\node[t,anchor=north,text=pwr] at ({xp},-2.13) {{{inlab[0]}}}; \\node[t,anchor=north] at ({xn},-2.13) {{{inlab[1]}}};")
    w(f"\\node[t,anchor=south,text=pwr] at ({xp},-3.77) {{{outlab[0]}}}; \\node[t,anchor=south] at ({xn},-3.77) {{{outlab[1]}}};")
    return xp, xn
def sink(xc, wd, ht, text, xp, xn, lab, fill="white", thick=T16):
    w(f"\\draw[p,{thick}] ({xp},-3.8) -- ({xp},-5.5); \\draw[n,{thick}] ({xn},-3.8) -- ({xn},-5.5);")
    w(f"\\node[cmp,fill={fill},minimum width={wd}cm,minimum height={ht}cm,anchor=north,text width={wd-0.15}cm] at ({xc},-5.5) {{{text}}};")
    if lab: w(f"\\node[t,anchor=east,text=pwr,align=right] at ({xp-0.06},-4.65) {{{lab}}};")
xp, xn = load(8.75, 1.45, "\\textbf{XY-3606}\\\\buck")
sink(8.75, 1.75, 1.7, "\\textbf{Raspberry Pi 5}\\\\pins 2, 4: 5\\,V\\\\pin 6: GND", xp, xn, "5.1\\,V", fill="soft")
xp, xn = load(11.85, 1.55, "\\textbf{XL4016 A}\\\\buck")
sink(11.85, 1.8, 1.7, "\\textbf{Servo strip A}\\\\+ 1000\\,\\textmu F\\\\ch. 0, 1, 2, 6", xp, xn, "6.8\\,V")
xp, xn = load(13.7, 1.55, "\\textbf{XL4016 B}\\\\buck")
sink(13.7, 1.8, 1.7, "\\textbf{Servo strip B}\\\\+ 1000\\,\\textmu F\\\\ch. 3, 4, 5, 7", xp, xn, "6.8\\,V")
xp, xn = load(15.3, 1.25, "\\textbf{BTS7960}\\\\left", ("B+", "B$-$"), ("M+", "M$-$"))
sink(15.3, 1.3, 1.7, "\\textbf{Motor L}\\\\M1, M2 of its 6-pin plug", xp, xn, "", fill="act!15")
xp, xn = load(16.65, 1.25, "\\textbf{BTS7960}\\\\right", ("B+", "B$-$"), ("M+", "M$-$"))
sink(16.65, 1.3, 1.7, "\\textbf{Motor R}\\\\M1, M2 of its 6-pin plug", xp, xn, "", fill="act!15")
w("\\node[anchor=north west,align=left,font=\\scriptsize,text width=6.3cm,draw=black!40,rounded corners=2pt,inner sep=4pt] at (0,-4.5) {"
  "\\textbf{Legend}\\\\[1pt]\\tikz{\\draw[draw=pwr,line width=1.7pt] (0,0)--(0.6,0);} positive, 14 AWG \\quad \\tikz{\\draw[draw=pwr,line width=0.9pt] (0,0)--(0.6,0);} positive, 16 AWG\\\\"
  "\\tikz{\\draw[line width=1.7pt] (0,0)--(0.6,0);} negative, 14 AWG \\quad \\tikz{\\draw[line width=0.9pt] (0,0)--(0.6,0);} negative, 16 AWG\\\\"
  "\\tikz{\\fill (0,0) circle (1.6pt);} soldered splice with heat-shrink\\\\[3pt]"
  "\\textbf{Order on the positive lead:} battery, XT60, fuse, switch, the two taps before the emergency stop (divider and Pi supply), the emergency stop, then every actuator.\\\\[3pt]"
  "The +ACT and GND splices sit at the rear of the upper deck, next to the converters. The converters are not isolated: IN$-$ and OUT$-$ are the same node. The Mega has no power wire here; it is fed by the Pi over USB.};")
w("\\end{tikzpicture}")
open("wiring/power.tex", "w").write("\n".join(out))
print("ok")
