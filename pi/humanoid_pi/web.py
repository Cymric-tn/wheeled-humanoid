"""Web control page: buttons for every function, hold-to-drive pad, live state, eyes and camera.

Only commands matching ALLOWED are forwarded to the Mega; the bench calibration commands (CAL ...)
are deliberately not reachable from the network.
"""
import re
import time

from flask import Flask, Response, jsonify, request

ALLOWED = re.compile(
    r"^(?:(?:FWD|BACK) \d{1,3}|TURN -?\d{1,3}|VEL -?\d{1,3} -?\d{1,3}|(?:WAVE|POINT)(?: [LR])?"
    r"|NOD|SHAKE|REST|STOP|DEMO|RESET|PING|SERVO (?:ON|OFF)|HEAD -?\d{1,2} -?\d{1,2}"
    r"|ARM [LR] -?\d{1,3} -?\d{1,3} -?\d{1,3})$")

PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Wheeled Humanoid</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--ink:#1d2330;--muted:#6b7280;--acc:#2f5bd3;--stop:#d0342c;--ok:#1f8a4c}
@media (prefers-color-scheme:dark){:root{--bg:#14171d;--card:#1e232c;--ink:#e8eaee;--muted:#9aa3b2;--acc:#6d8ff0}}
*{box-sizing:border-box}body{margin:0;font:15px/1.4 system-ui,sans-serif;background:var(--bg);color:var(--ink)}
header{display:flex;align-items:center;gap:12px;padding:12px 16px}h1{font-size:18px;margin:0;flex:1}
.pill{padding:3px 10px;border-radius:99px;font-size:13px;background:#888;color:#fff}.up{background:var(--ok)}.down{background:var(--stop)}
main{display:grid;gap:12px;padding:0 16px 16px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.card{background:var(--card);border-radius:12px;padding:14px}.card h2{font-size:14px;margin:0 0 10px;color:var(--muted);font-weight:600}
button{font:inherit;padding:10px 12px;border-radius:9px;border:0;background:var(--acc);color:#fff;cursor:pointer;touch-action:none}
button:active{filter:brightness(.85)}.row{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:8px}
#stop{background:var(--stop);width:100%;font-size:20px;padding:16px;margin-bottom:10px}
.pad{display:grid;grid-template-columns:repeat(3,64px);gap:6px;justify-content:center}.pad button{height:56px}
img{max-width:100%;border-radius:8px;display:block;background:#000}
table{width:100%;font-size:13px;border-collapse:collapse}td{padding:2px 4px}td:first-child{color:var(--muted)}
#log{font:12px ui-monospace,monospace;height:180px;overflow:auto;white-space:pre-wrap;color:var(--muted)}
</style></head><body>
<header><h1>Wheeled Humanoid</h1><span id="link" class="pill">...</span></header>
<main>
<section class="card"><button id="stop" onclick="cmd('STOP')">STOP</button>
<h2>Drive (hold the arrows)</h2>
<div class="pad"><span></span><button data-v="200 0">&#9650;</button><span></span>
<button data-v="0 60">&#9664;</button><button onclick="cmd('STOP')">&#9632;</button><button data-v="0 -60">&#9654;</button>
<span></span><button data-v="-150 0">&#9660;</button><span></span></div>
<h2 style="margin-top:12px">Moves</h2>
<div class="row"><button onclick="cmd('FWD 50')">Forward 50 cm</button><button onclick="cmd('BACK 50')">Back 50 cm</button>
<button onclick="cmd('TURN 90')">Turn left 90&deg;</button><button onclick="cmd('TURN -90')">Turn right 90&deg;</button></div></section>
<section class="card"><h2>Gestures</h2>
<div class="row"><button onclick="cmd('WAVE R')">Wave right</button><button onclick="cmd('WAVE L')">Wave left</button>
<button onclick="cmd('POINT R')">Point</button><button onclick="cmd('NOD')">Nod</button><button onclick="cmd('SHAKE')">Shake</button>
<button onclick="cmd('REST')">Rest</button></div>
<h2>Robot</h2><div class="row"><button onclick="cmd('DEMO')">Run DEMO</button><button onclick="cmd('SERVO ON')">Servos on</button>
<button onclick="cmd('SERVO OFF')">Servos off</button><button onclick="cmd('RESET')">Reset fault</button></div>
<h2>Eyes</h2><img id="eyes" src="eyes.png" alt="eyes"></section>
<section class="card" id="camcard"><h2>Head camera</h2><img src="stream.mjpg" alt="camera"></section>
<section class="card"><h2>State</h2><table id="state"></table><h2 style="margin-top:10px">Messages</h2><div id="log"></div></section>
</main>
<script>
const TOKEN=new URLSearchParams(location.search).get('token')||'';
async function cmd(c){const r=await fetch('api/cmd',{method:'POST',headers:{'Content-Type':'application/json','X-Token':TOKEN},body:JSON.stringify({cmd:c})});
 if(!r.ok){const j=await r.json().catch(()=>({}));alert(j.error||r.status)}}
let drive=null;
document.querySelectorAll('[data-v]').forEach(b=>{
 const go=()=>{if(drive)return;const send=()=>cmd('VEL '+b.dataset.v);send();drive=setInterval(send,200)};
 const halt=()=>{if(!drive)return;clearInterval(drive);drive=null;cmd('VEL 0 0')};
 b.addEventListener('pointerdown',go);['pointerup','pointerleave','pointercancel'].forEach(e=>b.addEventListener(e,halt))});
const LABELS={st:'state',f:'faults',x:'x (mm)',y:'y (mm)',th:'heading (0.1 deg)',v:'speed (mm/s)',obst:'obstacle (mm)',vbat:'battery (mV)',drv:'drive',gest:'gesture',srv:'servos',demo:'demo step'};
async function poll(){try{const j=await (await fetch('api/state')).json();
 const l=document.getElementById('link');l.textContent=j.connected?'Mega connected':'no link';l.className='pill '+(j.connected?'up':'down');
 document.getElementById('state').innerHTML=Object.entries(LABELS).map(([k,n])=>`<tr><td>${n}</td><td>${j.state[k]??'-'}</td></tr>`).join('');
 const lg=document.getElementById('log');lg.textContent=j.log.join('\\n');lg.scrollTop=1e9;
 document.getElementById('camcard').style.display=j.camera?'':'none';
 document.getElementById('eyes').src='eyes.png?'+Date.now();}catch(e){}}
setInterval(poll,500);poll();
</script></body></html>"""


def create_app(link, behaviour, eyes, camera, token=""):
    app = Flask(__name__)

    def authorised():
        return not token or request.headers.get("X-Token") == token

    @app.get("/")
    def index():
        return PAGE

    @app.get("/api/state")
    def state():
        return jsonify(connected=link.connected, state=link.state, state_age=round(link.state_age(), 2),
                       camera=camera.available,
                       log=[time.strftime("%H:%M:%S ", time.localtime(t)) + ln for t, ln in list(link.history)[-40:]])

    @app.post("/api/cmd")
    def cmd():
        if not authorised():
            return jsonify(error="bad token"), 403
        c = " ".join(str((request.get_json(silent=True) or {}).get("cmd", "")).upper().split())
        if not ALLOWED.match(c):
            return jsonify(error=f"command not allowed: {c!r}"), 400
        try:
            behaviour.command(c)
        except ConnectionError as e:
            return jsonify(error=str(e)), 503
        return jsonify(ok=True)

    @app.get("/eyes.png")
    def eyes_png():
        return Response(eyes.png(), mimetype="image/png", headers={"Cache-Control": "no-store"})

    @app.get("/stream.mjpg")
    def stream():
        if not camera.available:
            return "no camera", 404

        def gen():
            for f in camera.frames():
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n"
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app
