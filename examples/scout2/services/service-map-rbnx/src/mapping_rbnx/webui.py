# SPDX-License-Identifier: MulanPSL-2.0
"""Lightweight map web UI for the mapping service.

A dependency-light (stdlib http.server + Pillow) page that lets an operator
*see* the live SLAM map and pose, and drive the runtime map operations:
  - live 2D occupancy + robot pose preview (auto-refresh)
  - Save  → save_map(map_id)
  - Library → list saved maps with thumbnails; Load → load_map(map_id, mode)
  - Pose Estimate → click the map to seed pose_estimate(x, y, theta)

Runs inside the mapping bridge process, so its buttons call the same
map_ops impls the gRPC/MCP capabilities use — no extra round trip. It reads
the live map/pose off its own dedicated rclpy node (see _ensure_subscriptions).

The Mapping bridge enables this on port 8091 by default; deployment config may
set ``webui_port: 0`` to disable it. The server binds 127.0.0.1 by default
because it exposes unauthenticated map operations; an authenticated deployment
may explicitly set MAPPING_WEBUI_HOST otherwise.
"""
from __future__ import annotations

import collections
import io
import json
import logging
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from . import map_ops

log = logging.getLogger("mapping_rbnx.webui")

MAPS_DIR = os.environ.get("MAPPING_MAPS_DIR", "/mapping/maps")
MAP_TOPIC = os.environ.get("MAPPING_MAP_TOPIC", "/map")
POSE_TOPIC = os.environ.get("MAPPING_POSE_TOPIC", "/robonix/map/pose")

# Latest map / pose, filled by ROS subscriptions on the webui's own node.
_latest = {"grid": None, "pose": None}
_subscribed = False
_sub_lock = threading.Lock()
# Keep the dedicated node + executor alive for the process lifetime.
_webui_node = None
_webui_exec = None

# ── activity log ──────────────────────────────────────────────────────────────
# A small in-memory ring of timestamped action records so the operator can see,
# in the page, what each button did and how a pose_estimate converged. Surfaced
# at GET /api/log; also mirrored to the Python logger.
_LOG = collections.deque(maxlen=200)
_log_lock = threading.Lock()
# Last pose_estimate seed, so convergence can be measured against where the
# robot actually settled after relocalizing.
_seed = {"x": None, "y": None, "theta": None, "t": 0.0}
# Last commanded SLAM mode, surfaced so the UI can show/highlight it. Seeded
# from the config startup map_mode via set_mode_hint(); updated on switch/load.
_mode = os.environ.get("MAPPING_STARTUP_MODE", "mapping")


def set_mode_hint(mode: str) -> None:
    """Record the current SLAM mode for the UI (called by atlas_bridge at init
    with the config's startup map_mode, and by the switch/load handlers)."""
    global _mode
    if mode:
        _mode = mode


def _log_add(kind: str, msg: str) -> None:
    """Append one timestamped entry (kind ∈ save|load|switch|pose|info) to the
    UI activity log and mirror it to the service logger."""
    with _log_lock:
        _LOG.append({"t": time.time(), "kind": kind, "msg": msg})
    log.info("[webui:%s] %s", kind, msg)


def _live_pose_xytheta():
    """Current map-frame pose as (x, y, yaw) from the latest /pose, or None."""
    ps = _latest.get("pose")
    if ps is None:
        return None
    pp = ps.pose.pose
    q = pp.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return pp.position.x, pp.position.y, yaw


def _track_convergence(sx: float, sy: float, stheta: float, settle_s: float = 4.0) -> None:
    """After seeding a pose, wait settle_s for rtabmap to relocalize, then log
    where the robot actually settled and its offset from the clicked estimate
    (distance in metres + heading delta in degrees)."""
    def _check():
        time.sleep(settle_s)
        cur = _live_pose_xytheta()
        if cur is None:
            _log_add("pose", "no live pose — cannot measure convergence")
            return
        cx, cy, cyaw = cur
        dist = math.hypot(cx - sx, cy - sy)
        dth = abs((cyaw - stheta + math.pi) % (2 * math.pi) - math.pi)
        _log_add("pose", f"converged → ({cx:.2f}, {cy:.2f}, {math.degrees(cyaw):.0f}°)  "
                         f"Δ from estimate = {dist:.2f} m / {math.degrees(dth):.0f}°")
    threading.Thread(target=_check, daemon=True).start()


def _ensure_subscriptions() -> None:
    """Subscribe (once) to the live occupancy grid + pose so the UI can render
    them. Best-effort — if ROS isn't up yet the preview stays empty and we
    retry on the next request.

    Uses a DEDICATED node + executor rather than map_ops' shared node. The
    shared node's SingleThreadedExecutor is already spinning by the time the
    UI is first hit, and a subscription created on an already-spinning
    SingleThreadedExecutor is not reliably serviced — the callback never fires
    and the map stays blank even though /map is publishing. Here the
    subscriptions are created BEFORE this node's executor starts spinning, so
    they are always serviced.
    """
    global _subscribed, _webui_node, _webui_exec
    with _sub_lock:
        if _subscribed:
            return
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            from nav_msgs.msg import OccupancyGrid
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

            # The bridge process initialises rclpy lazily (map_ops._get_node on
            # the first map op). The UI may be hit before any map op, so init
            # the context here if needed — mirrors map_ops' own guard. Without
            # this the UI returned before ever subscribing and stayed blank.
            if not rclpy.ok():
                rclpy.init(args=None)

            # Match rtabmap's /map publisher (RELIABLE + TRANSIENT_LOCAL) so the
            # last latched grid is delivered immediately on subscribe.
            latched = QoSProfile(depth=1)
            latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            latched.reliability = QoSReliabilityPolicy.RELIABLE

            def _on_grid(msg):
                _latest["grid"] = msg

            def _on_pose(msg):
                _latest["pose"] = msg

            node = Node("mapping_webui")
            node.create_subscription(OccupancyGrid, MAP_TOPIC, _on_grid, latched)
            node.create_subscription(PoseWithCovarianceStamped, POSE_TOPIC, _on_pose, 10)
            ex = SingleThreadedExecutor()
            ex.add_node(node)
            threading.Thread(target=ex.spin, daemon=True).start()
            _webui_node, _webui_exec = node, ex
            _subscribed = True
            log.info("webui subscribed (dedicated node): map=%s pose=%s", MAP_TOPIC, POSE_TOPIC)
        except Exception as e:  # noqa: BLE001
            log.warning("webui subscriptions failed: %s", e)


def _grid_to_png(grid, pose=None) -> bytes:
    """Render a nav_msgs/OccupancyGrid to a PNG (free=white, occ=black,
    unknown=grey), origin bottom-left, with an optional robot pose marker."""
    from PIL import Image, ImageDraw
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = grid.data
    img = Image.new("RGB", (w, h), (128, 128, 128))
    px = img.load()
    for j in range(h):
        row = j * w
        for i in range(w):
            v = data[row + i]
            if v < 0:
                continue  # unknown → grey
            c = 255 - int(v * 255 / 100)  # 0→white(free), 100→black(occ)
            px[i, h - 1 - j] = (c, c, c)  # flip Y so up = +y
    if pose is not None and res > 0:
        try:
            p = pose.pose.pose
            mx = int((p.position.x - ox) / res)
            my = h - 1 - int((p.position.y - oy) / res)
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            d = ImageDraw.Draw(img)
            r = 4
            d.ellipse([mx - r, my - r, mx + r, my + r], fill=(220, 30, 30))
            d.line([mx, my, mx + int(10 * math.cos(yaw)), my - int(10 * math.sin(yaw))],
                   fill=(220, 30, 30), width=2)
        except Exception:  # noqa: BLE001
            pass
    # Upscale small grids so the page isn't a postage stamp.
    scale = max(1, min(4, 700 // max(1, w)))
    if scale > 1:
        img = img.resize((w * scale, h * scale), Image.NEAREST)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def _list_saved_maps() -> list[dict]:
    out = []
    if not os.path.isdir(MAPS_DIR):
        return out
    for name in sorted(os.listdir(MAPS_DIR)):
        d = os.path.join(MAPS_DIR, name)
        if not os.path.isdir(d):
            continue
        db = os.path.join(d, "rtabmap.db")
        meta = {}
        mp = os.path.join(d, "meta.yaml")
        if os.path.isfile(mp):
            try:
                for line in open(mp):
                    if ":" in line:
                        k, v = line.split(":", 1)
                        meta[k.strip()] = v.strip()
            except Exception:  # noqa: BLE001
                pass
        out.append({
            "map_id": name,
            "has_db": os.path.isfile(db),
            "has_preview": os.path.isfile(os.path.join(d, "occupancy.png")),
            "db_size": os.path.getsize(db) if os.path.isfile(db) else 0,
            "updated": int(os.path.getmtime(db)) if os.path.isfile(db) else 0,
            "meta": meta,
        })
    return out


_PAGE = """<!doctype html><html><head><meta charset=utf-8>
<title>Robonix · mapping</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;background:#0f1115;color:#e6e6e6}
 header{padding:10px 16px;background:#171a21;font-weight:600}
 .wrap{display:flex;gap:16px;padding:16px;flex-wrap:wrap}
 .card{background:#171a21;border:1px solid #262b36;border-radius:8px;padding:12px}
 #mapcv{background:#0a0d12;border-radius:4px;display:block;cursor:grab;touch-action:none;width:720px;height:540px;max-width:100%}
 #mapcv:active{cursor:grabbing}
 button{background:#2d6cdf;color:#fff;border:0;border-radius:6px;padding:7px 12px;cursor:pointer;font-size:14px}
 button.alt{background:#3a4150}
 button.active{background:#1f8a44;box-shadow:0 0 0 2px #2bd66f55}
 button.del{background:#7a2d2d;padding:5px 9px}
 input,select{background:#0f1115;color:#e6e6e6;border:1px solid #2a3140;border-radius:6px;padding:6px}
 .lib{display:flex;flex-direction:column;gap:8px;min-width:240px}
 .mapitem{display:flex;gap:8px;align-items:center;border:1px solid #262b36;border-radius:6px;padding:6px}
 .mapitem img{width:64px;height:64px;object-fit:contain;background:#000;border-radius:4px}
 .muted{color:#8b93a3;font-size:12px}
 #status{padding:6px 16px;color:#8b93a3;font-size:13px}
</style></head><body>
<header>Robonix · mapping live map</header>
<div id=status>connecting…</div>
<div class=wrap>
 <div class=card>
  <canvas id=mapcv width=720 height=540></canvas>
  <div class=muted>drag = pan · wheel = zoom · click = set pose estimate (relocalize)
   · <button class=alt style="padding:2px 8px" onclick="fitView()">Fit</button></div>
 </div>
 <div class=card style="min-width:280px">
  <h3 style="margin:4px 0 10px">Save current map</h3>
  <div style="display:flex;gap:8px">
   <input id=saveid placeholder="map_id e.g. lab_3f" style="flex:1">
   <button onclick="doSave()">Save</button>
  </div>
  <h3 style="margin:16px 0 10px">Mode <span id=modebadge class=muted style="font-weight:400">mode: —</span></h3>
  <div style="display:flex;gap:8px">
   <button id=btn-mapping onclick="doSwitch('mapping')">Mapping (build)</button>
   <button id=btn-localization class=alt onclick="doSwitch('localization')">Localization</button>
  </div>
  <div style="margin-top:10px">
   <button class=del onclick="doReset()">Reset map (clear &amp; rebuild)</button>
   <span class=muted>wipes the live map; origin drifts</span>
  </div>
  <h3 style="margin:16px 0 10px">Library</h3>
  <div id=lib class=lib></div>
 </div>
 <div class=card style="min-width:340px;flex:1">
  <h3 style="margin:4px 0 10px">Activity log</h3>
  <div id=logbox style="height:360px;overflow:auto;font-family:ui-monospace,monospace;font-size:12px;line-height:1.5"></div>
 </div>
</div>
<script>
function setStatus(t){document.getElementById('status').textContent=t}
// ── interactive canvas map: pan (drag) / zoom (wheel) / grid / pose / click-pose
// ── scene-style world-centered canvas (proven model from scene webui) ──
// fit() pins the canvas backing-store resolution to its CSS display size,
// so pointer coords map 1:1 — this is what kept the click coords honest.
const cv=document.getElementById('mapcv'),cx=cv.getContext('2d');
function fit(){if(cv.width!=cv.clientWidth)cv.width=cv.clientWidth;if(cv.height!=cv.clientHeight)cv.height=cv.clientHeight}
window.addEventListener('resize',()=>{fit();draw()});fit();
let MI=null, mapImg=null;
let center=[0,0], pxPerM=40, userMoved=false;   // world center + zoom
function w2p(x,y){return [cv.width/2+(x-center[0])*pxPerM, cv.height/2-(y-center[1])*pxPerM]}
function p2w(sx,sy){return [center[0]+(sx-cv.width/2)/pxPerM, center[1]-(sy-cv.height/2)/pxPerM]}
function reloadMapImg(){let i=new Image();i.onload=()=>{mapImg=i;draw()};i.onerror=()=>{};i.src='/api/map.png?'+Date.now()}
function fitView(){if(!MI)return;userMoved=false;fit();
 let wM=MI.width*MI.resolution,hM=MI.height*MI.resolution;
 center=MI.pose?[MI.pose.x,MI.pose.y]:[MI.origin_x+wM/2,MI.origin_y+hM/2];
 pxPerM=Math.min(cv.width/wM,cv.height/hM)*0.9;draw()}
function draw(){fit();cx.clearRect(0,0,cv.width,cv.height);
 if(!MI){cx.fillStyle='#5a6172';cx.font='13px system-ui';cx.fillText('no map yet',16,24);return}
 if(!userMoved&&MI.pose)center=[MI.pose.x,MI.pose.y];
 // occupancy underlay — map.png is already y-flipped (row0 = world max-y),
 // so place top-left at world (origin_x, origin_y+hMeters) and grow down.
 if(mapImg&&MI.resolution>0){let wM=MI.width*MI.resolution,hM=MI.height*MI.resolution;
  let tl=w2p(MI.origin_x,MI.origin_y+hM);
  cx.imageSmoothingEnabled=false;cx.drawImage(mapImg,tl[0],tl[1],wM*pxPerM,hM*pxPerM)}
 // 1 m grid aligned to world
 cx.strokeStyle='rgba(90,130,200,0.18)';cx.lineWidth=1;
 let step=pxPerM,ox=((cv.width/2)-center[0]*pxPerM)%step,oy=((cv.height/2)+center[1]*pxPerM)%step;
 cx.beginPath();
 for(let x=ox;x<cv.width;x+=step){cx.moveTo(x,0);cx.lineTo(x,cv.height)}
 for(let y=oy;y<cv.height;y+=step){cx.moveTo(0,y);cx.lineTo(cv.width,y)}
 cx.stroke();
 // live pose marker
 if(MI.pose){let p=w2p(MI.pose.x,MI.pose.y),yaw=MI.pose.theta;
  cx.fillStyle='#e63b3b';cx.strokeStyle='#e63b3b';cx.lineWidth=2;
  cx.beginPath();cx.arc(p[0],p[1],5,0,7);cx.fill();
  cx.beginPath();cx.moveTo(p[0],p[1]);cx.lineTo(p[0]+18*Math.cos(yaw),p[1]-18*Math.sin(yaw));cx.stroke()}}
setInterval(reloadMapImg,2000);reloadMapImg()
// interaction — fit() makes internal==display, so (clientX-rect.left) is canvas px
function pt(e){let r=cv.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top]}
let drag=null,moved=0;
cv.addEventListener('mousedown',e=>{drag=pt(e);moved=0});
window.addEventListener('mouseup',()=>{drag=null});
window.addEventListener('mousemove',e=>{if(!drag)return;let p=pt(e);
 center[0]-=(p[0]-drag[0])/pxPerM;center[1]+=(p[1]-drag[1])/pxPerM;
 moved+=Math.abs(p[0]-drag[0])+Math.abs(p[1]-drag[1]);userMoved=true;drag=p;draw()});
cv.addEventListener('wheel',e=>{e.preventDefault();let p=pt(e),wp=p2w(p[0],p[1]);
 pxPerM*=e.deltaY<0?1.15:1/1.15;
 center[0]=wp[0]-(p[0]-cv.width/2)/pxPerM;center[1]=wp[1]+(p[1]-cv.height/2)/pxPerM;userMoved=true;draw()},{passive:false});
cv.addEventListener('dblclick',()=>fitView());
cv.addEventListener('click',async e=>{if(moved>4||!MI)return;
 let p=pt(e),wp=p2w(p[0],p[1]);
 if(!confirm('Seed pose estimate at ('+wp[0].toFixed(2)+', '+wp[1].toFixed(2)+')?'))return;
 setStatus('seeding pose…');
 let r=await (await fetch('/api/pose_estimate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:wp[0],y:wp[1],theta:0})})).json();
 setStatus(r.detail||'seeded')});
async function poll(){try{let s=await (await fetch('/api/state')).json();MI=s;
  if(CURMODE==null&&s.mode)CURMODE=s.mode;applyMode();
  setStatus(s.has_map?('map '+s.width+'×'+s.height+' @'+s.resolution+'m  pose='+(s.pose?('('+s.pose.x.toFixed(2)+', '+s.pose.y.toFixed(2)+', '+s.pose.theta.toFixed(2)+')'):'—')+(s.dist_from_seed!=null?'  Δseed='+s.dist_from_seed+'m':'')):'no map yet');draw()}
  catch(e){setStatus('disconnected')}}
setInterval(poll,1000);poll()
async function loadLib(){let m=await (await fetch('/api/maps')).json();
 let el=document.getElementById('lib');el.innerHTML='';
 if(!m.length){el.innerHTML='<div class=muted>no saved maps yet</div>';return}
 for(const x of m){let d=document.createElement('div');d.className='mapitem';
  d.innerHTML=`<img src="/api/maps/${x.map_id}/preview.png?${Date.now()}">
   <div style="flex:1"><b>${x.map_id}</b><div class=muted>${(x.db_size/1e6).toFixed(1)} MB${x.has_db?'':' · no db'}</div></div>
   <button class=alt onclick="doLoad('${x.map_id}')">Load</button>
   <button class=del onclick="doDelete('${x.map_id}')">Del</button>`;
  el.appendChild(d)}}
setInterval(loadLib,5000);loadLib()
const KCOL={save:'#5bd66f',load:'#5aa9ff',switch:'#d6a85b',pose:'#d65b9a',info:'#8b93a3'};
async function loadLog(){try{let L=await (await fetch('/api/log')).json();
 let box=document.getElementById('logbox');let atBottom=box.scrollTop+box.clientHeight>=box.scrollHeight-20;
 box.innerHTML=L.map(e=>{let t=new Date(e.t*1000).toLocaleTimeString();
  let c=KCOL[e.kind]||'#8b93a3';
  return `<div><span class=muted>${t}</span> <b style="color:${c}">${e.kind}</b> ${e.msg.replace(/</g,'&lt;')}</div>`}).join('');
 if(atBottom)box.scrollTop=box.scrollHeight}catch(e){}}
setInterval(loadLog,1500);loadLog()
async function doSave(){let id=document.getElementById('saveid').value.trim();
 if(!id){alert('enter a map_id');return}setStatus('saving '+id+'…');
 let r=await (await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_id:id})})).json();
 setStatus(r.detail||'saved');loadLib()}
async function doLoad(id){if(!confirm('Load map '+id+' (localization)?'))return;setStatus('loading '+id+'…');
 let r=await (await fetch('/api/load',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_id:id,mode:'localization'})})).json();
 setStatus(r.detail||'loaded')}
async function doSwitch(mode){setStatus('switching to '+mode+'…');
 let r=await (await fetch('/api/switch_mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:mode})})).json();
 if(r.ok)CURMODE=mode;applyMode();setStatus(r.detail||('mode '+mode))}
async function doReset(){if(!confirm('Clear the LIVE map and rebuild from scratch? The new map origin = robot current position, so it will NOT match the old frame (origin drift). Saved maps on disk are not affected.'))return;
 setStatus('resetting map…');
 let r=await (await fetch('/api/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'})).json();
 setStatus(r.detail||'reset')}
async function doDelete(id){if(!confirm('Delete saved map '+id+'? This cannot be undone.'))return;
 setStatus('deleting '+id+'…');
 let r=await (await fetch('/api/delete',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({map_id:id})})).json();
 setStatus(r.detail||'deleted');loadLib()}
let CURMODE=null;
function applyMode(){let mp=document.getElementById('btn-mapping'),lo=document.getElementById('btn-localization');
 let bdg=document.getElementById('modebadge');if(bdg)bdg.textContent=CURMODE?('mode: '+CURMODE):'mode: —';
 if(mp&&lo){mp.classList.toggle('active',CURMODE=='mapping');lo.classList.toggle('active',CURMODE=='localization')}}
</script></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet
        return

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, "application/json", json.dumps(obj).encode())

    def do_GET(self):
        p = urlparse(self.path).path
        try:
            if p == "/" or p == "/index.html":
                return self._send(200, "text/html; charset=utf-8", _PAGE.encode())
            if p == "/api/map.png":
                _ensure_subscriptions()
                g = _latest["grid"]
                if g is None:
                    return self._send(503, "text/plain", b"no map yet")
                # Pure occupancy only — the live pose marker is drawn by the
                # canvas on top, so don't bake a second (scaled) one into the PNG.
                return self._send(200, "image/png", _grid_to_png(g))
            if p == "/api/state":
                _ensure_subscriptions()
                g = _latest["grid"]
                st = {"has_map": g is not None, "mode": _mode}
                if g is not None:
                    st.update(width=g.info.width, height=g.info.height,
                              resolution=round(g.info.resolution, 4),
                              origin_x=g.info.origin.position.x,
                              origin_y=g.info.origin.position.y)
                cur = _live_pose_xytheta()
                if cur is not None:
                    st["pose"] = {"x": cur[0], "y": cur[1], "theta": cur[2]}
                    if _seed["x"] is not None:
                        st["seed"] = {"x": _seed["x"], "y": _seed["y"], "theta": _seed["theta"]}
                        st["dist_from_seed"] = round(math.hypot(cur[0] - _seed["x"],
                                                                cur[1] - _seed["y"]), 3)
                return self._json(st)
            if p == "/api/log":
                with _log_lock:
                    return self._json(list(_LOG))
            if p == "/api/maps":
                return self._json(_list_saved_maps())
            if p.startswith("/api/maps/") and p.endswith("/preview.png"):
                mid = p[len("/api/maps/"):-len("/preview.png")]
                fp = os.path.join(MAPS_DIR, map_ops._sanitize_map_id(mid), "occupancy.png")
                if os.path.isfile(fp):
                    return self._send(200, "image/png", open(fp, "rb").read())
                return self._send(404, "text/plain", b"no preview")
            return self._send(404, "text/plain", b"not found")
        except Exception as e:  # noqa: BLE001
            log.exception("webui GET %s failed", p)
            return self._send(500, "text/plain", str(e).encode())

    def do_POST(self):
        p = urlparse(self.path).path
        try:
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n) or b"{}")
            if p == "/api/save":
                mid = body.get("map_id", "")
                out = map_ops.save_map_impl(mid, body.get("note", ""),
                                            active_db=_active_db_hint())
                _log_add("save", out.get("detail") or (f"saved {mid}" if out.get("ok") else "save failed"))
                return self._json(out)
            if p == "/api/load":
                mid, mode = body.get("map_id", ""), body.get("mode", "localization")
                out = map_ops.load_map_impl(
                    mid, mode, bool(body.get("has_initial_pose", False)),
                    float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                    float(body.get("theta", 0.0)))
                if out.get("ok"):
                    set_mode_hint(mode)
                _log_add("load", f"{'✓' if out.get('ok') else '✗'} load {mid} ({mode}): {out.get('detail','')}")
                return self._json(out)
            if p == "/api/delete":
                mid = body.get("map_id", "")
                out = map_ops.delete_map_impl(mid)
                _log_add("delete", out.get("detail") or (f"deleted {mid}" if out.get("ok") else "delete failed"))
                return self._json(out)
            if p == "/api/reset":
                out = map_ops.reset_map_impl()
                _log_add("reset", out.get("detail") or ("map cleared" if out.get("ok") else "reset failed"))
                return self._json(out)
            if p == "/api/pose_estimate":
                x, y, th = (float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                            float(body.get("theta", 0.0)))
                out = map_ops.pose_estimate_impl(x, y, th)
                if out.get("ok"):
                    _seed.update(x=x, y=y, theta=th, t=time.time())
                    _log_add("pose", f"estimate seeded → ({x:.2f}, {y:.2f}, {math.degrees(th):.0f}°); "
                                     "waiting for relocalization…")
                    _track_convergence(x, y, th)
                else:
                    _log_add("pose", f"✗ pose estimate: {out.get('detail','')}")
                return self._json(out)
            if p == "/api/switch_mode":
                mode = body.get("mode", "")
                out = map_ops.switch_mode_impl(mode)
                if out.get("ok"):
                    set_mode_hint(mode)
                _log_add("switch", f"{'✓' if out.get('ok') else '✗'} switch to {mode}: {out.get('detail','')}")
                return self._json(out)
            return self._send(404, "text/plain", b"not found")
        except Exception as e:  # noqa: BLE001
            log.exception("webui POST %s failed", p)
            return self._json({"ok": False, "detail": str(e)}, code=500)


# Set by atlas_bridge so save_map can checkpoint the live (possibly ephemeral)
# session's db; webui itself has no session state.
_active_db_fn = None


def set_active_db_hint(fn) -> None:
    global _active_db_fn
    _active_db_fn = fn


def _active_db_hint() -> str:
    try:
        return _active_db_fn() if _active_db_fn else ""
    except Exception:  # noqa: BLE001
        return ""


_server = None


def maybe_start() -> None:
    """Start the web UI iff MAPPING_WEBUI_PORT is set. Idempotent; non-fatal."""
    global _server
    port = os.environ.get("MAPPING_WEBUI_PORT", "").strip()
    if not port or _server is not None:
        return
    # This is an unauthenticated admin plane (delete/reset/load/pose seed), so
    # fail safe on loopback. A deployment that has an authenticated overlay
    # network may opt into another host explicitly through `webui_host`.
    host = os.environ.get("MAPPING_WEBUI_HOST", "127.0.0.1").strip() or "127.0.0.1"
    try:
        srv = ThreadingHTTPServer((host, int(port)), _Handler)
    except Exception as e:  # noqa: BLE001
        log.warning("webui: cannot bind %s:%s: %s", host, port, e)
        return
    _server = srv
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log.info("map web UI on http://%s:%s", host, port)
