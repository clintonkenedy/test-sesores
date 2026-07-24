#!/usr/bin/env python3
"""Live RTK dashboard: watch rovers in real time, no page reloads.

Tails the telemetry log that rtcm_to_lora.py (server mode) writes, and serves
a dashboard over HTTP. The page polls for new points every second and appends
them to the map - zoom stays put, nothing flickers, and it works from any
browser on the LAN (the base PC, the Mac, a phone).

Handles several rovers at once: each gets its own track color, the dots are
colored by fix status, and a side list shows every rover's latest state.

Routes:
    /            the dashboard
    /data?since=N  points after sequence N, plus session stats (JSON)
    /tiles/...   local orthophoto pyramid, if --tiles points at one

Run it on the machine that has the log (the base PC), next to the server:
    python rtk_live_server.py
    python rtk_live_server.py --log rover_telemetry.log --port 8765
Then open  http://localhost:8765  (or http://<ip-de-la-pc>:8765 from a phone).
"""

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

# ===========================================================================
#  CONFIGURATION
# ===========================================================================

LOG_FILE = "rover_telemetry.log"
HTTP_PORT = 8765
TILES_DIR = "/Users/cventura/Downloads/servidor/fms/tiles_melvin"
TILES_MIN_ZOOM, TILES_MAX_ZOOM = 15, 21
TAIL_INTERVAL = 0.2      # seconds between log polls

# ===========================================================================

FIX_NAMES = {0: "NO FIX", 1: "SINGLE", 2: "DGPS", 4: "RTK FIXED",
             5: "RTK FLOAT", 6: "DEAD RECKON"}


def dm_to_degrees(value, hemisphere):
    if not value:
        return None
    raw = float(value)
    degrees = int(raw // 100)
    decimal = degrees + (raw - degrees * 100) / 60.0
    return -decimal if hemisphere in ("S", "W") else decimal


class SessionStore:
    """Points parsed from the log, shared between the tail thread and HTTP."""

    def __init__(self):
        self.lock = threading.Lock()
        self.points = []          # dicts with seq/ip/lat/lon/q/sats/age/hsig/t
        self.last_hsig = {}       # per rover ip

    def add_gga(self, stamp, ip, fields):
        lat = dm_to_degrees(fields[2], fields[3])
        lon = dm_to_degrees(fields[4], fields[5])
        if lat is None or lon is None:
            return
        with self.lock:
            self.points.append({
                "seq": len(self.points) + 1,
                "t": stamp, "ip": ip,
                "lat": round(lat, 8), "lon": round(lon, 8),
                "alt": float(fields[9]) if fields[9] else None,
                "q": int(fields[6]) if fields[6] else 0,
                "sats": int(fields[7]) if fields[7] else 0,
                "age": float(fields[13]) if fields[13] else None,
                "hsig": self.last_hsig.get(ip),
            })

    def add_gst(self, ip, fields):
        try:
            self.last_hsig[ip] = round(
                math.hypot(float(fields[6]), float(fields[7])), 3)
        except ValueError:
            pass

    def since(self, seq):
        with self.lock:
            fresh = [p for p in self.points if p["seq"] > seq]
            return fresh, self._stats(), self._rovers()

    def _stats(self):
        """Session statistics; scatter metrics over the FIXED subset."""
        pts = self.points
        if not pts:
            return {}
        fixed = [p for p in pts if p["q"] == 4]
        stats = {"n": len(pts),
                 "pct_fixed": round(100.0 * len(fixed) / len(pts), 1)}
        if len(fixed) >= 10:
            lat0 = sum(p["lat"] for p in fixed) / len(fixed)
            lon0 = sum(p["lon"] for p in fixed) / len(fixed)
            m_lat = 111132.95
            m_lon = 111319.49 * math.cos(math.radians(lat0))
            radial = sorted(math.hypot((p["lon"] - lon0) * m_lon,
                                       (p["lat"] - lat0) * m_lat)
                            for p in fixed)
            k = (len(radial) - 1) * 0.95
            lo = int(k)
            cep95 = radial[lo] + (radial[min(lo + 1, len(radial) - 1)]
                                  - radial[lo]) * (k - lo)
            stats["cep95_cm"] = round(cep95 * 100, 1)
            stats["rms_cm"] = round(math.sqrt(
                sum(r * r for r in radial) / len(radial)) * 100, 1)
        return stats

    def _rovers(self):
        latest = {}
        for p in self.points:
            latest[p["ip"]] = p
        return latest


def tail_log(path, store, stop):
    """Follow the log file, surviving rotation (size shrink -> reopen)."""
    position = 0
    while not stop.is_set():
        try:
            size = os.path.getsize(path)
        except OSError:
            time.sleep(TAIL_INTERVAL)
            continue
        if size < position:      # rotated/truncated
            position = 0
        if size > position:
            with open(path, encoding="utf-8", errors="replace") as handle:
                handle.seek(position)
                chunk = handle.read()
                position = handle.tell()
            for raw in chunk.splitlines():
                parts = raw.split("\t", 2)
                if len(parts) != 3 or not parts[2].startswith("$"):
                    continue
                stamp, ip, text = parts
                fields = text[1:].split("*", 1)[0].split(",")
                kind = fields[0]
                if kind.endswith("GST") and len(fields) >= 9:
                    store.add_gst(ip, fields)
                elif kind.endswith("GGA") and len(fields) >= 15:
                    store.add_gga(stamp, ip, fields)
        time.sleep(TAIL_INTERVAL)


PAGE = r"""<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RTK en vivo</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
:root{color-scheme:light;
  --surface:#fcfcfb; --ink:#0b0b0b; --ink2:#52514e; --muted:#898781;
  --border:rgba(11,11,11,.14);
  --good:#0ca30c; --warn:#fab219; --serious:#ec835a; --dgps:#2a78d6;}
@media (prefers-color-scheme:dark){:root{color-scheme:dark;
  --surface:#1a1a19; --ink:#fff; --ink2:#c3c2b7; --muted:#898781;
  --border:rgba(255,255,255,.14); --dgps:#3987e5;}}
html,body,#map{height:100%;margin:0}
body{font:14px/1.4 system-ui,-apple-system,"Segoe UI",sans-serif}
.panel{position:absolute;top:12px;right:12px;z-index:1000;width:272px;
  background:var(--surface);color:var(--ink);border:1px solid var(--border);
  border-radius:12px;padding:14px;box-shadow:0 4px 16px rgba(0,0,0,.18)}
.hd{font-weight:650;font-size:13px;display:flex;justify-content:space-between;
  align-items:center;margin-bottom:10px}
.live{color:var(--good);font-size:11px}
.tiles{display:grid;grid-template-columns:1fr 1fr;gap:6px 10px}
.tiles div{display:flex;justify-content:space-between;gap:6px}
.k{color:var(--muted);font-size:11px;text-transform:uppercase;
  letter-spacing:.05em;align-self:center}
.v{font-weight:650;font-variant-numeric:tabular-nums}
#fix.f4{color:var(--good)} #fix.f5{color:var(--warn)}
#fix.f1,#fix.f0{color:var(--serious)}
.rovers{margin-top:10px;border-top:1px solid var(--border);padding-top:8px;
  font-size:12px}
.rovers div{display:flex;gap:6px;align-items:center;padding:2px 0;
  color:var(--ink2);font-variant-numeric:tabular-nums}
.sw{width:9px;height:9px;border-radius:50%;flex:none}
.btns{display:flex;gap:8px;margin-top:10px}
button{flex:1;background:var(--surface);color:var(--ink);
  border:1px solid var(--border);border-radius:8px;padding:7px 8px;
  font:inherit;font-size:12.5px;cursor:pointer}
button:hover{border-color:var(--muted)}
button.on{border-color:var(--good);color:var(--good)}
.foot{margin-top:8px;color:var(--muted);font-size:11px}
</style></head><body>
<div id="map"></div>
<div class="panel">
  <div class="hd"><span>RTK EN VIVO</span><span class="live" id="clock">●</span></div>
  <div class="tiles">
    <div><span class="k">Fix</span><span class="v" id="fix">—</span></div>
    <div><span class="k">Sats</span><span class="v" id="sats">—</span></div>
    <div><span class="k">Edad</span><span class="v" id="age">—</span></div>
    <div><span class="k">±GST</span><span class="v" id="hsig">—</span></div>
    <div><span class="k">Puntos</span><span class="v" id="n">—</span></div>
    <div><span class="k">%Fixed</span><span class="v" id="pf">—</span></div>
    <div><span class="k">CEP95</span><span class="v" id="cep">—</span></div>
    <div><span class="k">RMS</span><span class="v" id="rms">—</span></div>
  </div>
  <div class="rovers" id="rovers"></div>
  <div class="btns">
    <button id="follow" class="on">Seguir</button>
    <button id="fit">⟲ Encuadrar</button>
  </div>
  <div class="foot" id="status">conectando…</div>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
var HAS_TILES=__HAS_TILES__, TMIN=__TMIN__, TMAX=__TMAX__;
var FIXCOLOR={4:"#0ca30c",5:"#fab219",2:"#2a78d6",1:"#ec835a",0:"#ec835a",6:"#ec835a"};
var FIXNAME={0:"NO FIX",1:"SINGLE",2:"DGPS",4:"RTK FIXED",5:"RTK FLOAT",6:"DR"};
var ROVERCOLOR=["#2a78d6","#eb6834","#1baf7a","#eda100","#e87ba4"];

var osm=L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png",
  {maxZoom:22,maxNativeZoom:19,attribution:"&copy; OpenStreetMap"});
var sat=L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/"+
  "World_Imagery/MapServer/tile/{z}/{y}/{x}",
  {maxZoom:22,maxNativeZoom:19,attribution:"Esri"});
var map=L.map("map",{layers:[sat],preferCanvas:true}).setView([0,0],3);
var bases={"Satélite":sat,"Calles":osm}, overlays={};
if(HAS_TILES){
  var ortho=L.tileLayer("/tiles/{z}/{x}/{y}.png",
    {minZoom:TMIN,maxZoom:22,maxNativeZoom:TMAX}).addTo(map);
  overlays["Ortofoto"]=ortho;
}
L.control.layers(bases,overlays).addTo(map);

var seq=0, first=true, follow=true;
var tracks={}, dots=L.layerGroup().addTo(map), roverIdx={};

function roverColor(ip){
  if(!(ip in roverIdx)) roverIdx[ip]=Object.keys(roverIdx).length;
  return ROVERCOLOR[roverIdx[ip]%ROVERCOLOR.length];
}
function track(ip){
  if(!tracks[ip]){
    tracks[ip]=L.polyline([],{color:roverColor(ip),weight:2.5,opacity:.9})
      .addTo(map);
  }
  return tracks[ip];
}
function fitAll(){
  var all=[]; Object.values(tracks).forEach(function(t){
    all=all.concat(t.getLatLngs());});
  if(all.length) map.fitBounds(L.latLngBounds(all).pad(0.3));
}
document.getElementById("fit").onclick=fitAll;
document.getElementById("follow").onclick=function(){
  follow=!follow; this.classList.toggle("on",follow);};
map.on("dragstart",function(){follow=false;
  document.getElementById("follow").classList.remove("on");});

function put(id,val){document.getElementById(id).textContent=val;}

function poll(){
  fetch("/data?since="+seq).then(function(r){return r.json();})
  .then(function(d){
    d.points.forEach(function(p){
      seq=p.seq;
      L.circleMarker([p.lat,p.lon],{radius:3,stroke:false,fillOpacity:.75,
        fillColor:FIXCOLOR[p.q]||"#898781"}).addTo(dots);
      track(p.ip).addLatLng([p.lat,p.lon]);
    });
    var last=d.points[d.points.length-1];
    if(last){
      var f=document.getElementById("fix");
      f.textContent=FIXNAME[last.q]||"?";
      f.className="v f"+last.q;
      put("sats",last.sats);
      put("age",last.age==null?"—":last.age+" s");
      put("hsig",last.hsig==null?"—":"±"+last.hsig+" m");
      if(first){map.setView([last.lat,last.lon],19);first=false;}
      else if(follow){map.panTo([last.lat,last.lon]);}
    }
    if(d.stats.n){
      put("n",d.stats.n);
      put("pf",d.stats.pct_fixed+"%");
      put("cep",d.stats.cep95_cm==null?"—":d.stats.cep95_cm+" cm");
      put("rms",d.stats.rms_cm==null?"—":d.stats.rms_cm+" cm");
    }
    var box=document.getElementById("rovers"); box.innerHTML="";
    Object.keys(d.rovers).sort().forEach(function(ip){
      var p=d.rovers[ip], row=document.createElement("div");
      row.innerHTML='<span class="sw" style="background:'+roverColor(ip)+
        '"></span><span>'+ip+'</span><span style="margin-left:auto">'+
        (FIXNAME[p.q]||"?")+' · '+p.sats+' sats</span>';
      box.appendChild(row);
    });
    put("status","último dato "+(last?last.t.slice(11):"—"));
    put("clock","● "+new Date().toLocaleTimeString());
  })
  .catch(function(){put("status","sin conexión con el servidor…");});
}
setInterval(poll,1000); poll();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    store = None
    tiles_dir = None

    def log_message(self, *args):
        pass   # keep the console clean

    def _send(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path == "/":
            page = (PAGE
                    .replace("__HAS_TILES__",
                             "true" if self.tiles_dir else "false")
                    .replace("__TMIN__", str(TILES_MIN_ZOOM))
                    .replace("__TMAX__", str(TILES_MAX_ZOOM)))
            self._send(200, "text/html; charset=utf-8", page.encode("utf-8"))

        elif parsed.path == "/data":
            since = int(parse_qs(parsed.query).get("since", ["0"])[0])
            fresh, stats, rovers = self.store.since(since)
            body = json.dumps({"points": fresh, "stats": stats,
                               "rovers": rovers}).encode("utf-8")
            self._send(200, "application/json", body)

        elif parsed.path.startswith("/tiles/") and self.tiles_dir:
            # /tiles/z/x/y.png -> guard against path escapes, then serve.
            relative = os.path.normpath(parsed.path[len("/tiles/"):])
            full = os.path.join(self.tiles_dir, relative)
            if relative.startswith("..") or not os.path.isfile(full):
                self._send(404, "text/plain", b"not found")
                return
            with open(full, "rb") as handle:
                self._send(200, "image/png", handle.read())

        else:
            self._send(404, "text/plain", b"not found")


def lan_ip():
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        ip = probe.getsockname()[0]
        probe.close()
        return ip
    except OSError:
        return "127.0.0.1"


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", default=LOG_FILE,
                        help="telemetry log to follow (default: %(default)s)")
    parser.add_argument("--port", type=int, default=HTTP_PORT,
                        help="HTTP port (default: %(default)s)")
    parser.add_argument("--tiles", default=TILES_DIR,
                        help="local orthophoto tiles (default: %(default)s)")
    args = parser.parse_args()

    store = SessionStore()
    stop = threading.Event()
    threading.Thread(target=tail_log, args=(args.log, store, stop),
                     daemon=True).start()

    Handler.store = store
    Handler.tiles_dir = args.tiles if os.path.isdir(args.tiles) else None

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print("RTK live dashboard")
    print("  following:  {}".format(args.log))
    print("  orthophoto: {}".format(Handler.tiles_dir or "not found (layer off)"))
    print("  open:       http://localhost:{}".format(args.port))
    print("  on the LAN: http://{}:{}   (Mac, phone, etc.)".format(
        lan_ip(), args.port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop.set()
        server.server_close()


if __name__ == "__main__":
    main()
