#!/usr/bin/env python3
"""Log rover RTK positions and build a self-contained HTML precision report.

Collects GGA/GST from the rover receiver (through the transparent usb_bridge
on the ESP32), optionally injecting live corrections from the base server so
the rover holds RTK while it is being measured. On Ctrl+C (or --minutes) it
computes the precision statistics and writes:

  rtk_report.html   single shareable file: map, point cloud, metrics, analysis
  rtk_points.csv    every sample, for reprocessing

The empirical numbers (scatter of positions around their mean) are the ground
truth of precision; the receiver's own GST estimate is reported alongside.

Examples:
    python rtk_map.py --list
    python rtk_map.py --port /dev/cu.usbserial-0002 --base-host 192.168.1.162
    python rtk_map.py --port /dev/cu.usbserial-0002 --base-host 192.168.1.162 --minutes 10
"""

import argparse
import csv
import json
import math
import socket
import sys
import threading
import time
from datetime import datetime

import serial
from serial.tools import list_ports

# ===========================================================================
#  CONFIGURATION - edit these, or override on the command line.
# ===========================================================================

SERIAL_PORT = "COM5" if sys.platform.startswith("win") else "/dev/cu.usbserial-0002"
SERIAL_BAUD = 115200

BASE_HOST = "192.168.1.162"
BASE_PORT = 8887

REPORT_FILE = "rtk_report.html"
CSV_FILE = "rtk_points.csv"

# Local orthophoto pyramid (gdal2tiles XYZ). Loads only on this machine; when
# the report is shared, the satellite/street layers still work for everyone.
TILES_DIR = "/Users/cventura/Downloads/servidor/fms/tiles_melvin"
TILES_MIN_ZOOM, TILES_MAX_ZOOM = 15, 21

RECONNECT_DELAY = 3.0
MAX_PLOT_POINTS = 3000          # metrics use every sample; plots subsample

# ===========================================================================

FIX_NAMES = {0: "NO FIX", 1: "SINGLE", 2: "DGPS", 4: "RTK FIXED",
             5: "RTK FLOAT", 6: "DEAD RECKON"}

# Status colors from the validated reference palette (light / dark).
FIX_COLOR = {
    4: ("#0ca30c", "#0ca30c"),   # FIXED  -> good
    5: ("#fab219", "#fab219"),   # FLOAT  -> warning
    2: ("#2a78d6", "#3987e5"),   # DGPS   -> series blue
    1: ("#ec835a", "#ec835a"),   # SINGLE -> serious
}
FIX_FALLBACK = ("#898781", "#898781")


def show_ports():
    ports = sorted(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for port in ports:
        print("  {:<22} {}".format(port.device, port.description))


def dm_to_degrees(value, hemisphere):
    if not value:
        return None
    raw = float(value)
    degrees = int(raw // 100)
    decimal = degrees + (raw - degrees * 100) / 60.0
    return -decimal if hemisphere in ("S", "W") else decimal


class Sample:
    __slots__ = ("t", "lat", "lon", "alt", "quality", "sats", "age", "hsig")

    def __init__(self, t, lat, lon, alt, quality, sats, age, hsig):
        self.t, self.lat, self.lon, self.alt = t, lat, lon, alt
        self.quality, self.sats, self.age, self.hsig = quality, sats, age, hsig


def inject_from_tcp(stop, rover, host, port):
    """Pull corrections from the base and write them into the rover receiver."""
    while not stop.is_set():
        try:
            with socket.create_connection((host, port), timeout=5) as link:
                print("  corrections: connected to {}:{}".format(host, port))
                link.settimeout(1.0)
                while not stop.is_set():
                    try:
                        chunk = link.recv(1024)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    rover.write(chunk)
        except OSError as error:
            print("  corrections: {}:{} - {}".format(host, port, error))
        if not stop.is_set():
            time.sleep(RECONNECT_DELAY)


def collect(args):
    """Read NMEA until Ctrl+C or the time limit; return the sample list."""
    samples = []
    stop = threading.Event()
    rover = serial.Serial(args.port, args.baud, timeout=1)

    if args.base_host:
        threading.Thread(target=inject_from_tcp,
                         args=(stop, rover, args.base_host, args.base_port),
                         daemon=True).start()
    else:
        print("NOTE: no --base-host; logging whatever fix the rover already has.")

    deadline = time.monotonic() + args.minutes * 60 if args.minutes else None
    last_hsig = None
    line = bytearray()
    last_echo = 0.0

    print("Collecting... Ctrl+C to stop and build the report.")
    try:
        while deadline is None or time.monotonic() < deadline:
            data = rover.read(256)
            for byte in data:
                if byte != 0x0A:
                    if byte != 0x0D and len(line) < 200:
                        line.append(byte)
                    continue
                text = line.decode("ascii", "replace").strip()
                line.clear()
                if not text.startswith("$"):
                    continue
                fields = text[1:].split("*", 1)[0].split(",")
                kind = fields[0]

                if kind.endswith("GST") and len(fields) >= 9:
                    try:
                        last_hsig = math.hypot(float(fields[6]), float(fields[7]))
                    except ValueError:
                        pass
                elif kind.endswith("GGA") and len(fields) >= 15:
                    lat = dm_to_degrees(fields[2], fields[3])
                    lon = dm_to_degrees(fields[4], fields[5])
                    if lat is None or lon is None:
                        continue
                    sample = Sample(
                        time.time(), lat, lon,
                        float(fields[9]) if fields[9] else None,
                        int(fields[6]) if fields[6] else 0,
                        int(fields[7]) if fields[7] else 0,
                        float(fields[13]) if fields[13] else None,
                        last_hsig)
                    samples.append(sample)
                    now = time.monotonic()
                    if now - last_echo >= 5:
                        last_echo = now
                        print("  {:>5} pts  {}  sats={}  age={}".format(
                            len(samples), FIX_NAMES.get(sample.quality, "?"),
                            sample.sats, sample.age))
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        stop.set()
        rover.close()
    return samples


def parse_telemetry_log(path, rover_ip=None):
    """Build samples from the server's rover_telemetry.log (standalone rover).

    Each line is '<iso-time>\t<rover-ip>\t<sentence>'. Only raw $..GGA/$..GST
    lines carry positions; HELLO/FIX summaries are skipped.
    """
    samples = []
    last_hsig = {}
    seen_ips = set()
    with open(path, encoding="utf-8", errors="replace") as handle:
        for raw in handle:
            parts = raw.rstrip("\n").split("\t", 2)
            if len(parts) != 3:
                continue
            stamp, ip, text = parts
            if not text.startswith("$"):
                continue
            seen_ips.add(ip)
            if rover_ip and ip != rover_ip:
                continue
            try:
                t = datetime.fromisoformat(stamp).timestamp()
            except ValueError:
                continue
            fields = text[1:].split("*", 1)[0].split(",")
            kind = fields[0]
            if kind.endswith("GST") and len(fields) >= 9:
                try:
                    last_hsig[ip] = math.hypot(float(fields[6]), float(fields[7]))
                except ValueError:
                    pass
            elif kind.endswith("GGA") and len(fields) >= 15:
                lat = dm_to_degrees(fields[2], fields[3])
                lon = dm_to_degrees(fields[4], fields[5])
                if lat is None or lon is None:
                    continue
                samples.append(Sample(
                    t, lat, lon,
                    float(fields[9]) if fields[9] else None,
                    int(fields[6]) if fields[6] else 0,
                    int(fields[7]) if fields[7] else 0,
                    float(fields[13]) if fields[13] else None,
                    last_hsig.get(ip)))
    if not rover_ip and len(seen_ips) > 1:
        print("WARNING: log contains several rovers ({}) - mixing them all."
              .format(", ".join(sorted(seen_ips))))
        print("         Re-run with --rover-ip <ip> to isolate one.")
    return samples


# ---------------------------------------------------------------------------
#  Metrics
# ---------------------------------------------------------------------------

def enu_offsets(samples, lat0, lon0):
    """Equirectangular local projection: fine at metre scale."""
    m_lat = 111132.95
    m_lon = 111319.49 * math.cos(math.radians(lat0))
    return [((s.lon - lon0) * m_lon, (s.lat - lat0) * m_lat) for s in samples]


def percentile(sorted_values, q):
    if not sorted_values:
        return None
    k = (len(sorted_values) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return sorted_values[lo]
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (k - lo)


def subset_stats(samples):
    """Scatter statistics of a sample subset around its own mean."""
    if len(samples) < 2:
        return None
    lat0 = sum(s.lat for s in samples) / len(samples)
    lon0 = sum(s.lon for s in samples) / len(samples)
    enu = enu_offsets(samples, lat0, lon0)
    east = [e for e, _ in enu]
    north = [n for _, n in enu]
    n = len(enu)
    sd = lambda vals: math.sqrt(sum(v * v for v in vals) / (n - 1))
    radial = sorted(math.hypot(e, no) for e, no in enu)
    alts = [s.alt for s in samples if s.alt is not None]
    alt_sd = None
    if len(alts) > 1:
        alt_mean = sum(alts) / len(alts)
        alt_sd = math.sqrt(sum((a - alt_mean) ** 2 for a in alts) / (len(alts) - 1))
    return {
        "n": n, "lat0": lat0, "lon0": lon0,
        "sd_e": sd(east), "sd_n": sd(north),
        "rms": math.sqrt(sum(r * r for r in radial) / n),
        "cep50": percentile(radial, 0.50),
        "cep95": percentile(radial, 0.95),
        "max": radial[-1],
        "sd_alt": alt_sd,
    }


def build_metrics(samples):
    by_q = {}
    for s in samples:
        by_q.setdefault(s.quality, []).append(s)
    fixed = by_q.get(4, [])
    ages = [s.age for s in samples if s.age is not None]
    hsigs = [s.hsig for s in samples if s.hsig is not None]
    return {
        "total": len(samples),
        "t_start": samples[0].t, "t_end": samples[-1].t,
        "counts": {q: len(v) for q, v in by_q.items()},
        "all": subset_stats(samples),
        "fixed": subset_stats(fixed) if len(fixed) >= 2 else None,
        "age_mean": sum(ages) / len(ages) if ages else None,
        "age_max": max(ages) if ages else None,
        "hsig_mean": sum(hsigs) / len(hsigs) if hsigs else None,
    }


def analysis_text(m):
    """Plain-language verdict for the report, derived from the numbers."""
    total = m["total"]
    pct_fixed = 100.0 * m["counts"].get(4, 0) / total
    pct_float = 100.0 * m["counts"].get(5, 0) / total
    pct_single = 100.0 * m["counts"].get(1, 0) / total
    lines = []
    if m["fixed"] and pct_fixed >= 80:
        lines.append(
            "Solución sólida: {:.0f}% del tiempo en RTK FIXED. El 95% de las "
            "posiciones cae dentro de {:.1f} cm de su centro — precisión "
            "centimétrica real, medida.".format(pct_fixed, m["fixed"]["cep95"] * 100))
    elif m["fixed"]:
        lines.append(
            "RTK FIXED alcanzado el {:.0f}% del tiempo (FLOAT {:.0f}%). Cuando fija, "
            "el 95% cae en {:.1f} cm. Para subir el porcentaje: mejor vista de "
            "cielo en ambas antenas y menos superficies reflectantes cerca."
            .format(pct_fixed, pct_float, m["fixed"]["cep95"] * 100))
    elif pct_float > 30:
        lines.append(
            "El rover llega a RTK FLOAT ({:.0f}%) pero no cierra el FIXED: las "
            "correcciones llegan, pero las ambigüedades no resuelven. Causa "
            "típica: cielo parcial o multipath (antenas cerca de paredes/techos)."
            .format(pct_float))
    else:
        lines.append(
            "Predomina SINGLE ({:.0f}%): el receptor casi no aplicó correcciones "
            "durante la sesión. Revisar el enlace de correcciones antes de leer "
            "las métricas de dispersión.".format(pct_single))
    if m["age_max"] is not None and m["age_max"] > 5:
        lines.append(
            "La edad de corrección llegó a {:.0f} s (media {:.1f} s): hubo cortes "
            "en el enlace. Con LoRa, vigilar este número.".format(
                m["age_max"], m["age_mean"]))
    if m["fixed"] and m["hsig_mean"] is not None:
        lines.append(
            "El receptor auto-reporta ±{:.3f} m (GST medio); la dispersión "
            "empírica FIXED es RMS {:.3f} m — {}."
            .format(m["hsig_mean"], m["fixed"]["rms"],
                    "coherentes" if abs(m["hsig_mean"] - m["fixed"]["rms"]) <
                    max(0.05, m["fixed"]["rms"]) else "difieren, tomar la empírica"))
    return lines


# ---------------------------------------------------------------------------
#  Report
# ---------------------------------------------------------------------------

def nice_step(span):
    raw = span / 4.0
    mag = 10 ** math.floor(math.log10(raw)) if raw > 0 else 1
    for mult in (1, 2, 5, 10):
        if raw <= mult * mag:
            return mult * mag
    return 10 * mag


def svg_scatter(samples, stats):
    """Point cloud in local ENU metres, colored by fix state, with sigma rings."""
    size, pad = 640, 54
    enu = enu_offsets(samples, stats["lat0"], stats["lon0"])
    span = max(max(abs(v) for pair in enu for v in pair), 0.02) * 1.15
    scale = (size / 2 - pad) / span

    def px(e, n):
        return size / 2 + e * scale, size / 2 - n * scale

    parts = []
    # Hairline metre grid.
    step = nice_step(span)
    ticks = []
    v = -math.floor(span / step) * step
    while v <= span:
        ticks.append(v)
        v += step
    for t in ticks:
        x, _ = px(t, 0)
        _, y = px(0, t)
        parts.append('<line x1="{0:.1f}" y1="{1}" x2="{0:.1f}" y2="{2}" class="grid"/>'
                     .format(x, pad, size - pad))
        parts.append('<line x1="{1}" y1="{0:.1f}" x2="{2}" y2="{0:.1f}" class="grid"/>'
                     .format(y, pad, size - pad))
        label = "{:g}".format(round(t / step) * step)
        parts.append('<text x="{:.1f}" y="{}" class="tick">{}</text>'
                     .format(x, size - pad + 16, label))
        parts.append('<text x="{}" y="{:.1f}" class="tick" text-anchor="end">{}</text>'
                     .format(pad - 6, y + 3, label))

    # 1-sigma and 2-sigma rings around the subset mean.
    sigma = math.hypot(stats["sd_e"], stats["sd_n"]) / math.sqrt(2)
    for k in (1, 2):
        r = sigma * k * scale
        parts.append('<circle cx="{0}" cy="{0}" r="{1:.1f}" class="ring"/>'
                     .format(size / 2, r))
        parts.append('<text x="{:.1f}" y="{:.1f}" class="ringlbl">{}σ</text>'
                     .format(size / 2 + r * 0.7071 + 4, size / 2 - r * 0.7071 - 4, k))

    # Dots, oldest first so late (converged) samples paint on top.
    stride = max(1, len(samples) // MAX_PLOT_POINTS)
    for i in range(0, len(samples), stride):
        s, (e, n) = samples[i], enu[i]
        x, y = px(e, n)
        parts.append(
            '<circle cx="{:.1f}" cy="{:.1f}" r="3" class="q{}">'
            '<title>{} · E {:+.3f} m · N {:+.3f} m · sats {}</title></circle>'
            .format(x, y, s.quality, FIX_NAMES.get(s.quality, "?"), e, n, s.sats))

    parts.append('<line x1="{0}" y1="{1}" x2="{0}" y2="{2}" class="cross"/>'
                 .format(size / 2, size / 2 - 8, size / 2 + 8))
    parts.append('<line x1="{1}" y1="{0}" x2="{2}" y2="{0}" class="cross"/>'
                 .format(size / 2, size / 2 - 8, size / 2 + 8))
    parts.append('<text x="{}" y="{}" class="axis">metros Este →</text>'
                 .format(size / 2, size - 14))
    return ('<svg viewBox="0 0 {0} {0}" role="img" '
            'aria-label="Nube de puntos de posiciones">{1}</svg>'
            ).format(size, "".join(parts))


REPORT_TEMPLATE = r"""<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Reporte de precisión RTK</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
:root{color-scheme:light;
  --page:#f9f9f7; --surface:#fcfcfb; --ink:#0b0b0b; --ink2:#52514e;
  --muted:#898781; --grid:#e1e0d9; --baseline:#c3c2b7;
  --border:rgba(11,11,11,.10);
  --good:#0ca30c; --warn:#fab219; --serious:#ec835a; --dgps:#2a78d6;}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){color-scheme:dark;
  --page:#0d0d0d; --surface:#1a1a19; --ink:#ffffff; --ink2:#c3c2b7;
  --muted:#898781; --grid:#2c2c2a; --baseline:#383835;
  --border:rgba(255,255,255,.10); --dgps:#3987e5;}}
:root[data-theme=dark]{color-scheme:dark;
  --page:#0d0d0d; --surface:#1a1a19; --ink:#ffffff; --ink2:#c3c2b7;
  --muted:#898781; --grid:#2c2c2a; --baseline:#383835;
  --border:rgba(255,255,255,.10); --dgps:#3987e5;}
*{box-sizing:border-box;margin:0}
body{background:var(--page);color:var(--ink);
  font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif;padding:24px}
.wrap{max-width:1060px;margin:0 auto;display:grid;gap:20px}
h1{font-size:22px;font-weight:650;text-wrap:balance}
.sub{color:var(--ink2);font-size:13px}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px}
.tile{background:var(--surface);border:1px solid var(--border);border-radius:10px;
  padding:14px 16px}
.tile .k{font-size:11px;letter-spacing:.06em;text-transform:uppercase;color:var(--muted)}
.tile .v{font-size:30px;font-weight:650;margin-top:2px}
.tile .u{font-size:13px;color:var(--ink2)}
.panel{background:var(--surface);border:1px solid var(--border);border-radius:10px;
  padding:18px}
.panel h2{font-size:14px;font-weight:650;margin-bottom:10px}
.duo{display:grid;grid-template-columns:1fr 1fr;gap:20px}
@media (max-width:860px){.duo{grid-template-columns:1fr}}
svg{width:100%;height:auto;display:block}
.grid{stroke:var(--grid);stroke-width:1}
.tick{fill:var(--muted);font-size:10px;text-anchor:middle}
.axis{fill:var(--muted);font-size:11px;text-anchor:middle}
.ring{fill:none;stroke:var(--baseline);stroke-width:1;stroke-dasharray:4 4}
.ringlbl{fill:var(--muted);font-size:10px}
.cross{stroke:var(--ink);stroke-width:1.5}
.q4{fill:var(--good);fill-opacity:.8}.q5{fill:var(--warn);fill-opacity:.8}
.q2{fill:var(--dgps);fill-opacity:.8}
.q1,.q0,.q6{fill:var(--serious);fill-opacity:.7}
.map-reset{background:#fff;color:#0b0b0b;border:2px solid rgba(0,0,0,.25);
  border-radius:6px;padding:5px 10px;font:13px/1 system-ui,sans-serif;
  cursor:pointer;box-shadow:0 1px 4px rgba(0,0,0,.25)}
.map-reset:hover{background:#f2f2ef}
.legend{display:flex;gap:16px;flex-wrap:wrap;margin-top:10px;font-size:12.5px;
  color:var(--ink2)}
.legend span{display:inline-flex;align-items:center;gap:6px}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block}
#map{height:420px;border-radius:8px;border:1px solid var(--border)}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;
  font-size:13.5px}
th{color:var(--muted);font-size:11px;letter-spacing:.06em;text-transform:uppercase;
  text-align:right;padding:6px 10px;border-bottom:1px solid var(--grid)}
th:first-child,td:first-child{text-align:left}
td{padding:7px 10px;text-align:right;border-bottom:1px solid var(--grid)}
.analysis li{margin:6px 0 6px 18px;color:var(--ink2)}
.analysis li::marker{color:var(--muted)}
button{background:var(--surface);color:var(--ink);border:1px solid var(--border);
  border-radius:8px;padding:8px 14px;font:inherit;font-size:13px;cursor:pointer}
button:hover{border-color:var(--muted)}
button:focus-visible{outline:2px solid var(--dgps);outline-offset:2px}
.foot{color:var(--muted);font-size:12px}
</style></head><body>
<div class="wrap">
  <div>
    <h1>Reporte de precisión RTK</h1>
    <div class="sub">__SUBTITLE__</div>
  </div>
  <div class="tiles">__TILES__</div>
  <div class="duo">
    <div class="panel"><h2>Nube de puntos (metros, centrada en la media)</h2>
      __SCATTER__
      <div class="legend">
        <span><i class="dot" style="background:var(--good)"></i>RTK FIXED</span>
        <span><i class="dot" style="background:var(--warn)"></i>RTK FLOAT</span>
        <span><i class="dot" style="background:var(--dgps)"></i>DGPS</span>
        <span><i class="dot" style="background:var(--serious)"></i>SINGLE</span>
        <span>◦ anillos = 1σ y 2σ</span>
      </div>
    </div>
    <div class="panel"><h2>Mapa</h2><div id="map"></div>
      <div class="legend"><span>Capas: Satélite / Calles / Ortofoto (solo en
      esta máquina) · línea azul = recorrido · círculo verde = CEP95 ·
      ⟲ Reset reencuadra.</span></div>
    </div>
  </div>
  <div class="panel"><h2>Métricas</h2><table>
    <tr><th>Subconjunto</th><th>N</th><th>σ Este</th><th>σ Norte</th>
        <th>RMS horiz.</th><th>CEP50</th><th>CEP95</th><th>Máx.</th><th>σ Alt.</th></tr>
    __METRIC_ROWS__
  </table></div>
  <div class="panel analysis"><h2>Análisis</h2><ul>__ANALYSIS__</ul>
    <p style="margin-top:12px"><button id="copy">Copiar resumen</button>
    <span class="foot" id="copied"></span></p></div>
  <p class="foot">Generado __GENERATED__ · rtk_map.py · datos crudos en rtk_points.csv</p>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
var POINTS=__POINTS__, MEAN=__MEAN__, CEP95=__CEP95__, ORTHO=__ORTHO_URL__;
var COLORS={4:"#0ca30c",5:"#fab219",2:"#2a78d6",1:"#ec835a",0:"#ec835a",6:"#ec835a"};
try{
  var osm=L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    {maxZoom:22,maxNativeZoom:19,attribution:"&copy; OpenStreetMap"});
  var sat=L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/"+
    "World_Imagery/MapServer/tile/{z}/{y}/{x}",
    {maxZoom:22,maxNativeZoom:19,attribution:"Esri World Imagery"});
  var map=L.map("map",{layers:[sat]});
  var latlngs=POINTS.map(function(p){return [p[0],p[1]];});

  // The route, in chronological order.
  var track=L.polyline(latlngs,{color:"#2a78d6",weight:2.5,opacity:.9}).addTo(map);
  var dots=L.layerGroup(POINTS.map(function(p){
    return L.circleMarker([p[0],p[1]],{radius:3,stroke:false,fillOpacity:.75,
      fillColor:COLORS[p[2]]||"#898781"});})).addTo(map);
  var cep=L.circle(MEAN,{radius:Math.max(CEP95,0.05),color:"#0ca30c",weight:2,
    fillOpacity:.06}).addTo(map);
  L.marker(MEAN).addTo(map).bindPopup("Media: "+MEAN[0].toFixed(7)+", "
    +MEAN[1].toFixed(7));

  var overlays={"Recorrido":track,"Puntos":dots,"CEP95":cep};
  if(ORTHO){
    overlays["Ortofoto"]=L.tileLayer(ORTHO,
      {minZoom:__ORTHO_MIN__,maxZoom:22,maxNativeZoom:__ORTHO_MAX__}).addTo(map);
  }
  L.control.layers({"Satélite":sat,"Calles":osm},overlays).addTo(map);

  function resetView(){map.fitBounds(track.getBounds().pad(0.3));}
  resetView();
  var Reset=L.Control.extend({options:{position:"topleft"},
    onAdd:function(){
      var b=L.DomUtil.create("button","map-reset");
      b.textContent="⟲ Reset";
      b.title="Volver a encuadrar el recorrido";
      L.DomEvent.on(b,"click",function(e){L.DomEvent.stop(e);resetView();});
      L.DomEvent.disableClickPropagation(b);
      return b;}});
  map.addControl(new Reset());
}catch(e){document.getElementById("map").innerHTML=
  "<p style='padding:16px;color:#898781'>Mapa no disponible sin internet — "+
  "la nube de puntos de la izquierda contiene los mismos datos.</p>";}
document.getElementById("copy").addEventListener("click",function(){
  navigator.clipboard.writeText(__SUMMARY_TEXT__).then(function(){
    document.getElementById("copied").textContent="Copiado ✓";});});
</script></body></html>
"""


def fmt_m(value):
    if value is None:
        return "—"
    return "{:.3f} m".format(value)


def ortho_url(tiles_dir):
    """file:// tile template for a local gdal2tiles pyramid, or '' if absent."""
    import os
    from pathlib import Path
    if tiles_dir and os.path.isdir(tiles_dir):
        return Path(tiles_dir).resolve().as_uri() + "/{z}/{x}/{y}.png"
    return ""


def build_report(samples, metrics, out_path, tiles_dir=TILES_DIR):
    m = metrics
    span = m["t_end"] - m["t_start"]
    pct = {q: 100.0 * c / m["total"] for q, c in m["counts"].items()}
    headline = m["fixed"] or m["all"]
    headline_name = "RTK FIXED" if m["fixed"] else "todas las muestras"

    subtitle = "{} · {} puntos en {:.1f} min · {:.7f}, {:.7f}".format(
        datetime.fromtimestamp(m["t_start"]).strftime("%Y-%m-%d %H:%M"),
        m["total"], span / 60, m["all"]["lat0"], m["all"]["lon0"])

    tiles = []
    for label, value, unit in [
            ("CEP95 ({})".format(headline_name),
             "{:.1f}".format(headline["cep95"] * 100), "cm"),
            ("RMS horizontal", "{:.1f}".format(headline["rms"] * 100), "cm"),
            ("% RTK FIXED", "{:.0f}".format(pct.get(4, 0.0)), "% del tiempo"),
            ("Edad de corrección",
             "—" if m["age_mean"] is None else "{:.1f}".format(m["age_mean"]),
             "s (media)")]:
        tiles.append('<div class="tile"><div class="k">{}</div>'
                     '<div class="v">{}</div><div class="u">{}</div></div>'
                     .format(label, value, unit))

    rows = []
    for name, st in [("Todas", m["all"]), ("Solo RTK FIXED", m["fixed"])]:
        if st is None:
            continue
        rows.append("<tr><td>{}</td><td>{}</td>{}</tr>".format(
            name, st["n"],
            "".join("<td>{}</td>".format(fmt_m(v)) for v in
                    (st["sd_e"], st["sd_n"], st["rms"], st["cep50"],
                     st["cep95"], st["max"], st["sd_alt"]))))

    analysis = "".join("<li>{}</li>".format(t) for t in analysis_text(m))

    stride = max(1, len(samples) // MAX_PLOT_POINTS)
    points = [[round(s.lat, 8), round(s.lon, 8), s.quality]
              for s in samples[::stride]]

    summary = ("Precisión RTK — {} puntos, {:.1f} min | FIXED {:.0f}% | "
               "CEP95 {:.1f} cm | RMS {:.1f} cm | media {:.7f},{:.7f}").format(
        m["total"], span / 60, pct.get(4, 0.0), headline["cep95"] * 100,
        headline["rms"] * 100, m["all"]["lat0"], m["all"]["lon0"])

    html = (REPORT_TEMPLATE
            .replace("__SUBTITLE__", subtitle)
            .replace("__TILES__", "".join(tiles))
            .replace("__SCATTER__", svg_scatter(samples, m["all"]))
            .replace("__METRIC_ROWS__", "".join(rows))
            .replace("__ANALYSIS__", analysis)
            .replace("__GENERATED__", datetime.now().strftime("%Y-%m-%d %H:%M"))
            .replace("__POINTS__", json.dumps(points))
            .replace("__MEAN__", json.dumps(
                [round(m["all"]["lat0"], 8), round(m["all"]["lon0"], 8)]))
            .replace("__CEP95__", "{:.3f}".format(headline["cep95"]))
            .replace("__ORTHO_URL__", json.dumps(ortho_url(tiles_dir)))
            .replace("__ORTHO_MIN__", str(TILES_MIN_ZOOM))
            .replace("__ORTHO_MAX__", str(TILES_MAX_ZOOM))
            .replace("__SUMMARY_TEXT__", json.dumps(summary)))

    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(html)
    return summary


def write_csv(samples, path):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_iso", "lat", "lon", "alt_m", "quality",
                         "fix", "sats", "age_s", "gst_hsigma_m"])
        for s in samples:
            writer.writerow([
                datetime.fromtimestamp(s.t).isoformat(timespec="seconds"),
                "{:.8f}".format(s.lat), "{:.8f}".format(s.lon),
                s.alt if s.alt is not None else "",
                s.quality, FIX_NAMES.get(s.quality, "?"), s.sats,
                s.age if s.age is not None else "",
                "{:.3f}".format(s.hsig) if s.hsig is not None else ""])


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=SERIAL_PORT)
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--base-host", default=None,
                        help="inject corrections from this base while logging")
    parser.add_argument("--base-port", type=int, default=BASE_PORT)
    parser.add_argument("--minutes", type=float, default=None,
                        help="stop automatically after this long")
    parser.add_argument("--out", default=REPORT_FILE)
    parser.add_argument("--csv", default=CSV_FILE)
    parser.add_argument("--tiles", default=TILES_DIR,
                        help="local orthophoto tile folder (default: %(default)s)")
    parser.add_argument("--nmea-log", default=None, metavar="FILE",
                        help="build the report from the server's telemetry log "
                             "(standalone rover; no serial connection needed)")
    parser.add_argument("--rover-ip", default=None,
                        help="with --nmea-log: keep only this rover's lines")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        show_ports()
        return

    if args.nmea_log:
        try:
            samples = parse_telemetry_log(args.nmea_log, args.rover_ip)
        except OSError as error:
            print("Could not read {}: {}".format(args.nmea_log, error))
            return
    else:
        try:
            samples = collect(args)
        except serial.SerialException as error:
            print("Could not open {}: {}".format(args.port, error))
            return

    if len(samples) < 10:
        print("Only {} samples - not enough for statistics.".format(len(samples)))
        return

    metrics = build_metrics(samples)
    write_csv(samples, args.csv)
    summary = build_report(samples, metrics, args.out, args.tiles)

    print("\n" + summary)
    print("Report:  {}".format(args.out))
    print("Data:    {}".format(args.csv))


if __name__ == "__main__":
    main()
