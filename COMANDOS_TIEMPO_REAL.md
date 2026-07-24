# SmartMine RTK — Operación en TIEMPO REAL por LoRa (una antena)

Runbook de la etapa actual: correcciones y telemetría por radio, sin WiFi en
el camión, dashboard en vivo. (El banco WiFi anterior quedó en `COMANDOS.md`.)

---

## Arquitectura

```
LC29H-BS ──UART──► ESP32 base ──USB──► PC ──TCP──► E90-A ~~915.125 MHz~~ E22 ──UART──► C6 ──UART──► LC29H-DA
 (base fija)     (base_gps_esp32)      │   x2                            (GPIO21/20)  (camión T1..T5)
                                       │◄── telemetría "T1 $GNGGA..." ◄~~RF~~ (turno TDMA del camión)
                                       ▼
                          rover_telemetry.log ──► dashboard :8765
```

Firmware del camión: el nodo CAM00x propio (JSON) o **`rover_lora_c6/`**.
Cableado validado en PCB: `E22 TXD → GPIO20` · `E22 RXD → GPIO21` ·
`M0,M1 → GND` · `E22 VCC → 5V` · LC29H en GPIO18/19 · GND común.

- **Una sola radio en la base (E90-A)**: transmite y recibe. El TDMA separa en
  tiempo: corrección (~150 ms) → turnos T1..T5 → aire libre → repite cada 2 s.
- E90-C y E90-B: repuestos guardados.

## Estándar de radio de la flota (TODOS iguales)

| Parámetro | Valor |
|---|---|
| Canal | **65** (= 915.125 MHz) |
| **NET ID** | **20** |
| Air rate | 38.4 kbps |
| Paquete | 240 B |
| Modo | Transparente / Normal |
| Address / Key | 0 / 0 |
| Relay / LBT / WOR | Disable |
| UART del E22 | 115200 8N1 |
| Potencia | 30 dBm (mesa: antenas separadas ≥5-10 m) |

⚠️ El NET ID es el apellido de la familia: **uno distinto = radio sorda**.
Los E90 vienen de nuestra config anterior en 10 → cambiar a 20 en su web.

## Identidad de cada camión (en el firmware del C6)

```cpp
#define TRUCK_ID     "T1"     // T1..T5
#define TRUCK_INDEX  1        // 1..5 → turno +200/+350/+500/+650/+800 ms
```

---

## Archivos que deben estar en la PC (versión actual)

`lora_gateway.py` · `rtcm_to_lora.py` · `rtcm_optimizer.py` ·
`rtk_live_server.py` · `rtk_map.py` · `lora_ping.py` · `check_base_position.py`

## Orden de encendido

```
1. E90-A con antena y energía → web: NET ID 20 → guardar/reiniciar
2. Base (LC29H + ESP32) por USB a la PC
3. Consola 1: gateway          (comando abajo)
4. Consola 2: dashboard        (comando abajo)
5. Camión: C6 flasheado (rover_rs485_c6 V1.2) + E22 grabado + antenas
```

## Comandos de operación

```cmd
cd C:\ruta\de\trabajo        &:: gateway y dashboard: MISMA carpeta (mismo log)

:: ── Consola 1 · GATEWAY (correcciones + telemetría por la misma antena A) ──
python lora_gateway.py --port COM5 --level 3 --epoch-div 2 --dtu-a 192.168.4.103:8887 --dtu-c 192.168.4.103:8887

:: ── Consola 2 · DASHBOARD en tiempo real ──────────────────────────────────
python rtk_live_server.py
::   navegador:  http://localhost:8765
::   celular:    http://192.168.1.162:8765   (WiFi Starlink)
```

Variantes del gateway:

```cmd
:: correcciones cada 1 s (más convergencia, más aire ocupado):
python lora_gateway.py --port COM5 --level 3 --dtu-a ... --dtu-c ...
:: más constelaciones (GPS+BDS+terminador QZSS):
python lora_gateway.py --port COM5 --level 2 --epoch-div 2 --dtu-a ... --dtu-c ...
:: repetir 1005 más seguido (arranque de camiones más rápido):
python lora_gateway.py ... --rate-1005 3
```

## Qué debe verse (en orden)

**Gateway:**
```
[C] telemetry: connected to 192.168.4.103:8887
--- 10 s ---  [A connected] out ~190 B/s | [C] telemetry lines N
<- T1  HELLO seq=...      (cuando el camión aún no tiene fix)
```

**Monitor USB del C6 (solo diagnóstico, no hace falta en operación):**
```
[RTK STATUS] Fix: SINGLE  | RTCM In: creciendo (NN frames)  ← parser comiendo frames
[SLOT] Uplink a +200 ms del beacon                          ← turno correcto
[RTK STATUS] Fix: RTK_FLOAT → RTK_FIXED                     ← meta
```

**Dashboard:** T1 aparece con su color → fix escala → nube de puntos en cm.
Botones: Seguir · ⟲ Encuadrar · **▶ Sesión** (reinicia métricas y cronómetro).

## Reportes de precisión (después de una sesión)

```cmd
python rtk_map.py --nmea-log rover_telemetry.log --out reporte_lora.html
python rtk_map.py --nmea-log rover_telemetry.log --rover-ip T1   &:: un camión solo
```

## Diagnóstico del enlace (con el gateway APAGADO — canal exclusivo)

```cmd
:: ¿el DTU responde por cable?
ping 192.168.4.103
:: ¿la base sigue transmitiendo su posición correcta? (desde la Mac)
python check_base_position.py --host 192.168.1.162
```

## Fallas rápidas

| Síntoma | Revisar |
|---|---|
| Gateway `cannot reach 192.168.4.103` | E90-A: Socket A en TCP Server, IP, cable al switch |
| C6 `RTCM In: 0 B` | **NET ID distinto (10 vs 20)** · conversor RS-485 · UART E22 ≠ 115200 |
| Frames suben pero `SINGLE` eterno | Antena GNSS del camión sin cielo · base apagada |
| No llega telemetría al gateway | DE/RE del RS-485 · el C6 no oye ráfagas (mirar `[SLOT]`) |
| Pérdidas raras con radios cerca | Saturación por 30 dBm → separar antenas ≥5-10 m |
| `... no data from COM5` | ESP32 base / USB / otro programa con el COM |
| `Acceso denegado` en el COM | Otra consola/Monitor Serie tiene el puerto: cerrarla |

## Reglas de oro

1. **Un proceso por COM** — cerrar Monitores Serie y consolas sobrantes.
2. Gateway y dashboard **desde la misma carpeta** (comparten el log).
3. Correcciones **nunca** se acumulan ni se retrasan; telemetría solo en su turno.
4. Cualquier cambio de optimización se valida A/B (12 min) antes de adoptarlo.
5. Antenas: GNSS con cielo abierto; radios con antena SIEMPRE antes de energizar.
