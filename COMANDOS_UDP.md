# SmartMine RTK — Operación por UDP (transporte definitivo)

Runbook del gateway UDP (`lora_gateway_udp.py`). Reemplaza al transporte TCP
tras la falla de campo del 2026-07-24 (pila TCP del E90 saturada de zombies
al minuto 28). UDP no mantiene conexiones: **no hay nada en el DTU que se
pueda podrir**, y todo sobrevive a cualquier reinicio de cualquier lado.

---

## Arquitectura

```
LC29H-BS ─UART─► ESP32 base ─USB─► PC ══UDP══► E90 (.102) ~~915.125 MHz~~ E22 ─► C6 ─► LC29H-DA
 (base fija)                        │ :8888 ◄══UDP══ telemetría JSON ◄~~RF~~ (slot TDMA del camión)
                                    ▼
                       rover_telemetry.log ──► dashboard :8765
```

- Una sola antena (E90 .102), un solo socket UDP, cero conexiones.
- Radio de flota: canal 65 · **NET ID 20** · 38.4k · 240 B · transparente.

## Configuración del DTU (una sola vez, web del .102)

**Socket A:**

| Campo | Valor |
|---|---|
| Work mode | **UDP Client** |
| Local port | **8886** (recibe correcciones) |
| Dest IP | **192.168.4.100** (esta PC) |
| Dest port | **8888** (donde escucha el gateway) |

Guardar → reiniciar el DTU. Socket B deshabilitado. Wireless sin tocar.
"Timeout restart" (Advanced) en su default 300 s: es el auto-reboot bueno —
si el DTU se cuelga y deja de pasar datos, se reinicia solo.

⚠️ La IP de la PC en la red del switch DEBE ser fija (192.168.4.100): el DTU
manda la telemetría a esa dirección grabada.

## Archivos necesarios en la PC (los 4 juntos, versión actual)

`lora_gateway_udp.py` (el que se ejecuta) + `lora_gateway.py` +
`rtcm_optimizer.py` + `rtcm_to_lora.py` (le prestan funciones) +
`rtk_live_server.py` (dashboard).

## Comandos de operación (2 consolas, MISMA carpeta)

```cmd
:: ── Consola 1 · GATEWAY UDP ───────────────────────────────────────────────
python lora_gateway_udp.py --port COM5 --level 3 --epoch-div 2 --dtu 192.168.4.102:8886

:: ── Consola 2 · DASHBOARD ─────────────────────────────────────────────────
python rtk_live_server.py
::   navegador:  http://localhost:8765
::   celular:    http://192.168.1.162:8765  (WiFi Starlink)
```

Primera vez: Windows pregunta por el **firewall (UDP entrante) → Permitir**.

Variantes útiles:

```cmd
python lora_gateway_udp.py ... --level 2              &:: + GLONASS/Galileo fuera, terminador dentro
python lora_gateway_udp.py ... --epoch-div 1          &:: correcciones a 1 Hz
python lora_gateway_udp.py ... --rate-1005 3          &:: arranque de camiones más rápido
python lora_gateway_udp.py ... --local-port 9999      &:: si cambia el dest port del DTU
```

## Cómo leer la consola (la salud es POR DATOS — no hay "connected")

```
--- 120 s ---  out 180.2 B/s | telemetry lines 58 (last 2s ago)
```

| Señal | Lectura |
|---|---|
| `out ~180 B/s` estable | correcciones saliendo (nivel 3 div 2) |
| `(last 1-3s ago)` | enlace completo VIVO — es el "conectado" de UDP |
| `(last 45s ago)` + WARN stale | camión apagado, fuera de alcance, o DTU caído |
| `(never)` tras 1 min | telemetría no llega: Dest IP del DTU ≠ IP real de la PC, o firewall |
| `"f":4` en el JSON | RTK FIXED real en el camión 🎯 |

## Fallas rápidas

| Síntoma | Revisar |
|---|---|
| `(never)` eterno | `ipconfig` → ¿la PC es 192.168.4.100? · Dest IP/port del DTU · firewall UDP 8888 |
| out fluye pero camión en `"f":1` | como siempre: antena GNSS con cielo, NET ID 20 en todos, E22 115200 |
| `sendto failed` | cable/switch caído — al volver el cable, fluye solo (nada que reconectar) |
| Telemetría a saltos | mirar el tramo RF (distancia/obstáculos), no el cable |
| `... no data from COM5` | ESP32 base / USB / otro programa con el COM |

## Plan B — volver a TCP (si hiciera falta)

1. Web del DTU → Socket A → Work mode **TCP Server**, local port 8886
2. `python lora_gateway.py --port COM5 --level 3 --epoch-div 2 --dtu-a 192.168.4.102:8886 --dtu-c 192.168.4.102:8886`
   (misma IP en ambos = modo duplex, una sola conexión)

## Por qué UDP (para el que lea esto en el futuro)

- TCP guarda una "ficha" por conexión en la RAM del DTU; las conexiones mal
  cerradas quedan como zombies hasta llenar la tabla (~6) → el DTU acepta y
  mata todo lo nuevo. Nos pasó al minuto 28 de una caminata.
- UDP procesa y olvida: memoria constante, nada que limpiar, nada que
  reconectar. La pérdida de un datagrama en el cable (~nula, blindado, 20 m)
  cuesta lo mismo que un paquete de radio perdido: una época — invisible.
- La corrección RTK vieja es veneno: la retransmisión de TCP era un
  anti-feature. UDP pierde y sigue — la semántica exacta del RTK.
