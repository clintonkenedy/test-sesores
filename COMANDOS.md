# SmartMine RTK — Comandos de arranque y operación

Guía rápida del banco de pruebas validado el 2026-07-23 (Ananea).
Resultado de referencia: **RTK FIXED · CEP95 0.6 cm · 99.8% disponibilidad**.

---

## Arquitectura

```
LC29H-BS ──UART──► ESP32 base ──USB──► PC Windows ──WiFi/LAN──► ESP32 rover ──UART──► LC29H-DA
 (base fija)      (base_gps_esp32)     · server Python                (rover_esp32)    (rover)
                                       · dashboard :8765
                                       · rover_telemetry.log
```

Próxima etapa: reemplazar el tramo WiFi por E90-DTU LoRa 915 MHz (A → B repetidor → C).

## Firmware de cada ESP32

| Equipo | Sketch | Notas |
|---|---|---|
| ESP32 **base** | `base_gps_esp32/` | `DEBUG_BASE 0` · `RESURVEY_ON_BOOT 0` (antena fija) |
| ESP32 **rover** | `rover_esp32/` | WiFi standalone; editar SSID/clave/`BASE_HOST` antes de flashear |
| Diagnóstico | `usb_bridge/` | Puente transparente USB↔LC29H (para pruebas desde una computadora) |
| Diagnóstico | `rover_gnss_echo/`, `uart_loopback/` | ¿El GPS habla? / ¿Los pines TX2-RX2 funcionan? |

Cableado rover: LC29H TX→GPIO16, LC29H RX→GPIO17, GND común.
Cableado base:  LC29H TX→GPIO26, LC29H RX→GPIO27, GND común.

---

## Arranque normal (PC Windows, 2 consolas)

```cmd
:: Consola 1 — servidor de correcciones (elegir UNO):
python rtcm_to_lora.py --mode server --port COM5              :: filtro seguro (solo NMEA fuera)
python rtcm_to_lora.py --mode server --port COM5 --raw        :: crudo byte a byte (baseline)
python rtcm_optimizer.py --mode server --port COM5 --level 2  :: GPS+BeiDou (~440 B/s)
python rtcm_optimizer.py --mode server --port COM5 --level 3  :: + sin 1114 (~410 B/s)
python rtcm_optimizer.py --mode server --port COM5 --level 3 --epoch-div 2  :: máximo (~210 B/s)

:: Consola 2 — dashboard en vivo:
python rtk_live_server.py
```

Dashboard: `http://localhost:8765` (o `http://<ip-pc>:8765` desde celular/Mac).
Botones: **Seguir** (cámara al rover) · **⟲ Encuadrar** · **▶ Sesión** (reinicia métricas+cronómetro).
Encender el rover afuera → esperar `RTK FIXED age=1s` → **▶ Sesión** → medir.

## Reportes de precisión

```cmd
:: Desde el log del server (rover standalone):
python rtk_map.py --nmea-log rover_telemetry.log --out reporte.html
python rtk_map.py --nmea-log telemetria_raw.log --out reporte_raw.html
python rtk_map.py --nmea-log rover_telemetry.log --rover-ip 192.168.1.112   :: aislar un rover

:: Vista live simple (recarga cada 5 s, alternativa liviana al dashboard):
python rtk_map.py --nmea-log rover_telemetry.log --live 5
```

Genera `rtk_report.html` (mapa + nube + métricas + análisis) y `rtk_points.csv`.

## Sesión de pruebas A/B (protocolo completo: `plan_pruebas_rtk.xlsx`)

```cmd
:: rotar el log entre fases (server DETENIDO):
ren rover_telemetry.log telemetria_raw.log
```

Regla: 12 min estáticos por fase, nadie toca la antena, **▶ Sesión** al ver FIXED.

---

## Configurar la base (server detenido — el COM es exclusivo)

```cmd
python send_base_command.py --port COM5 --read                :: ver configuración actual
python send_base_command.py --port COM5 --survey 120 30       :: re-survey (solo si se MUEVE la base)
python send_base_command.py --port COM5 --fixed 2150874.220 -5789942.096 -1603624.640
python send_base_command.py --port COM5 --save                :: guardar en flash del GPS
python send_base_command.py --port COM5 --restart             :: aplicar
```

Coordenadas fijadas (Ananea, survey 2026-07-23): ECEF `2150874.220, -5789942.096, -1603624.640`
= lat/lon `-14.648249, -69.620745`, h 4544 m.

```bash
# Verificar qué posición transmite la base (desde cualquier máquina de la red):
python check_base_position.py --host 192.168.1.162
```

Debe decir esas coordenadas y `LIVE`. Si dice otra cosa, la base está mal configurada.

---

## Prueba del enlace LoRa (E90-DTU) — antes de mandar RTCM

Requisitos: ambos DTU con **antena conectada**, potencia mínima en mesa, mismo
canal/air-rate (915 MHz, 38.4k, packet 240), modo **TCP Server**, IP fija, y
Ethernet de ambos al router.

```cmd
:: Escalón 1 — salto directo A→C:
python lora_ping.py --tx 192.168.1.100:8887 --rx 192.168.1.101:8887

:: carga tipo correcciones:
python lora_ping.py --tx ... --rx ... --size 110 --interval 0.25

:: Escalón 2 — igual, con B en modo relay en el medio (misma línea de comando)
```

Pasa si: loss ≤1%, latencia estable 50–300 ms. `dup>0` en mesa = el repetidor repite (normal).

Cuando el enlace esté certificado, apuntar el server al DTU en vez de servir WiFi:

```cmd
python rtcm_optimizer.py --mode tcp --host 192.168.1.100 --link-port 8887 --level 2
```

---

## Fallas rápidas

| Síntoma | Revisar |
|---|---|
| No aparece COM | Driver CP210x / cable |
| `rover connected` no aparece | WiFi/clave del sketch, firewall de la PC |
| `FIX SINGLE age=?` eterno | ¿Base transmite la posición correcta? (`check_base_position`) · cable GPIO17→RX del rover |
| `age` sube sin techo | Enlace caído (WiFi fuera de alcance / server parado) |
| gnss_in=0 en el rover | Cable LC29H TX→GPIO16 o GND |
| Dashboard sin datos | ¿El server escribe `rover_telemetry.log`? ¿mismo directorio? |
| Puerto ocupado / access denied | Otro programa tiene el COM (cerrar Monitor Serie, un lector a la vez) |

## Lecciones que no hay que olvidar

1. **Nunca filtrar el mensaje MSM con DF393=0** (el 1114 "vacío" cierra la época). Nivel 3 lo resuelve reescribiendo DF393+CRC.
2. La corrección RTK se transmite **en vivo, jamás acumulada** — una corrección vieja es basura.
3. Base con antena fija = **modo fijo** (`--fixed`), no survey-in por arranque.
4. El COM y el canal LoRa son **exclusivos**: un proceso/uso a la vez.
5. Toda optimización se valida con A/B de 12 min contra baseline crudo.
