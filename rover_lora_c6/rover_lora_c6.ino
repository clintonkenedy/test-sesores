/*
 * SmartMine FMS V1.3 - Nodo GNSS RTK Rover con E22 DIRECTO (ESP32-C6)
 * Hardware: ESP32-C6 + Quectel LC29HDA + EBYTE E22-900T30D por UART TTL
 *
 * Flujo de Correcciones RTCM:
 *   E90-A (base) ~~RF~~ E22 -> UART0 (GPIO21) -> parser de frames -> LC29HDA
 *
 * Flujo de Telemetría (TDMA por beacon):
 *   LC29HDA -> GGA -> turno propio -> UART0 (GPIO20) -> E22 ~~RF~~ base
 *
 * CABLEADO:
 *   LC29H TX  -> GPIO18 (RX1)        E22 TXD -> GPIO21 (RX0 del C6)
 *   LC29H RX  -> GPIO19 (TX1)        E22 RXD -> GPIO20 (TX0 del C6)
 *   E22 M0,M1 -> GND (modo normal; config grabada con RF_Setting)
 *   E22 VCC   -> 5V (picos ~600 mA a 30 dBm)   GND común en todo
 *
 * RADIO (grabado previamente en el E22 con la herramienta de Ebyte):
 *   canal 65 (915.125 MHz) · NET ID 20 · air 38.4k · 240 B · transparente
 *   UART 115200 8N1 · address 0 · key 0     (idéntico al E90-A de la base)
 *
 * TDMA: la ráfaga de corrección (cada ~2 s) es el metrónomo. Al TERMINAR,
 * cada camión espera su turno (SLOT_BASE + índice*SLOT_WIDTH) y transmite su
 * GGA más fresco. Sin beacon por 5 s (sombra) -> transmisión libre con jitter.
 *
 * Diagnóstico remoto: responde solo a "PING T1" (dirigido), nunca a "PING"
 * pelado - un PING general haría chocar a los 5 camiones al responder juntos.
 */

#include <HardwareSerial.h>
#include <math.h>

// --- IDENTIDAD Y TURNO DE ESTE CAMIÓN ---
#define TRUCK_ID        "T1"
#define TRUCK_INDEX     1        // 1..5: turno tras cada corrección

// --- PLAN DE SLOTS (ms después del fin de la ráfaga de corrección) ---
#define SLOT_BASE_MS        200
#define SLOT_WIDTH_MS       150
#define BURST_MIN_BYTES     150
#define BURST_GAP_MS         60
#define FALLBACK_AFTER_MS  5000
#define FALLBACK_PERIOD_MS 2000

// --- PINES Y PUERTOS (ESP32-C6) ---
// 1. GNSS Quectel LC29HDA - UART1
#define GNSS_RX_PIN 18
#define GNSS_TX_PIN 19
#define GNSS_BAUD_RATE 115200
HardwareSerial SerialGNSS(1);

// 2. Radio E22-900T30D - UART0 (TTL directo, sin RS-485)
#define E22_RX_PIN 20            // C6 recibe  <- TXD del E22 (pinout validado en PCB)
#define E22_TX_PIN 21            // C6 envía   -> RXD del E22
#define E22_BAUD_RATE 115200     // igual a lo grabado en el E22
HardwareSerial SerialE22(0);

// --- ESTADO NMEA / RTK ---
char nmeaLine[128];
size_t nmeaLen = 0;

uint32_t rtcmBytesIn = 0;
uint32_t rtcmFrames  = 0;
uint32_t gnssBytesIn = 0;
uint32_t nmeaLines = 0;

String estadoRTK = "NO_FIX";
String satelites = "0";
String latitud = "0.000000";
String longitud = "0.000000";
String altitud = "0.00";
float lastHSigma = -1.0f;

String bufferCmd = "";           // ASCII fuera de frames (PING dirigido)

// --- PARSER DE FRAMES RTCM3 (0xD3 | largo 10 bits | payload | CRC24) ---
enum EstadoRTCM { FUERA_DE_FRAME, LARGO_1, LARGO_2, DENTRO_DE_FRAME };
EstadoRTCM estadoRtcm = FUERA_DE_FRAME;
uint16_t rtcmRestantes = 0;

// --- TDMA POR BEACON ---
char pendingGga[128];
bool ggaPending = false;
uint32_t burstBytes = 0;
uint32_t lastRtcmByte = 0;
uint32_t beaconAt = 0;
bool slotArmed = false;
uint32_t lastUplink = 0;
uint32_t helloSeq = 0;
const uint32_t SLOT_OFFSET_MS = SLOT_BASE_MS + (TRUCK_INDEX - 1) * SLOT_WIDTH_MS;

// ---------------------------------------------------------------------------

const char* fixName(int quality) {
  switch (quality) {
    case 0: return "NO_FIX";
    case 1: return "SINGLE";
    case 2: return "DGPS";
    case 4: return "RTK_FIXED";
    case 5: return "RTK_FLOAT";
    case 6: return "DEAD_RECKON";
    default: return "UNKNOWN";
  }
}

bool nmeaField(const char* line, int n, char* out, size_t outSize) {
  int field = 0;
  const char* p = line;
  while (field < n) {
    p = strchr(p, ',');
    if (!p) return false;
    p++;
    field++;
  }
  const char* end = strchr(p, ',');
  size_t len = end ? (size_t)(end - p) : strlen(p);
  if (len >= outSize) len = outSize - 1;
  memcpy(out, p, len);
  out[len] = '\0';
  return true;
}

void reportGGA(const char* line) {
  char quality[8], sats[8], lat[16], lon[16], alt[16];

  if (!nmeaField(line, 2, lat, sizeof(lat))) return;
  if (!nmeaField(line, 4, lon, sizeof(lon))) return;
  if (!nmeaField(line, 6, quality, sizeof(quality))) return;
  if (!nmeaField(line, 7, sats, sizeof(sats))) return;
  if (!nmeaField(line, 9, alt, sizeof(alt))) return;

  estadoRTK = String(fixName(atoi(quality)));
  if (sats[0]) satelites = String(sats);
  if (lat[0]) latitud = String(lat);
  if (lon[0]) longitud = String(lon);
  if (alt[0]) altitud = String(alt);

  Serial.printf("[RTK STATUS] Fix: %s | Sats: %s | Error H: %.3fm | RTCM In: %u B (%u frames)\n",
                estadoRTK.c_str(), satelites.c_str(), lastHSigma,
                rtcmBytesIn, rtcmFrames);
}

void reportGST(const char* line) {
  char latSigma[16], lonSigma[16];
  if (!nmeaField(line, 6, latSigma, sizeof(latSigma))) return;
  if (!nmeaField(line, 7, lonSigma, sizeof(lonSigma))) return;
  if (!latSigma[0] || !lonSigma[0]) return;

  float la = atof(latSigma), lo = atof(lonSigma);
  lastHSigma = sqrtf(la * la + lo * lo);
}

void handleNmeaLine() {
  nmeaLine[nmeaLen] = '\0';
  nmeaLines++;

  if (strstr(nmeaLine, "GGA")) {
    reportGGA(nmeaLine);
    // El GGA nuevo pisa al no enviado: una posición vieja no sirve.
    strncpy(pendingGga, nmeaLine, sizeof(pendingGga) - 1);
    pendingGga[sizeof(pendingGga) - 1] = '\0';
    ggaPending = true;
  } else if (strstr(nmeaLine, "GST")) {
    reportGST(nmeaLine);
  }
  nmeaLen = 0;
}

// Uplink del turno: GGA fresco con ID, o HELLO si aún no hay fix.
void enviarUplink(uint32_t ahora) {
  if (ggaPending) {
    SerialE22.print(TRUCK_ID " ");
    SerialE22.print(pendingGga);
    SerialE22.print("\r\n");
    ggaPending = false;
  } else {
    SerialE22.printf("%s HELLO seq=%lu rtcm_in=%lu frames=%lu\r\n",
                     TRUCK_ID, (unsigned long)++helloSeq,
                     (unsigned long)rtcmBytesIn, (unsigned long)rtcmFrames);
  }
  lastUplink = ahora;
  Serial.printf("[SLOT] Uplink a +%lu ms del beacon\n",
                (unsigned long)(ahora - beaconAt));
}

// CSV de estado, solo ante "PING T1" dirigido a ESTE camión.
void responderPing() {
  String respuesta = String(TRUCK_ID) + "," + estadoRTK + "," +
                     latitud + "," + longitud + "," + altitud + "," +
                     satelites + "," + String(lastHSigma, 3) + "\r\n";
  SerialE22.print(respuesta);
  Serial.print("[PING TX] " + respuesta);
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  SerialGNSS.begin(GNSS_BAUD_RATE, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  SerialGNSS.setRxBufferSize(2048);
  SerialE22.begin(E22_BAUD_RATE, SERIAL_8N1, E22_RX_PIN, E22_TX_PIN);
  SerialE22.setRxBufferSize(2048);   // una época completa llega en ráfaga

  randomSeed(esp_random());          // jitter del modo sombra único por camión

  Serial.println("\n=================================================");
  Serial.println(" SMARTMINE FMS: ROVER RTK + E22 DIRECTO (C6)     ");
  Serial.println("=================================================");
  Serial.printf("-> Camion %s | turno +%lu ms tras cada correccion\n",
                TRUCK_ID, (unsigned long)SLOT_OFFSET_MS);
  Serial.println("-> Radio esperado: ch65 / NETID 20 / 38.4k / 240B / 115200");
  Serial.println("-------------------------------------------------");
}

// ---------------------------------------------------------------------------

void loop() {
  uint32_t ahora = millis();

  // ── 1. GNSS -> parseo NMEA ───────────────────────────────────────────────
  while (SerialGNSS.available() > 0) {
    char c = SerialGNSS.read();
    gnssBytesIn++;
    if (c == '\n') handleNmeaLine();
    else if (c != '\r' && nmeaLen < sizeof(nmeaLine) - 1) nmeaLine[nmeaLen++] = c;
    else if (nmeaLen >= sizeof(nmeaLine) - 1) nmeaLen = 0;
  }

  // ── 2. E22 -> demultiplexado RTCM (binario) / comandos (ASCII) ──────────
  while (SerialE22.available() > 0) {
    uint8_t b = SerialE22.read();
    if (estadoRtcm != FUERA_DE_FRAME || b == 0xD3) {
      burstBytes++;
      lastRtcmByte = ahora;
    }

    switch (estadoRtcm) {

      case FUERA_DE_FRAME:
        if (b == 0xD3) {
          SerialGNSS.write(b);
          rtcmBytesIn++;
          estadoRtcm = LARGO_1;
        } else {
          char c = (char)b;
          if (c == '\n') {
            bufferCmd.trim();
            if (bufferCmd == "PING " TRUCK_ID) {   // solo dirigido a ESTE camión
              responderPing();
            }
            bufferCmd = "";
          } else if (c != '\r' && bufferCmd.length() < 20) {
            bufferCmd += c;
          }
        }
        break;

      case LARGO_1:
        SerialGNSS.write(b);
        rtcmBytesIn++;
        rtcmRestantes = (uint16_t)(b & 0x03) << 8;
        estadoRtcm = LARGO_2;
        break;

      case LARGO_2:
        SerialGNSS.write(b);
        rtcmBytesIn++;
        rtcmRestantes |= b;
        rtcmRestantes += 3;                  // + CRC-24Q
        estadoRtcm = (rtcmRestantes > 3) ? DENTRO_DE_FRAME : FUERA_DE_FRAME;
        break;

      case DENTRO_DE_FRAME:
        SerialGNSS.write(b);
        rtcmBytesIn++;
        if (--rtcmRestantes == 0) {
          rtcmFrames++;
          estadoRtcm = FUERA_DE_FRAME;
        }
        break;
    }
  }

  // ── 3. TDMA: fin de ráfaga = beacon; transmitir en el turno propio ──────
  if (burstBytes >= BURST_MIN_BYTES && ahora - lastRtcmByte > BURST_GAP_MS) {
    beaconAt = ahora;
    slotArmed = true;
    burstBytes = 0;
  } else if (burstBytes > 0 && ahora - lastRtcmByte > 500) {
    burstBytes = 0;              // bytes sueltos: no era una época
  }

  if (slotArmed && ahora - beaconAt >= SLOT_OFFSET_MS) {
    slotArmed = false;
    enviarUplink(ahora);
  }

  // Modo sombra: sin beacon (obstrucción) -> transmisión libre con jitter
  if (ahora - beaconAt > FALLBACK_AFTER_MS &&
      ahora - lastUplink > FALLBACK_PERIOD_MS + (uint32_t)random(0, 400)) {
    enviarUplink(ahora);
  }

  // ── 4. Salud ─────────────────────────────────────────────────────────────
  static uint32_t lastHealth = 0;
  if (ahora - lastHealth > 3000) {
    lastHealth = ahora;
    if (gnssBytesIn == 0) {
      Serial.println("[ALERTA] Sin datos del LC29HDA. Revisar GPIO18.");
    }
    if (rtcmBytesIn == 0) {
      Serial.println("[ALERTA] Sin RTCM del E22. Revisar GPIO21, NET ID (20) y canal (65).");
    }
  }
}
