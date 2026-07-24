/*
 * SmartMine FMS V1.1 - Nodo Integrado GNSS RTK Rover (Sin WiFi)
 * Hardware: ESP32-C6 + Quectel LC29HDA + Enlace RS-485
 *
 * Flujo de Correcciones RTCM:
 *   Bus RS-485 (Maestro) -> SerialRS485 -> SerialGNSS -> Quectel LC29HDA
 *
 * Flujo de Coordenadas y Estado RTK:
 *   Quectel LC29HDA -> SerialGNSS -> ESP32-C6 (Parseo GGA/GST) -> Respuesta a PING por RS-485
 *
 * V1.1 - CORRECCIÓN CRÍTICA del demultiplexado RTCM/ASCII en RS-485:
 *   La versión anterior solo inyectaba al GNSS los bytes 0xD3 o >127.
 *   Un frame RTCM3 es binario y la mayoría de sus bytes son <128 (el largo,
 *   medio payload), así que llegaban frames agujereados -> CRC inválido ->
 *   el receptor descartaba TODAS las correcciones (NO_FIX eterno).
 *   Ahora un parser de frames cuenta el largo declarado en la cabecera y
 *   pasa el frame COMPLETO al GNSS; solo los bytes fuera de un frame se
 *   interpretan como comandos ASCII (PING).
 *
 * V1.2 - TELEMETRÍA ESPONTÁNEA CON TDMA POR BEACON (uplink a la antena C):
 *   La ráfaga de corrección (cada ~2 s) es el metrónomo compartido: cuando
 *   TERMINA, cada camión espera su turno (SLOT_BASE + índice*SLOT_WIDTH) y
 *   recién ahí transmite su GGA por el bus -> E22 -> aire -> E90-C. Así los
 *   5 camiones jamás se pisan entre sí ni pisan las correcciones.
 *   Si no se oyen correcciones por 5 s (sombra), pasa a transmisión libre
 *   cada ~2 s con jitter aleatorio, y se re-sincroniza solo al volver.
 *   La respuesta a PING se conserva para diagnóstico.
 *
 * REQUISITO (una sola vez, con la herramienta de Ebyte + USB-TTL): el E22
 * del camión debe quedar en modo transparente, UART 115200 8N1, canal 65,
 * NetID 10, air rate 38.4k, paquete 240 B, y M0/M1 cableados a GND (modo
 * normal). En esta arquitectura nadie mueve M0/M1, así que la config debe
 * estar grabada de antemano.
 */

// --- IDENTIDAD Y TURNO DE ESTE CAMIÓN ---
#define TRUCK_ID        "T1"
#define TRUCK_INDEX     1        // 1..5: define el turno tras cada corrección

// --- PLAN DE SLOTS (ms después del fin de la ráfaga de corrección) ---
#define SLOT_BASE_MS        200
#define SLOT_WIDTH_MS       150
#define BURST_MIN_BYTES     150   // menos que esto no es una época, es ruido
#define BURST_GAP_MS         60   // silencio que marca el fin de la ráfaga
#define FALLBACK_AFTER_MS  5000   // sin beacon por este tiempo => modo sombra
#define FALLBACK_PERIOD_MS 2000

#include <HardwareSerial.h>
#include <math.h>

// --- CONFIGURACIÓN DE PINES Y PUERTOS (ESP32-C6) ---
// 1. Módulo GNSS RTK (Quectel LC29HDA) - Asignado a UART1
#define GNSS_RX_PIN 18
#define GNSS_TX_PIN 19
#define GNSS_BAUD_RATE 115200
HardwareSerial SerialGNSS(1);

// 2. Bus RS-485 (Enlace de Datos / Correcciones RTCM) - Asignado a UART0
#define RS485_RX_PIN 6
#define RS485_TX_PIN 4
#define RS485_DE_RE_PIN 5
#define RS485_BAUD_RATE 115200
HardwareSerial SerialRS485(0);

// --- VARIABLES GLOBALES PARA PARSEO NMEA Y ESTADO RTK ---
char nmeaLine[128];
size_t nmeaLen = 0;

uint32_t rtcmBytesIn = 0;      // Contador de bytes RTCM inyectados al Quectel
uint32_t rtcmFrames  = 0;      // Frames RTCM completos inyectados
uint32_t gnssBytesIn = 0;      // Contador de bytes recibidos del Quectel
uint32_t nmeaLines = 0;        // Contador de sentencias NMEA completas

// Variables de Telemetría RTK almacenadas
String estadoRTK = "NO_FIX";
String satelites = "0";
String latitud = "0.000000";
String longitud = "0.000000";
String altitud = "0.00";
float lastHSigma = -1.0f;

// Búfer para comandos de control en el bus RS-485
String bufferRS485 = "";

// --- PARSER DE FRAMES RTCM3 SOBRE RS-485 ---
// Un frame RTCM3: 0xD3 | 2 bytes con el largo (10 bits) | payload | CRC(3).
// Dentro de un frame TODO byte va al GNSS, valga lo que valga.
enum EstadoRTCM { FUERA_DE_FRAME, LARGO_1, LARGO_2, DENTRO_DE_FRAME };
EstadoRTCM estadoRtcm = FUERA_DE_FRAME;
uint16_t rtcmRestantes = 0;

// --- ESTADO DEL TDMA POR BEACON (uplink en el turno propio) ---
char pendingGga[128];            // el GGA más fresco esperando su turno
bool ggaPending = false;
uint32_t burstBytes = 0;         // bytes RTCM de la ráfaga en curso
uint32_t lastRtcmByte = 0;       // millis del último byte RTCM recibido
uint32_t beaconAt = 0;           // millis del fin de la última ráfaga
bool slotArmed = false;
uint32_t lastUplink = 0;
uint32_t helloSeq = 0;
const uint32_t SLOT_OFFSET_MS = SLOT_BASE_MS + (TRUCK_INDEX - 1) * SLOT_WIDTH_MS;

// ---------------------------------------------------------------------------
// FUNCIONES DE INTERPRETACIÓN RTK (Lógica nativa conservada)
// ---------------------------------------------------------------------------

const char* fixName(int quality) {
  switch (quality) {
    case 0: return "NO_FIX";
    case 1: return "SINGLE";
    case 2: return "DGPS";
    case 4: return "RTK_FIXED";   // El estado óptimo de máxima precisión
    case 5: return "RTK_FLOAT";   // Convergiendo a Fixed
    case 6: return "DEAD_RECKON";
    default: return "UNKNOWN";
  }
}

// Extracción rápida de campos delimitados por comas en tramas NMEA
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

// Procesa la trama $GNGGA para extraer latitud, longitud, altitud y calidad RTK
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

  // Monitoreo de depuración vía USB
  Serial.printf("[RTK STATUS] Fix: %s | Sats: %s | Error H: %.3fm | RTCM In: %u B (%u frames)\n",
                estadoRTK.c_str(), satelites.c_str(), lastHSigma,
                rtcmBytesIn, rtcmFrames);
}

// Procesa la trama $GNGST para calcular la precisión horizontal estimada (Sigma)
void reportGST(const char* line) {
  char latSigma[16], lonSigma[16];
  if (!nmeaField(line, 6, latSigma, sizeof(latSigma))) return;
  if (!nmeaField(line, 7, lonSigma, sizeof(lonSigma))) return;
  if (!latSigma[0] || !lonSigma[0]) return;

  float la = atof(latSigma), lo = atof(lonSigma);
  lastHSigma = sqrtf(la * la + lo * lo);
}

// Evalúa la línea NMEA completa leída desde el Quectel
void handleNmeaLine() {
  nmeaLine[nmeaLen] = '\0';
  nmeaLines++;

  if (strstr(nmeaLine, "GGA")) {
    reportGGA(nmeaLine);
    // Encolar para el turno de este camión: el GGA nuevo pisa al no enviado
    // (una posición vieja no le sirve a nadie).
    strncpy(pendingGga, nmeaLine, sizeof(pendingGga) - 1);
    pendingGga[sizeof(pendingGga) - 1] = '\0';
    ggaPending = true;
  } else if (strstr(nmeaLine, "GST")) {
    reportGST(nmeaLine);
  }
  nmeaLen = 0;
}

// Transmite una línea por el bus RS-485 (-> conversor -> E22 -> aire -> C)
void transmitirRS485(const String& linea) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(50);
  SerialRS485.print(linea);
  SerialRS485.flush();
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

// Uplink del turno: el GGA más fresco con el ID del camión, o un HELLO
// para que la base sepa que el nodo vive aunque no haya fix todavía.
void enviarUplink(uint32_t ahora) {
  if (ggaPending) {
    transmitirRS485(String(TRUCK_ID " ") + pendingGga + "\r\n");
    ggaPending = false;
  } else {
    transmitirRS485(String(TRUCK_ID) + " HELLO seq=" + String(++helloSeq) +
                    " rtcm_in=" + String(rtcmBytesIn) +
                    " frames=" + String(rtcmFrames) + "\r\n");
  }
  lastUplink = ahora;
  Serial.printf("[SLOT] Uplink a +%lu ms del beacon\n",
                (unsigned long)(ahora - beaconAt));
}

// Responde al maestro con la telemetría y el estado exacto del RTK
// Formato CSV: ESTADO_RTK,LATITUD,LONGITUD,ALTITUD,SATS,ERROR_SIGMA_METROS
void responderPing() {
  String respuesta = estadoRTK + "," +
                     latitud + "," +
                     longitud + "," +
                     altitud + "," +
                     satelites + "," +
                     String(lastHSigma, 3) + "\n";

  // Protocolo de transmisión física RS-485
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(50);

  SerialRS485.print(respuesta);
  SerialRS485.flush();

  digitalWrite(RS485_DE_RE_PIN, LOW);

  Serial.print("[RS-485 TX] Telemetría enviada: " + respuesta);
}

// ---------------------------------------------------------------------------
// CONFIGURACIÓN PRINCIPAL
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inicialización de puertos hardware independientes del ESP32-C6
  SerialGNSS.begin(GNSS_BAUD_RATE, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  SerialGNSS.setRxBufferSize(2048);   // Ráfagas NMEA sin desbordes
  SerialRS485.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  SerialRS485.setRxBufferSize(2048);  // Una época RTCM completa (~400-760 B)

  // Configuración de control RS-485 (Escucha activa de comandos o RTCM)
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  randomSeed(esp_random());   // jitter del modo sombra distinto por camión

  Serial.println("\n=================================================");
  Serial.println(" SMARTMINE FMS: ROVER RTK + ENLACE DE DATOS C6   ");
  Serial.println("=================================================");
  Serial.printf("-> Camion %s | turno +%lu ms tras cada correccion\n",
                TRUCK_ID, (unsigned long)SLOT_OFFSET_MS);
  Serial.println("-> Listo. Esperando tramas RTCM3 (y PING de diagnostico)...");
  Serial.println("-------------------------------------------------");
}

// ---------------------------------------------------------------------------
// BUCLE PRINCIPAL (Rutinas No Bloqueantes)
// ---------------------------------------------------------------------------

void loop() {
  // =========================================================================
  // 1. LECTURA DEL GNSS Y PARSEO NMEA (Quectel -> ESP32-C6)
  // =========================================================================
  while (SerialGNSS.available() > 0) {
    char c = SerialGNSS.read();
    gnssBytesIn++;

    if (c == '\n') {
      handleNmeaLine();
    } else if (c != '\r' && nmeaLen < sizeof(nmeaLine) - 1) {
      nmeaLine[nmeaLen++] = c;
    } else if (nmeaLen >= sizeof(nmeaLine) - 1) {
      nmeaLen = 0; // Desbordamiento protegido: reiniciar búfer
    }
  }

  // =========================================================================
  // 2. RECEPCIÓN POR RS-485: demultiplexado RTCM (binario) / comandos (ASCII)
  //    Parser de frames: dentro de un frame RTCM, TODO byte va al GNSS.
  // =========================================================================
  uint32_t ahora = millis();

  while (SerialRS485.available() > 0) {
    uint8_t b = SerialRS485.read();
    if (estadoRtcm != FUERA_DE_FRAME || b == 0xD3) {
      burstBytes++;              // byte de corrección: alimenta el beacon
      lastRtcmByte = ahora;
    }

    switch (estadoRtcm) {

      case FUERA_DE_FRAME:
        if (b == 0xD3) {                     // Arranca un frame RTCM3
          SerialGNSS.write(b);
          rtcmBytesIn++;
          estadoRtcm = LARGO_1;
        } else {
          // Fuera de frame: es tráfico ASCII (consulta PING del maestro)
          char c = (char)b;
          if (c == '\n') {
            bufferRS485.trim();
            if (bufferRS485 == "PING") {
              responderPing();
            }
            bufferRS485 = "";
          } else if (c != '\r' && bufferRS485.length() < 20) {
            bufferRS485 += c;
          }
        }
        break;

      case LARGO_1:                          // Byte alto del largo (2 bits útiles)
        SerialGNSS.write(b);
        rtcmBytesIn++;
        rtcmRestantes = (uint16_t)(b & 0x03) << 8;
        estadoRtcm = LARGO_2;
        break;

      case LARGO_2:                          // Byte bajo del largo
        SerialGNSS.write(b);
        rtcmBytesIn++;
        rtcmRestantes |= b;
        rtcmRestantes += 3;                  // + CRC-24Q al final del frame
        estadoRtcm = (rtcmRestantes > 3) ? DENTRO_DE_FRAME : FUERA_DE_FRAME;
        break;

      case DENTRO_DE_FRAME:                  // Payload + CRC, contados
        SerialGNSS.write(b);
        rtcmBytesIn++;
        if (--rtcmRestantes == 0) {
          rtcmFrames++;
          estadoRtcm = FUERA_DE_FRAME;
        }
        break;
    }
  }

  // =========================================================================
  // 3. TDMA: fin de ráfaga = beacon; transmitir en el turno de ESTE camión
  // =========================================================================
  if (burstBytes >= BURST_MIN_BYTES && ahora - lastRtcmByte > BURST_GAP_MS) {
    beaconAt = ahora;            // la ráfaga terminó: arranca la cuenta
    slotArmed = true;
    burstBytes = 0;
  } else if (burstBytes > 0 && ahora - lastRtcmByte > 500) {
    burstBytes = 0;              // bytes sueltos, no era una época completa
  }

  if (slotArmed && ahora - beaconAt >= SLOT_OFFSET_MS) {
    slotArmed = false;
    enviarUplink(ahora);
  }

  // Modo sombra: sin correcciones (obstrucción), transmisión libre con jitter
  if (ahora - beaconAt > FALLBACK_AFTER_MS &&
      ahora - lastUplink > FALLBACK_PERIOD_MS + (uint32_t)random(0, 400)) {
    enviarUplink(ahora);
  }

  // =========================================================================
  // 4. DIAGNÓSTICO DE SALUD DE CONEXIÓN
  // =========================================================================
  static uint32_t lastHealth = 0;
  if (ahora - lastHealth > 3000) {
    lastHealth = ahora;
    if (gnssBytesIn == 0) {
      Serial.println("[ALERTA] No se detectan datos del Quectel LC29HDA. Revisar RX GPIO18.");
    }
  }
}
