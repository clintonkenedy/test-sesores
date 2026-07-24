// =============================================================================
//  SmartMine FMS — ROVER RTK CAM002  (Modo Silencioso + Testigos LED)
//  Hardware: ESP32-C6 + Quectel LC29HDA + Transmisor LoRa
//
//  V+patch: gps_fix ahora se lee del campo 6 del $GNGGA (calidad REAL del
//  RTK: 1=SINGLE 2=DGPS 4=RTK_FIXED 5=RTK_FLOAT). La versión anterior lo
//  sintetizaba con sats+HDOP y reportaba "f":4 sin tener RTK.
// =============================================================================

#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// ── Identificación y TDMA ─────────────────────────────────────────────────────
#define ID_CAMION   2
#define TRUCK_INDEX 1
#define SILENCIO_FIN_TRAMA  500
#define LORA_SLOT_MS        250

// ── Umbrales RTCM3 ────────────────────────────────────────────────────────────
#define MIN_BYTES_RTCM3    200
#define MAX_BYTES_RTCM3   1200

// ── PINES VALIDADOS ESP32-C6 ──────────────────────────────────────────────────
#define GNSS_RX_PIN 18  // Pad 9 físico en PCB (ESP RX <- GPS TX)
#define GNSS_TX_PIN 19  // Pad 10 físico en PCB (ESP TX -> GPS RX)
#define LORA_RX_PIN 20  // ESP RX <- LoRa TX
#define LORA_TX_PIN 21  // ESP TX -> LoRa RX

// ── PINES DE TESTIGOS LED ─────────────────────────────────────────────────────
#define PIN_LED_ERROR 7   // Enciende fijo si hay problema (Pérdida de Fix)
#define PIN_LED_TX    10  // Parpadea al transmitir datos (LoRa TX)

#define BAUD_GPS    115200
#define BAUD_LORA   115200

HardwareSerial GPS_Serial(1);
HardwareSerial LORA_Serial(0);
TinyGPSPlus gps;

// PATCH: lector del campo 6 del GGA (calidad de fix real, no sintetizada)
TinyGPSCustom ggaQuality(gps, "GNGGA", 6);

// ── Estado LoRa y Timers ──────────────────────────────────────────────────────
unsigned long t_ultimo_byte_lora   = 0;
unsigned long t_disparo_telemetria = 0;
unsigned long t_ultimo_envio       = 0;
unsigned long t_led_tx_apagado     = 0;  // Temporizador para apagar el LED TX
bool          rafaga_activa        = false;
int           bytes_recibidos      = 0;
bool          primera_valida       = false;

#define WATCHDOG_TX_MS (2000UL + (unsigned long)TRUCK_INDEX * 300UL)

// ── GNSS Variables ────────────────────────────────────────────────────────────
double gps_lat    = 0.0;
double gps_lon    = 0.0;
int    gps_fix    = 0;
int    gps_sats   = 0;
float  gps_hdop   = 99.0;
float  gps_vel    = 0.0;
bool   gps_valido = false;

// ── Sensores simulados (Para completar la trama JSON) ─────────────────────────
float sim_angulo = 0.0f;
int   sim_roll   = 0;
int   sim_pitch  = 0;
int   sim_tolva  = 0;
int   sim_health = 100;
int   sim_temp   = 75;

// ── Variables Auxiliares (RTCM y Cycle_ID) ────────────────────────────────────
enum RtcmInjState { RTCM_IDLE, RTCM_LEN1, RTCM_LEN2, RTCM_DATA, RTCM_CRC1, RTCM_CRC2, RTCM_CRC3 };
RtcmInjState rtcm_inj_state  = RTCM_IDLE;
uint16_t     rtcm_inj_remain = 0;

enum CidState { CID_IDLE, CID_C, CID_CI, CID_CID, CID_COLON, CID_DIGITS };
CidState      cid_state        = CID_IDLE;
uint16_t      cid_tmp          = 0;
uint16_t      current_cycle_id = 0;

char buffer_json[152];
int  json_idx        = 0;
bool capturando_json = false;

// =============================================================================
void setup() {
  // Configuramos los pines de los LEDs
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_TX, OUTPUT);

  // Encendemos ambos un segundo como "test de luces" al arrancar
  digitalWrite(PIN_LED_ERROR, HIGH);
  digitalWrite(PIN_LED_TX, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_ERROR, LOW);
  digitalWrite(PIN_LED_TX, LOW);

  GPS_Serial.setRxBufferSize(1024);
  GPS_Serial.begin(BAUD_GPS, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);

  LORA_Serial.setRxBufferSize(2048);
  LORA_Serial.begin(BAUD_LORA, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  delay(4000); // Completamos el retardo de estabilización
}

// =============================================================================
void loop() {
  unsigned long ahora = millis();

  // ── 1. LÓGICA DE ALARMA (LED GPIO 7) ────────────────────────────────────────
  // Si el GPS no tiene un Fix válido (0), asumimos que hay un problema de señal.
  if (!gps_valido || gps_fix == 0) {
    digitalWrite(PIN_LED_ERROR, HIGH); // Problema = LED ENCENDIDO
  } else {
    digitalWrite(PIN_LED_ERROR, LOW);  // Operación Normal = LED APAGADO
  }

  // ── 2. APAGADO NO BLOQUEANTE DEL LED TX ─────────────────────────────────────
  if (t_led_tx_apagado > 0 && ahora >= t_led_tx_apagado) {
    digitalWrite(PIN_LED_TX, LOW);
    t_led_tx_apagado = 0;
  }

  // ── A. Lectura GPS (NMEA Real) ──────────────────────────────────────────────
  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      if (gps.location.isUpdated() && gps.location.isValid()) {
        gps_lat    = gps.location.lat();
        gps_lon    = gps.location.lng();
        gps_valido = true;
      }
      if (gps.hdop.isUpdated())       gps_hdop = gps.hdop.hdop();
      if (gps.satellites.isUpdated()) gps_sats = gps.satellites.value();
      if (gps.speed.isUpdated())      gps_vel  = gps.speed.kmph();

      // PATCH: calidad de fix REAL desde el campo 6 del $GNGGA.
      // (Antes se sintetizaba con sats+HDOP y mentía "f":4 sin RTK.)
      if (ggaQuality.isUpdated()) gps_fix = atoi(ggaQuality.value());
    }
  }

  // ── B. Recepción LoRa (RTCM3 y Comandos) ────────────────────────────────────
  while (LORA_Serial.available()) {
    uint8_t b = LORA_Serial.read();
    t_ultimo_byte_lora = ahora;

    if (!rafaga_activa) {
      primera_valida   = (b == 0xD3);
      cid_state        = CID_IDLE;
      rtcm_inj_state   = RTCM_IDLE;
      rtcm_inj_remain  = 0;
    }
    rafaga_activa = true;
    bytes_recibidos++;

    if (bytes_recibidos > MAX_BYTES_RTCM3) {
      bytes_recibidos  = 0;
      primera_valida   = false;
      rafaga_activa    = false;
      rtcm_inj_state   = RTCM_IDLE;
      rtcm_inj_remain  = 0;
      while (LORA_Serial.available()) LORA_Serial.read();
      break;
    }

    // Inyección ciega de RTCM3 al GPS
    {
      bool _inj = false;
      switch (rtcm_inj_state) {
        case RTCM_IDLE:
          if (b == 0xD3) { rtcm_inj_state = RTCM_LEN1; _inj = true; }
          break;
        case RTCM_LEN1:
          rtcm_inj_remain = (uint16_t)(b & 0x03) << 8;
          rtcm_inj_state  = RTCM_LEN2;
          _inj = true;
          break;
        case RTCM_LEN2:
          rtcm_inj_remain |= b;
          rtcm_inj_state   = (rtcm_inj_remain > 0) ? RTCM_DATA : RTCM_CRC1;
          _inj = true;
          break;
        case RTCM_DATA:
          rtcm_inj_remain--;
          _inj = true;
          if (rtcm_inj_remain == 0) rtcm_inj_state = RTCM_CRC1;
          break;
        case RTCM_CRC1: rtcm_inj_state = RTCM_CRC2; _inj = true; break;
        case RTCM_CRC2: rtcm_inj_state = RTCM_CRC3; _inj = true; break;
        case RTCM_CRC3: rtcm_inj_state = RTCM_IDLE; _inj = true; break;
      }
      if (_inj) GPS_Serial.write(b);
    }

    // Lógica para atrapar Cycle_ID y V2V (Simulados en fondo)
    switch (cid_state) {
      case CID_IDLE: if (b == 'C') cid_state = CID_C; break;
      case CID_C:    cid_state = (b == 'I') ? CID_CI : CID_IDLE; break;
      case CID_CI:   cid_state = (b == 'D') ? CID_CID : CID_IDLE; break;
      case CID_CID:  if (b == ':') { cid_state = CID_COLON; cid_tmp = 0; } else cid_state = CID_IDLE; break;
      case CID_COLON:if (b >= '0' && b <= '9') { cid_tmp = b - '0'; cid_state = CID_DIGITS; } else cid_state = CID_IDLE; break;
      case CID_DIGITS:
        if      (b >= '0' && b <= '9') cid_tmp = cid_tmp * 10 + (b - '0');
        else if (b == '\n')            { current_cycle_id = cid_tmp; cid_state = CID_IDLE; }
        else                             cid_state = CID_IDLE;
        break;
    }

    if (b == '{') { capturando_json = true; json_idx = 0; }
    if (capturando_json) {
      if (json_idx >= 150) { capturando_json = false; json_idx = 0; }
      else {
        buffer_json[json_idx++] = (char)b;
        if (b == '\n') { buffer_json[json_idx] = '\0'; capturando_json = false; }
      }
    }
  }

  // ── C. Detectar fin de burst LoRa ───────────────────────────────────────────
  if (rafaga_activa && (ahora - t_ultimo_byte_lora >= SILENCIO_FIN_TRAMA)) {
    rafaga_activa = false;
    if (bytes_recibidos >= MIN_BYTES_RTCM3 && primera_valida) {
      t_disparo_telemetria = ahora + (unsigned long)TRUCK_INDEX * LORA_SLOT_MS;
    }
    bytes_recibidos = 0;
    primera_valida  = false;
  }

  // ── D. Watchdog autónomo ────────────────────────────────────────────────────
  if (t_disparo_telemetria == 0 && !rafaga_activa && (ahora - t_ultimo_envio >= WATCHDOG_TX_MS)) {
    t_disparo_telemetria = ahora;
  }

  // ── E. Actualizar sensores (Simulados matemáticamente) ──────────────────────
  sim_angulo += 0.003f;
  if (sim_angulo > 6.28318f) sim_angulo = 0.0f;
  sim_roll   = (int)round(2.5f  * sin(sim_angulo));
  sim_pitch  = (int)round(4.0f  * sin(sim_angulo * 0.7f));
  sim_health = 85 + (int)round(15.0f * (sin(sim_angulo * 0.3f) * 0.5f + 0.5f));
  sim_temp   = 70 + (int)round(20.0f * (sin(sim_angulo * 0.2f) * 0.5f + 0.5f));

  // ── F. Enviar telemetría JSON y PARPADEAR LED TX ────────────────────────────
  if (t_disparo_telemetria > 0 && ahora >= t_disparo_telemetria && !rafaga_activa) {
    StaticJsonDocument<256> doc;
    doc["c"] = ID_CAMION;
    doc["a"] = gps_valido ? round(gps_lat * 10000000.0) / 10000000.0 : 0.0;
    doc["o"] = gps_valido ? round(gps_lon * 10000000.0) / 10000000.0 : 0.0;
    doc["f"] = gps_fix;
    doc["s"] = gps_sats;
    doc["v"] = (gps_valido && gps_vel > 0.1f) ? (int)round(gps_vel * 10.0f) / 10.0f : 0.0f;
    doc["r"] = sim_roll;
    doc["p"] = sim_pitch;
    doc["u"] = sim_tolva;
    doc["h"] = sim_health;
    doc["t"] = sim_temp;
    doc["q"] = current_cycle_id;

    // Disparamos la ráfaga de datos por el transmisor LoRa
    serializeJson(doc, LORA_Serial);
    LORA_Serial.println();

    // ¡ACTIVAMOS EL LED TESTIGO DE TRANSMISIÓN!
    digitalWrite(PIN_LED_TX, HIGH);
    t_led_tx_apagado = ahora + 100; // Le decimos al sistema que lo apague en 100ms

    t_disparo_telemetria = 0;
    t_ultimo_envio       = ahora;
  }
}
