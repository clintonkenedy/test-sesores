// =============================================================================
//  SmartMine FMS — BASE GPS RTK  (ESP32 estándar — PRUEBA)
//  Hardware: ESP32 estándar (WROOM / WROVER / DevKit v1)
//            *** NO es ESP32-C6 — pines y UARTs distintos ***
//
//  Función:
//    Lee correcciones RTCM3 del receptor GNSS base (Quectel LC29H-BS)
//    y las reenvía por USB-Serial al servidor Python.
//    El servidor las acumula, detecta el fin de cada corrección completa
//    y las envía al gateway Ethernet E90 → LoRa → repetidor → rovers.
//
//    Al INICIAR, si RESURVEY_ON_BOOT=1, ordena al receptor rehacer su
//    survey-in en la ubicación actual (borra la posición fijada en flash).
//    Sin esto, una base movida de sitio sigue transmitiendo su posición
//    vieja y los rovers descartan todas las correcciones.
//
//  Conexiones físicas:
//    TXD GPS base  →  GPIO26  (ESP32 RX1 ← recibe RTCM3 del receptor)
//    RXD GPS base  →  GPIO27  (ESP32 TX1 → comandos al receptor, OBLIGATORIO
//                              para la secuencia de re-survey y send_base_command.py)
//    USB           →  PC      (COM del servidor Python — datos binarios RTCM3 puros)
//    LED           →  GPIO2   (parpadeo de actividad)
//
//  CRÍTICO — Contaminación del canal serial:
//    El servidor lee el COM en modo binario puro. Cualquier texto impreso
//    por Serial (println/print) se mezcla con los bytes RTCM3 → basura al rover.
//    DEBUG_BASE 0 en producción. Solo activar con IDE abierto y servidor DESCONECTADO.
//
//  ⚠ ADVERTENCIA OPERATIVA — RESURVEY_ON_BOOT:
//    Abrir el puerto COM desde la PC (arrancar el servidor) RESETEA el ESP32
//    (línea DTR). Con RESURVEY_ON_BOOT=1 eso dispara un survey-in nuevo:
//    ~2-3 min sin correcciones válidas y la posición base se mueve unos
//    metros en cada arranque. Perfecto para instalar/mover la base;
//    para operación estable dejar RESURVEY_ON_BOOT en 0 y reflashear.
//
//  Librerías: ninguna adicional (solo Arduino ESP32 core)
//  Board:     "ESP32 Dev Module" — arduino-esp32 >= 2.0.0
// =============================================================================

// ── Debug ─────────────────────────────────────────────────────────────────────
// 0 = producción  → Serial lleva RTCM3 puro, nada de texto
// 1 = diagnóstico → Solo con Arduino IDE abierto, SERVIDOR DESCONECTADO
#define DEBUG_BASE  0

// ── Secuencia de re-survey al iniciar ────────────────────────────────────────
// 1 = al arrancar ordena survey-in nuevo (usar al instalar/mover la base)
// 0 = arranca pasivo, el receptor conserva su configuración (producción)
#define RESURVEY_ON_BOOT       0
#define RESURVEY_SAVE          1     // guardar el modo survey-in en flash del GPS
#define RESURVEY_RESTART_GPS   1     // reiniciar el receptor para aplicar ya
#define SURVEY_SECONDS       120     // duración del survey-in
#define SURVEY_ACC_M          30     // límite de precisión 3D aceptado (m)

// ── Pines ─────────────────────────────────────────────────────────────────────
#define PIN_RX_GPS   26    // TXD del GPS → ESP32 recibe RTCM3
#define PIN_TX_GPS   27    // RXD del GPS ← ESP32 envía comandos
#define PIN_LED       2    // LED interno DevKit (activo HIGH)

#define BAUD_GPS    115200
#define BAUD_USB    115200

// ── UART del GPS ──────────────────────────────────────────────────────────────
// UART1 con pines remapeados: GPIO26 RX / GPIO27 TX
HardwareSerial GPS_Serial(1);

// ── Contadores de monitoreo ───────────────────────────────────────────────────
unsigned long bytes_total   = 0;
unsigned long bytes_ventana = 0;
unsigned long t_ventana     = 0;
unsigned long t_ultimo_led  = 0;
bool          led_estado    = false;

// =============================================================================
//  Comandos PQTM al receptor (checksum NMEA calculado, nunca a mano)
// =============================================================================
void enviarComandoGPS(const char* cuerpo) {
  uint8_t cs = 0;
  for (const char* p = cuerpo; *p; p++) cs ^= (uint8_t)*p;

  char frase[96];
  snprintf(frase, sizeof(frase), "$%s*%02X\r\n", cuerpo, cs);
  GPS_Serial.print(frase);

#if DEBUG_BASE
  Serial.print(F("[CMD->GPS] "));
  Serial.print(frase);
#endif
}

void secuenciaResurvey() {
  // El LC29H tarda ~1 s en aceptar comandos tras energizarse
  delay(1200);

  // Modo 1 = survey-in (anula el modo fijo). ECEF en cero: solo los usa el modo 2.
  char cfg[64];
  snprintf(cfg, sizeof(cfg), "PQTMCFGSVIN,W,1,%d,%d,0,0,0",
           SURVEY_SECONDS, SURVEY_ACC_M);
  enviarComandoGPS(cfg);
  delay(300);

#if RESURVEY_SAVE
  enviarComandoGPS("PQTMSAVEPAR");   // persiste el modo en la flash del GPS
  delay(300);
#endif

#if RESURVEY_RESTART_GPS
  enviarComandoGPS("PQTMSRR");       // reinicio del receptor → survey arranca ya
#endif

  // Tres parpadeos largos = secuencia enviada
  for (int i = 0; i < 6; i++) { digitalWrite(PIN_LED, i % 2); delay(200); }
  digitalWrite(PIN_LED, LOW);
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // USB al servidor — en producción: RTCM3 binario puro (nada de texto)
  Serial.begin(BAUD_USB);

#if DEBUG_BASE
  delay(600);
  Serial.println();
  Serial.println(F("======================================================"));
  Serial.println(F("  SmartMine FMS — BASE GPS RTK (ESP32)  [DEBUG MODE]"));
  Serial.println(F("======================================================"));
  Serial.print(F("  GPS UART1  RX=GPIO")); Serial.print(PIN_RX_GPS);
  Serial.print(F("  TX=GPIO")); Serial.println(PIN_TX_GPS);
#if RESURVEY_ON_BOOT
  Serial.println(F("  RESURVEY_ON_BOOT=1: se ordenara survey-in nuevo"));
#endif
  Serial.println(F("  Desconecta el servidor antes de usar este modo."));
  Serial.println(F("======================================================"));
#endif

  GPS_Serial.setRxBufferSize(2048);  // Evita desbordamiento con burst RTCM3 de ~760 B
  GPS_Serial.begin(BAUD_GPS, SERIAL_8N1, PIN_RX_GPS, PIN_TX_GPS);

#if RESURVEY_ON_BOOT
  secuenciaResurvey();
#endif

  t_ventana    = millis();
  t_ultimo_led = millis();
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  unsigned long ahora = millis();

  // ── Comandos: USB → GPS (send_base_command.py, config del receptor) ───────
  // El servidor Python nunca escribe al COM, así que en producción esta vía
  // queda muda; solo transporta comandos cuando se configura a propósito.
  while (Serial.available()) {
    GPS_Serial.write(Serial.read());
  }

  // ── Reenvío RTCM3: GPS → USB (binario puro) ──────────────────────────────
  if (GPS_Serial.available()) {
    uint8_t buf[256];
    int n = GPS_Serial.readBytes(buf, min((int)GPS_Serial.available(), 256));
    if (n > 0) {
      Serial.write(buf, n);   // bytes RTCM3 puros → servidor
      bytes_total   += n;
      bytes_ventana += n;

      if (ahora - t_ultimo_led >= 50) {
        led_estado = !led_estado;
        digitalWrite(PIN_LED, led_estado);
        t_ultimo_led = ahora;
      }
    }
  }

  // ── Throughput cada segundo (solo en modo debug) ──────────────────────────
  if (ahora - t_ventana >= 1000) {
    unsigned long bps = bytes_ventana;
    bytes_ventana = 0;
    t_ventana     = ahora;

#if DEBUG_BASE
    Serial.print(F("[BASE] RTCM3: "));
    Serial.print(bps);
    Serial.print(F(" B/s | Total: "));
    Serial.print(bytes_total);
    Serial.println(F(" B"));
#else
    (void)bps;
#endif
  }
}
