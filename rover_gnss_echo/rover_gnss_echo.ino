/*
 * GNSS echo test - isolates the LC29H -> ESP32 receive path.
 *
 * Does one thing: prints whatever the LC29H sends to GPIO16 (RX2) straight to
 * USB. No WiFi, no injection. Use it to answer a single question:
 *   is the receiver talking to the ESP32 at all?
 *
 *   NMEA text appears ($GNGGA, $GNRMC, ...)  -> RX wiring and baud are good;
 *                                               the problem is elsewhere.
 *   nothing / garbage                        -> LC29H TX -> GPIO16 wire, common
 *                                               GND, power, or baud is wrong.
 *
 * Wiring:
 *   LC29H TX  -> ESP32 GPIO16 (RX2)   <- the wire being tested
 *   GND       -> GND                  (common ground required)
 *
 * If nothing shows at 115200, change GNSS_BAUD to 9600, 38400, 460800 and retry.
 */

const uint32_t GNSS_BAUD = 115200;
const int PIN_RX2 = 16;   // ESP32 <- LC29H TX
const int PIN_TX2 = 17;   // unused here, declared for a complete Serial2 setup

void setup() {
  Serial.begin(115200);
  Serial2.begin(GNSS_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
  Serial.print("\nGNSS echo test at ");
  Serial.print(GNSS_BAUD);
  Serial.println(" baud. Listening on GPIO16...\n");
}

void loop() {
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
