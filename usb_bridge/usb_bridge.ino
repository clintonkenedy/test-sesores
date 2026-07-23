/*
 * Transparent USB <-> LC29H bridge.
 *
 * Passes bytes straight through in both directions so a computer on the USB
 * port talks directly to the LC29H, with no WiFi and no logic in the way:
 *
 *   Mac -> USB -> ESP32 -> Serial2 (TX2/GPIO17) -> LC29H   (corrections in)
 *   LC29H -> Serial2 (RX2/GPIO16) -> ESP32 -> USB -> Mac   (NMEA out)
 *
 * Use it with serial_rtk_test.py on the Mac to test injection and reception
 * without the WiFi rover firmware in the picture.
 *
 * Wiring:
 *   LC29H TX  -> ESP32 GPIO16 (RX2)
 *   LC29H RX  -> ESP32 GPIO17 (TX2)
 *   GND       -> GND
 */

const uint32_t GNSS_BAUD = 115200;   // must match the LC29H and the Mac script
const int PIN_RX2 = 16;              // ESP32 <- LC29H TX
const int PIN_TX2 = 17;              // ESP32 -> LC29H RX

void setup() {
  Serial.begin(GNSS_BAUD);   // USB side: keep it the same as the LC29H
  Serial2.begin(GNSS_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
}

void loop() {
  while (Serial.available()) {
    Serial2.write(Serial.read());    // corrections from the Mac -> LC29H
  }
  while (Serial2.available()) {
    Serial.write(Serial2.read());    // NMEA from the LC29H -> Mac
  }
}
