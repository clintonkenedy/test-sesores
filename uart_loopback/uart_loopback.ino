/*
 * UART loopback test - proves the ESP32's TX2 pin actually transmits.
 *
 * Disconnects the LC29H from the question entirely. Wire only ONE jumper:
 *
 *   ESP32 GPIO17 (TX2)  ->  ESP32 GPIO16 (RX2)     (a single wire, on the ESP32)
 *
 * Remove BOTH data wires from the LC29H first, so nothing else drives the pins.
 *
 * The sketch sends "PING #n" out of TX2 every second and prints whatever comes
 * back in on RX2.
 *
 *   PING lines appear  -> TX2 and RX2 both work; the ESP32 side is fine, so the
 *                         fault is the wire to the LC29H RXD or its pin.
 *   nothing appears    -> the jumper is not making contact, or a pin is dead.
 */

const uint32_t BAUD = 115200;
const int PIN_RX2 = 16;   // ESP32 receives
const int PIN_TX2 = 17;   // ESP32 transmits

uint32_t n = 0;
uint32_t lastSend = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
  Serial.println("\nUART loopback test");
  Serial.println("Jumper GPIO17 -> GPIO16. LC29H disconnected.");
  Serial.println("You should see each PING come back.\n");
}

void loop() {
  if (millis() - lastSend >= 1000) {
    lastSend = millis();
    char msg[24];
    int len = snprintf(msg, sizeof(msg), "PING #%lu\n", (unsigned long)++n);
    Serial2.write((uint8_t*)msg, len);
    Serial.print("[sent] ");
    Serial.print(msg);
  }
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.print("[recv] ");
    Serial.write(c);
  }
}
