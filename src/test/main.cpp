#include <Arduino.h>
#include <SPI.h>
#include <vt_lora>          // EBYTE LoRa
#include <Wire.h>

#define PIN_M0 PA5
#define PIN_M1 PA6

TwoWire        Wire1(PB9_ALT1, PB10);
HardwareSerial MySerial(PA_3, PA_2);
using lora_t = vt::lora_e22;
lora_t lora_tx(MySerial, PIN_M0, PIN_M1);
bool   loraReady = false;

unsigned long lastTxTime = 0, timeOnAirMs = 80000;

void blinkLED(int times = 1, int delayMs = 300, int pin = PC13) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, pin == PC13 ? LOW : HIGH);
    delay(delayMs);
    digitalWrite(pin, pin == PC13 ? HIGH : LOW);
    delay(delayMs);
  }
}


bool sendLoRa() {
  if (!loraReady) {
    Serial.println("LoRa configuration was not verified; transmission skipped.");
    return false;
  }
  if (millis() - lastTxTime < timeOnAirMs) {
    Serial.println("Waiting for previous transmission to complete");
    return false;
  }
  Serial.println("LoRa is sending");
  return true;
}
// VT is a configurator; transparent data is written to its hardware UART.
void payload_send(const char *message) {
  Serial.print("Payload: ");
  Serial.println(message);
  const size_t length  = strlen(message);
  const size_t written = lora_tx.hw_serial().write(
    reinterpret_cast<const uint8_t *>(message), length);
  lora_tx.hw_serial().flush();
  if (written == length) Serial.println("LoRa payload written to UART.");
  else
    Serial.println("LoRa UART write was incomplete.");
  // UART write completion is not a receiver acknowledgement.
}

// New compatibility helper. Upstream query_param() has an unbounded wait.
bool readLoRaRegisters(uint8_t (&reply)[12]) {
  HardwareSerial &port = lora_tx.hw_serial();
  while (port.available()) port.read();  // Discard configuration-write echoes.
  const uint8_t query[] = {0xC1, 0x00, 0x09};
  if (port.write(query, sizeof(query)) != sizeof(query)) return false;
  port.flush();
  const unsigned long started  = millis();
  size_t              received = 0;
  while (received < sizeof(reply) && millis() - started < 2000UL) {
    if (port.available()) reply[received++] = static_cast<uint8_t>(port.read());
    else
      delay(1);
  }
  return received == sizeof(reply) && reply[0] == 0xC1 &&
         reply[1] == 0x00 && reply[2] == 0x09;
}

void printParameters(const vt::lora_e22_param &c) {
  Serial.print("Channel: ");
  Serial.println(c.reg2);
  Serial.print("ADDH: ");
  Serial.println(c.addh, HEX);
  Serial.print("ADDL: ");
  Serial.println(c.addl, HEX);
}


void lora_configuration() {
  lora_tx.config();  // E22 configuration mode: M0 LOW, M1 HIGH, 9600 baud.
  // Native VT arguments corresponding to the recovered July 7 field block.
  // 300 is bits/sec (0.3 kbps); keep this distinct from the binary's 1200.
  lora_tx.set_param(0x0003, 0, 9600, vt::LoRaParity::PARITY_8N1,
                    300, 23, 240, true);
  lora_tx.hw_serial().flush();
  delay(500);

  // VT exposes no setter for REG3. Preserve the historical WORPeriod=011
  // through a one-register E22 configuration write. Other REG3 flags are zero:
  // transparent mode, RSSI/repeater/LBT disabled, WOR receiver.
  const uint8_t reg3Write[] = {0xC0, 0x06, 0x01, 0x03};
  lora_tx.hw_serial().write(reg3Write, sizeof(reg3Write));
  lora_tx.hw_serial().flush();
  delay(500);

  vt::lora_e22_param requested = lora_tx.get_param();
  requested.reg3               = 0x03;
  const uint8_t expected[]     = {requested.addh, requested.addl, requested.netid,
                                  requested.reg0, requested.reg1, requested.reg2, requested.reg3,
                                  requested.crypt_h, requested.crypt_l};
  uint8_t       reply[12]      = {};
  loraReady                    = readLoRaRegisters(reply) &&
                                 memcmp(reply + 3, expected, sizeof(expected)) == 0;
  Serial.println(loraReady ? "LoRa configuration verified." : "LoRa configuration verification failed or timed out.");
  printParameters(requested);
  lora_tx.begin(9600);  // Normal transparent UART operation, M0/M1 LOW.
}

// Queue consumer reconstructed from payload_send and the matching LoRaTask.

// void blinkLED(int times = 1, int delayMs = 300, int pin = PC13) {
//   for (int i = 0; i < times; i++) {
//     digitalWrite(pin, pin == PC13 ? LOW : HIGH);
//     delay(delayMs);
//     digitalWrite(pin, pin == PC13 ? HIGH : LOW);
//     delay(delayMs);
//   }
// }


void setup() {
  pinMode(PC13, OUTPUT);
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);

  MySerial.begin(9600);
  Serial.begin(115200);

  // blinkLED(1);

  lora_configuration();

  // blinkLED(1);

  Serial.println("LoRa ready.");
}

void loop() {
    if (sendLoRa()) {
        payload_send("Hello from LoRa");
        lastTxTime = millis();
    }

    delay(1000);
}