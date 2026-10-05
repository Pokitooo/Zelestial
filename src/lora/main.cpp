#include <Arduino.h>
#include "vt_lora"
#include "UserPins.h"

using namespace vt;
using lora_t = lora_e22;

HardwareSerial SerialLoRa(E22_RX, E22_TX);  // E22 UART

lora_t lora_tx(SerialLoRa, E22_M0, E22_M1);

// UART baud actually used on the STM32 side. Normally 9600, but if HSE_VALUE does not
// match the real crystal, every baud rate is scaled wrong and we need to compensate.
uint32_t lora_uart_baud = 0;

static void print_hex(const uint8_t *buf, const size_t n) {
  for (size_t i = 0; i < n; ++i) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// Read up to `len` bytes, giving up after `timeout_ms`. Returns bytes read.
static size_t read_reply(uint8_t *buf, const size_t len, const uint32_t timeout_ms) {
  size_t         n  = 0;
  const uint32_t t0 = millis();
  while (n < len && millis() - t0 < timeout_ms) {
    if (SerialLoRa.available()) buf[n++] = static_cast<uint8_t>(SerialLoRa.read());
  }
  return n;
}

// Module must already be in config mode (M1=1, M0=0). Reads registers 0x00..0x08.
// Unlike lora_e22::query_param(), this never blocks forever.
static bool query_registers(const uint32_t baud, uint8_t (&reply)[12]) {
  SerialLoRa.end();
  SerialLoRa.begin(baud);
  delay(50);
  while (SerialLoRa.available()) SerialLoRa.read();

  const uint8_t cmd[] = {0xC1, 0x00, 0x09};
  SerialLoRa.write(cmd, sizeof(cmd));
  SerialLoRa.flush();

  const size_t n = read_reply(reply, sizeof(reply), 500);
  Serial.print("  @");
  Serial.print(baud);
  Serial.print(" baud -> ");
  Serial.print(n);
  Serial.print(" bytes: ");
  print_hex(reply, n);
  return n == sizeof(reply) && reply[0] == 0xC1 && reply[1] == 0x00 && reply[2] == 0x09;
}

void setup() {
  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin();

  Serial.println("=== E22 LoRa test ===");
  Serial.print("SystemCoreClock (as HAL believes): ");
  Serial.println(SystemCoreClock);
  Serial.print("PCLK1 (USART2 clock, as HAL believes): ");
  Serial.println(HAL_RCC_GetPCLK1Freq());
  Serial.print("HSE_VALUE: ");
  Serial.println(static_cast<uint32_t>(HSE_VALUE));

  SerialLoRa.begin(E22_BAUDRATE);

  Serial.println("Entering configuration mode (M1=1, M0=0)...");
  lora_tx.config();
  delay(100);  // E22 needs time after a mode switch (AUX goes high)

  // 1) Normal case: UART baud matches what the E22 expects in config mode (9600).
  // 2) If HSE_VALUE says 25 MHz but the crystal is really 24 MHz, the UART runs 4% slow.
  //    Asking for 10000 then produces a real 9600 on the wire.
  uint8_t        reply[12] = {};
  const uint32_t candidates[] = {9600, 10000};
  for (const uint32_t baud: candidates) {
    if (query_registers(baud, reply)) {
      lora_uart_baud = baud;
      break;
    }
  }

  if (lora_uart_baud == 0) {
    Serial.println("E22 did not respond at either baud.");
    Serial.println("Check: E22 TXD -> PA3, E22 RXD -> PA2, M0 -> PA5, M1 -> PA6, common GND, 3.3-5V supply.");
    return;
  }

  if (lora_uart_baud != 9600) {
    Serial.println("!!! E22 only answered at a compensated baud rate.");
    Serial.println("!!! The real crystal does not match HSE_VALUE: all UART/timing is ~4% off.");
    Serial.println("!!! Fix HSE_VALUE in platformio.ini and the PLL in src/clock.c.");
  }

  Serial.println("Current registers OK. Writing new parameters...");
  while (SerialLoRa.available()) SerialLoRa.read();
  lora_tx.set_param(E22_ADDRESS,
                    0,
                    E22_BAUDRATE,
                    LoRaParity::PARITY_8N1,
                    1200,
                    E22_CHANNEL,
                    240,
                    true);

  // E22 acknowledges a C0 write with C1 + the written registers.
  const size_t n = read_reply(reply, sizeof(reply), 500);
  Serial.print("Write ack: ");
  print_hex(reply, n);
  Serial.println(n == sizeof(reply) && reply[0] == 0xC1 ? "Config written!" : "Config write NOT acknowledged");

  query_registers(lora_uart_baud, reply);

  lora_tx.begin(lora_uart_baud);  // back to normal (transparent) mode
  Serial.println("Normal mode. Sending a test message every second.");
}

void loop() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  if (lora_uart_baud != 0) {
    lora_tx.hw_serial().println("Message");
    while (lora_tx.hw_serial().available()) Serial.write(lora_tx.hw_serial().read());
  }

  delay(1000);
}
