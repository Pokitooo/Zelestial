#include <Arduino.h>
#include "vt_lora"
#include "UserPins.h"

using namespace vt;
using lora_t = lora_e22;

HardwareSerial SerialLoRa(E22_RX, E22_TX);  // E22 UART

lora_t lora_tx(SerialLoRa, E22_M0, E22_M1);
// lora_t lora_rx(Serial2, PA5, PA6);

void setup() {
    delay(5000);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin();

    SerialLoRa.begin(9600);

    // Begin configuration (sleep) mode.
    lora_t *loras[] = {&lora_tx};
    for (auto &lora: loras) {
        Serial.println("Beginning configuration mode...");
        lora->config();
        Serial.println("Entered configuration mode!");
        lora->set_param(0xffff,
                        0,
                        9600,
                        LoRaParity::PARITY_8N1,
                        1200,
                        63,
                        240,
                        true);

        // query_param() waits forever for a reply; this prints the reply or gives up after 1 s
        const uint8_t cmd_query[] = {0xC1, 0x00, 0x07};
        lora->hw_serial().write(cmd_query, sizeof(cmd_query));

        size_t         n  = 0;
        const uint32_t t0 = millis();
        while (millis() - t0 < 1000) {
            while (lora->hw_serial().available()) {
                const uint8_t b = lora->hw_serial().read();
                if (b < 0x10) Serial.print('0');
                Serial.print(b, HEX);
                Serial.print(' ');
                ++n;
            }
        }
        Serial.println();
        Serial.print("Bytes received from E22: ");
        Serial.println(n);
        if (n == 0)
            Serial.println("E22 did not respond (check baud/clock, TX<->RX, M0/M1, power)");

        Serial.println("Done config!");
    }

    lora_tx.begin(9600);
}

void loop() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Serial.println("Sent!");

    // Test tx
    // lora_tx.hw_serial().println("Message");

    // Test rx
    // while (lora_rx.hw_serial().available()) Serial.write(lora_rx.hw_serial().read());

    delay(1000);
}