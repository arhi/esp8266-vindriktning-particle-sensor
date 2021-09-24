#pragma once

#include <SoftwareSerial.h>

#include "Types.h"

namespace SerialCom {
    constexpr static const uint8_t PIN_UART_RX =  4; // D2 on Wemos D1 Mini
    constexpr static const uint8_t PIN_UART_TX = 13; // UNUSED

    SoftwareSerial sensorSerial(PIN_UART_RX, PIN_UART_TX);

    uint8_t serialRxBuf[255];
    uint8_t rxBufIdx = 0;

    void setup() {
        sensorSerial.begin(9600);
    }

    void clearRxBuf() {
        // Clear everything for the next message
        memset(serialRxBuf, 0, sizeof(serialRxBuf));
        rxBufIdx = 0;
    }

    void parseState(particleSensorState_t& state) {
        /**
         *         MSB  DF 3     DF 4  LSB
         * uint16_t = xxxxxxxx xxxxxxxx
         */
        const uint16_t pm25 = (serialRxBuf[5] << 8)        | serialRxBuf[6];   // DF3*256 + DF4
        const uint16_t pm01 = ((serialRxBuf[9] << 8) ^ 1)  | serialRxBuf[10];  // DF7*256 ^ 1 + DF8
        const uint16_t pm10 = ((serialRxBuf[13] << 8) ^ 1) | serialRxBuf[14];  // DF11*256 ^ 1 + DF12

        Serial.printf("Received PM2.5 reading: %d\r\n", pm25);
        Serial.printf("Received PM1.0 reading: %d\r\n", pm01);
        Serial.printf("Received PM10 reading: %d\r\n", pm10);

        state.measurements[state.measurementIdx] = pm25;
        state.avgPM01 = (state.avgPM01 + pm01 ) / 2;
        state.avgPM10 = (state.avgPM10 + pm10 ) / 2;

        state.measurementIdx = (state.measurementIdx + 1) % 5;

        if (state.measurementIdx == 0) {
            float avgPM25 = 0.0f;

            for (uint8_t i = 0; i < 5; ++i) {
                avgPM25 += state.measurements[i] / 5.0f;
            }

            state.avgPM25 = avgPM25;
            state.valid = true;

            Serial.printf("New Avg PM25: %d\r\n", state.avgPM25);
        }

        clearRxBuf();
    }

    bool isValidHeader() {
        bool headerValid = serialRxBuf[0] == 0x16 && serialRxBuf[1] == 0x11 && serialRxBuf[2] == 0x0B;

        if (!headerValid) {
            Serial.print("Received message with invalid header.\r\n");
        }

        return headerValid;
    }

    bool isValidChecksum() {
        uint8_t checksum = 0;

        for (uint8_t i = 0; i < 20; i++) {
            checksum += serialRxBuf[i];
        }

        if (checksum != 0) {
            Serial.printf("Received message with invalid checksum. Expected: 0. Actual: %d\r\n", checksum);
        }

        return checksum == 0;
    }

    void handleUart(particleSensorState_t& state) {
        if (!sensorSerial.available()) {
            return;
        }

        Serial.print("Receiving:");
        while (sensorSerial.available()) {
            serialRxBuf[rxBufIdx++] = sensorSerial.read();
            Serial.print(".");

            // Without this delay, receiving data breaks for reasons that are beyond me
            delay(15);

            if (rxBufIdx >= 64) {
                clearRxBuf();
            }
        }
        Serial.print("Done.\r\n");

        if (isValidHeader() && isValidChecksum()) {
            parseState(state);

            Serial.printf(
                "Current measurements: %d, %d, %d, %d, %d\r\n",

                state.measurements[0],
                state.measurements[1],
                state.measurements[2],
                state.measurements[3],
                state.measurements[4]
            );
        } else {
            clearRxBuf();
        }
    }
} // namespace SerialCom
