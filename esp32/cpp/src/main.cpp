#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "../lib/INA226/INA226.h"

#define FET_CHARGE    4
#define FET_WELD1     5
#define FET_WELD2     6
#define PEDAL_PIN     7

#define I2C_SDA       3
#define I2C_SCL       2

const char* ssid = "Jaime's Wi-Fi Network";
const char* password = "jackaustin";
const char* ws_host = "192.168.68.65";
const uint16_t ws_port = 8080;

WebSocketsClient webSocket;
INA226 ina(0x40);

// Battery thresholds
const float CHARGE_LIMIT = 9.02;
const float CHARGE_RESUME = 8.70;
const float HARD_LIMIT = 9.05;
const float MIN_WELD_VOLTAGE = 7.80;

// Weld settings
uint16_t weld_duration_ms = 50;
bool pedal_was_pressed = false;
unsigned long last_weld_time = 0;
const unsigned long WELD_COOLDOWN = 500;

// Battery monitoring
float vpack = 0.0;
float current_charge = 0.0;
unsigned long last_battery_read = 0;
const unsigned long BATTERY_READ_INTERVAL = 100;  // 100ms

// Forward declarations
void fireWeld();
String buildStatus() {
    bool charge_on = digitalRead(FET_CHARGE);
    String state = charge_on ? "ON" : "OFF";

    // For now, temp is NaN (no thermistor implemented yet)
    String t_str = "NaN";

    unsigned long now = millis();
    long cooldown_ms = (long)(WELD_COOLDOWN - (now - last_weld_time));
    if (cooldown_ms < 0) cooldown_ms = 0;

    String status = "STATUS";
    status += ",enabled=1";
    status += ",state=" + state;
    status += ",vpack=" + String(vpack, 3);
    status += ",i=" + String(current_charge, 3);
    status += ",temp=" + t_str;
    status += ",cooldown_ms=" + String(cooldown_ms);
    status += ",pulse_ms=" + String(weld_duration_ms);

    return status;
}

void updateBattery();
void controlCharger();

void updateBattery() {
    vpack = ina.readVoltage();
    current_charge = ina.readCurrent();
}

void controlCharger() {
    static int high_count = 0;
    
    if (vpack >= HARD_LIMIT) {
        high_count++;
        if (high_count >= 2) {
            digitalWrite(FET_CHARGE, LOW);
            Serial.println("⚠️ HARD LIMIT - Charging OFF");
            return;
        }
    } else {
        high_count = 0;
    }
    
    if (vpack >= CHARGE_LIMIT) {
        digitalWrite(FET_CHARGE, LOW);
    } else if (vpack < CHARGE_RESUME) {
        digitalWrite(FET_CHARGE, HIGH);
    }
}

void fireWeld() {
    // Check cooldown
    if (millis() - last_weld_time < WELD_COOLDOWN) {
        Serial.println("⚠️ Cooldown active");
        return;
    }
    
    // Check voltage
    if (vpack < MIN_WELD_VOLTAGE) {
        Serial.printf("⚠️ Voltage too low: %.2fV (min %.2fV)\n", vpack, MIN_WELD_VOLTAGE);
        return;
    }
    
    Serial.printf("🔥 FIRING %d ms weld! Vpack=%.2fV\n", weld_duration_ms, vpack);
    
    // Disable charge FET
    digitalWrite(FET_CHARGE, LOW);
    delay(5);
    
    // Fire weld
    digitalWrite(FET_WELD1, HIGH);
    digitalWrite(FET_WELD2, HIGH);
    
    unsigned long start = millis();
    delay(weld_duration_ms);
    unsigned long actual_duration = millis() - start;
    
    // Turn off
    digitalWrite(FET_WELD1, LOW);
    digitalWrite(FET_WELD2, LOW);
    
    delay(10);
    
    // Re-enable charge (will be controlled by voltage limits)
    controlCharger();
    
    last_weld_time = millis();
    
    Serial.printf("✅ Weld complete! Duration: %lu ms\n", actual_duration);
    
    // Send weld data back
    String response = "FIRED," + String(actual_duration);
    webSocket.sendTXT(response);
}

void processCommand(String cmd) {
    Serial.printf("[CMD] Processing: %s\n", cmd.c_str());
    
    if (cmd.startsWith("SET_PULSE,")) {
        int duration = cmd.substring(10).toInt();
        if (duration > 0 && duration <= 200) {
            weld_duration_ms = duration;
            Serial.printf("✅ Weld duration set to %d ms\n", weld_duration_ms);
            webSocket.sendTXT("ACK:SET_PULSE," + String(weld_duration_ms));
        }
    }
    else if (cmd == "FIRE") {
        Serial.println("⚠️ FIRE command ignored (pedal-only mode)");
    }
    else if (cmd == "CHARGE_ON") {
        digitalWrite(FET_CHARGE, HIGH);
        Serial.println("✅ Charging ON (manual)");
        webSocket.sendTXT("ACK:CHARGE_ON");
    }
    else if (cmd == "CHARGE_OFF") {
        digitalWrite(FET_CHARGE, LOW);
        Serial.println("✅ Charging OFF (manual)");
        webSocket.sendTXT("ACK:CHARGE_OFF");
    }
    else if (cmd == "STATUS") {
        String status = buildStatus();
        webSocket.sendTXT(status);
    }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("[WS] Disconnected");
            break;
        case WStype_CONNECTED:
            Serial.println("[WS] Connected!");
            webSocket.sendTXT("HELLO,ESP32");
            break;
        case WStype_TEXT:
            Serial.printf("[WS] RX: %s\n", payload);
            processCommand(String((char*)payload));
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== ESP32 Weld Controller v2.3 - BATTERY MONITOR ===");

    pinMode(FET_CHARGE, OUTPUT);
    pinMode(FET_WELD1, OUTPUT);
    pinMode(FET_WELD2, OUTPUT);
    pinMode(PEDAL_PIN, INPUT_PULLUP);
    
    digitalWrite(FET_CHARGE, LOW);  // Start with charging OFF
    digitalWrite(FET_WELD1, LOW);
    digitalWrite(FET_WELD2, LOW);

    // Initialize I2C and INA226
    Wire.begin(I2C_SDA, I2C_SCL);
    if (ina.begin(&Wire)) {
        Serial.println("✅ INA226 initialized");
        Serial.printf("   Calibration: 0x%04X\n", ina.getCalibration());
        
        // Read initial voltage
        updateBattery();
        Serial.printf("   Initial Vpack: %.2fV\n", vpack);
        Serial.printf("   Initial Current: %.2fA\n", current_charge);
        
        // Enable charging based on voltage
        controlCharger();
    } else {
        Serial.println("⚠️ INA226 init failed - charging disabled");
    }

    Serial.println("🦶 Pedal mode: ALWAYS ARMED");

    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    Serial.print("Connecting to: ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;

        if (attempts % 10 == 0) {
            Serial.print(" [");
            Serial.print((int)WiFi.status());
            Serial.print("] ");
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi CONNECTED!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        Serial.printf("Connecting WebSocket to %s:%u/ws\n", ws_host, ws_port);
        webSocket.begin(ws_host, ws_port, "/ws");
        webSocket.onEvent(webSocketEvent);
        webSocket.setReconnectInterval(5000);
    } else {
        Serial.println("\n❌ WiFi FAILED!");
    }
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        webSocket.loop();
    }
    
    // Update battery readings periodically
    if (millis() - last_battery_read >= BATTERY_READ_INTERVAL) {
        last_battery_read = millis();
        updateBattery();
        controlCharger();
        
        // Print status every 2 seconds
        static unsigned long last_print = 0;
        if (millis() - last_print >= 2000) {
            last_print = millis();
            bool charging = digitalRead(FET_CHARGE);
            Serial.printf("📊 Vpack=%.2fV I=%.2fA %s\n", 
                         vpack, current_charge, 
                         charging ? "⚡CHARGING" : "⏸️IDLE");

            // Send status to server via WebSocket
            String status = buildStatus();
            webSocket.sendTXT(status);
        }
    }
    
    // Pedal handling (active-low, stronger debounce, one weld per press)
    static int pedal_last_raw = HIGH;            // last raw read
    static int pedal_stable = HIGH;             // last debounced stable state
    static unsigned long pedal_last_change_ms = 0;
    const unsigned long PEDAL_DEBOUNCE_MS = 40; // slightly longer debounce

    int pedal_raw = digitalRead(PEDAL_PIN);
    unsigned long now = millis();

    // Track changes in the raw signal
    if (pedal_raw != pedal_last_raw) {
        pedal_last_change_ms = now;
        pedal_last_raw = pedal_raw;
    }

    // Accept a new stable state only if it's held for PEDAL_DEBOUNCE_MS
    if ((now - pedal_last_change_ms) >= PEDAL_DEBOUNCE_MS) {
        if (pedal_raw != pedal_stable) {
            int prev_stable = pedal_stable;
            pedal_stable = pedal_raw;

            // Active‑low: HIGH -> LOW is a clean "press"
            if (prev_stable == HIGH && pedal_stable == LOW) {
                if (now - last_weld_time >= WELD_COOLDOWN) {
                    Serial.println("🦶 Pedal pressed -> trigger weld");
                    fireWeld();
                } else {
                    Serial.println("🦶 Pedal pressed but cooldown active");
                }
            }

            // LOW -> HIGH (release) just arms us for next press;
            // we don't need to do anything special here.
        }
    }

    delay(10);

}
