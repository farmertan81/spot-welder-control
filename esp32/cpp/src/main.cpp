#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "../lib/INA226/INA226.h"

// -------------------- Pin Definitions --------------------
#define FET_CHARGE    4
#define FET_WELD1     5
#define FET_WELD2     6
#define PEDAL_PIN     7

#define I2C_SDA       3
#define I2C_SCL       2

// -------------------- WiFi / TCP Settings ----------------
const char* ssid     = "Jaime's Wi-Fi Network";
const char* password = "jackaustin";

// TCP server for Pi (ESP32Link expects this)
WiFiServer server(8888);
WiFiClient client;

// -------------------- INA226 -----------------------------
INA226 ina(0x40);

// -------------------- Battery thresholds -----------------
const float CHARGE_LIMIT     = 9.02;
const float CHARGE_RESUME    = 8.70;
const float HARD_LIMIT       = 9.05;
const float MIN_WELD_VOLTAGE = 7.80;

// -------------------- Weld settings ----------------------
uint16_t weld_duration_ms = 50;
unsigned long last_weld_time = 0;
const unsigned long WELD_COOLDOWN = 500;

// -------------------- Battery monitoring -----------------
float vpack = 0.0;
float current_charge = 0.0;
unsigned long last_battery_read = 0;
const unsigned long BATTERY_READ_INTERVAL = 100;  // 100ms

// Forward declarations
void fireWeld();
void updateBattery();
void controlCharger();
void processCommand(String cmd);
String buildStatus();
void sendToPi(const String &msg);

// -------------------- Helper: Build STATUS line ----------
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

// -------------------- Helper: Send line to Pi ------------
void sendToPi(const String &msg) {
    if (client && client.connected()) {
        client.println(msg);   // newline‑terminated
        Serial.printf("[TCP] TX: %s\n", msg.c_str());
    } else {
        Serial.printf("[TCP] Not connected, drop: %s\n", msg.c_str());
    }
}

// -------------------- Battery / Charger ------------------
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

// -------------------- Weld Logic -------------------------
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
    
    // Send weld result back to Pi
    String response = "FIRED," + String(actual_duration);
    sendToPi(response);
}

// -------------------- Command Parser ---------------------
void processCommand(String cmd) {
    Serial.printf("[CMD] Processing: %s\n", cmd.c_str());
    
    if (cmd.startsWith("SET_PULSE,")) {
        int duration = cmd.substring(10).toInt();
        if (duration > 0 && duration <= 200) {
            weld_duration_ms = duration;
            Serial.printf("✅ Weld duration set to %d ms\n", weld_duration_ms);
            sendToPi("ACK:SET_PULSE," + String(weld_duration_ms));
        }
    }
    else if (cmd == "FIRE") {
        // Pi should not use this in pedal‑only mode, but we keep for compatibility
        Serial.println("⚠️ FIRE command ignored (pedal-only mode)");
    }
    else if (cmd == "CHARGE_ON") {
        digitalWrite(FET_CHARGE, HIGH);
        Serial.println("✅ Charging ON (manual)");
        sendToPi("ACK:CHARGE_ON");
    }
    else if (cmd == "CHARGE_OFF") {
        digitalWrite(FET_CHARGE, LOW);
        Serial.println("✅ Charging OFF (manual)");
        sendToPi("ACK:CHARGE_OFF");
    }
    else if (cmd == "STATUS") {
        String status = buildStatus();
        sendToPi(status);
    }
}

// -------------------- Setup ------------------------------
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== ESP32 Weld Controller v2.4 - TCP LINK ===");

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

    // ---------------- WiFi ----------------
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

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

        Serial.println("Starting TCP server on port 8888...");
        server.begin();
        server.setNoDelay(true);
    } else {
        Serial.println("\n❌ WiFi FAILED!");
    }
}

// -------------------- Loop -------------------------------
void loop() {
    // -------- TCP server: accept & read commands ----------
    if (WiFi.status() == WL_CONNECTED) {
        // Accept a new client if none / disconnected
        if (!client || !client.connected()) {
            WiFiClient newClient = server.available();
            if (newClient) {
                client = newClient;
                Serial.println("[TCP] Client connected from " +
                               client.remoteIP().toString());
                // Optional hello
                // sendToPi("HELLO,ESP32");
            }
        } else if (client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) {
                Serial.printf("[TCP] RX: %s\n", line.c_str());
                processCommand(line);
            }
        }
    }
    
    // -------- Battery readings & charger control ----------
    if (millis() - last_battery_read >= BATTERY_READ_INTERVAL) {
        last_battery_read = millis();
        updateBattery();
        controlCharger();
        
        // Print & send status every 2 seconds
        static unsigned long last_print = 0;
        if (millis() - last_print >= 2000) {
            last_print = millis();
            bool charging = digitalRead(FET_CHARGE);
            Serial.printf("📊 Vpack=%.2fV I=%.2fA %s\n", 
                         vpack, current_charge, 
                         charging ? "⚡CHARGING" : "⏸️IDLE");

            String status = buildStatus();
            sendToPi(status);
        }
    }
    
    // -------- Pedal handling (active-low, debounced) ------
    static int pedal_last_raw = HIGH;            // last raw read
    static int pedal_stable   = HIGH;            // last debounced stable state
    static unsigned long pedal_last_change_ms = 0;
    const unsigned long PEDAL_DEBOUNCE_MS = 40;  // debounce

    int pedal_raw = digitalRead(PEDAL_PIN);
    unsigned long now = millis();

    // Track changes in the raw signal
    if (pedal_raw != pedal_last_raw) {
        pedal_last_change_ms = now;
        pedal_last_raw = pedal_raw;
    }

    // Accept a new stable state only if held long enough
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
        }
    }

    delay(10);
}
