#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>
#include "../lib/INA226/INA226.h"

// -------------------- Pin Definitions --------------------
#define FET_CHARGE    4
#define FET_WELD1     5
#define FET_WELD2     6
#define PEDAL_PIN     7

#define I2C_SDA       3
#define I2C_SCL       2
#define THERM_PIN     8   // Thermistor on GPIO8

// -------------------- WiFi / TCP Settings ----------------
const char* ssid     = "Jaime's Wi-Fi Network";
const char* password = "jackaustin";

// TCP server for Pi (ESP32Link expects this)
WiFiServer server(8888);
WiFiClient client;

// -------------------- INA226 -----------------------------
INA226 ina(0x40);      // Vpack
INA226 inaCell1(0x44); // cell 1 sense (node 1)
INA226 inaCell2(0x41); // cell 2 sense (node 2)

// -------------------- Battery monitoring -----------------
float vpack = 0.0;
float current_charge = 0.0;
const float VPACK_SCALE = 0.9977f;        // calibrated so 8.85 → ~8.83
unsigned long last_battery_read = 0;
const unsigned long BATTERY_READ_INTERVAL = 100;  // 100ms

// -------------------- Battery thresholds -----------------
const float CHARGE_LIMIT     = 9.02;
const float CHARGE_RESUME    = 8.70;
const float HARD_LIMIT       = 9.05;
const float MIN_WELD_VOLTAGE = 7.80;

// -------------------- Weld settings ----------------------
uint16_t weld_duration_ms = 50;
unsigned long last_weld_time = 0;
const unsigned long WELD_COOLDOWN = 500;

// -------------------- Thermistor (MicroPython‑matched) ---
float temperature_c    = NAN;
float temp_ema         = NAN;
float temp_last_valid  = NAN;

const float SERIES_RESISTOR      = 10000.0f;
const float THERMISTOR_NOMINAL  = 173000.0f;
const float TEMPERATURE_NOMINAL = 20.0f;    // °C
const float BETA_COEFF          = 3950.0f;

const float TEMP_EMA_ALPHA      = 0.05f;    // smoothing
const float TEMP_OUTLIER_THRESH = 5.0f;     // °C

// Forward declarations
void fireWeld();
void updateBattery();
void updateTemperature();
void controlCharger();
void processCommand(String cmd);
String buildStatus();
void sendToPi(const String &msg);

// -------------------- Calibrated Cell reading helpers ---------------
// Returns true if successful, fills V1/V2/V3 (cumulative) and C1/C2/C3 (per‑cell)
// Calibrated to match your DMM: C1≈2.911, C2≈2.900, C3≈3.022 when the
// uncalibrated readings were C1≈3.103, C2≈2.830, C3≈2.916.
bool readCellsOnce(float &V1, float &V2, float &V3,
                   float &C1, float &C2, float &C3)
{
    // Raw cumulative node voltages from INA226s
    float v_node1 = inaCell1.readVoltage();  // total at node 1
    float v_node2 = inaCell2.readVoltage();  // total at node 2
    float v_pack  = vpack;                   // already sampled pack voltage (scaled)

    if (!isfinite(v_node1) || !isfinite(v_node2) || !isfinite(v_pack)) {
        return false;
    }

    // Raw per‑cell values (same math as MicroPython read_cells_once)
    float c1_raw = v_node1;
    float c2_raw = v_node2 - v_node1;
    float c3_raw = v_pack  - v_node2;

    // ---- Per‑cell calibration offsets (tuned from your DMM) ----
    const float C1_OFFSET = -0.192f;
    const float C2_OFFSET = +0.070f;
    const float C3_OFFSET = +0.106f;

    C1 = c1_raw + C1_OFFSET;
    C2 = c2_raw + C2_OFFSET;
    C3 = c3_raw + C3_OFFSET;

    // Rebuild cumulative voltages from calibrated cells
    V1 = C1;
    V2 = C1 + C2;
    V3 = C1 + C2 + C3;

    return true;
}

// -------------------- Thermistor helpers -----------------
float readThermistorOnce() {
    int raw = analogRead(THERM_PIN);   // 0..4095 (12‑bit)

    if (raw <= 0)    raw = 1;
    if (raw >= 4095) raw = 4094;

    // Convert ADC reading to voltage
    float v = 3.3f * ((float)raw / 4095.0f);
    // R_therm = R_series * Vout / (3.3 - Vout)
    float r_therm = SERIES_RESISTOR * (v / (3.3f - v));

    // Beta formula (Steinhart)
    float steinhart = r_therm / THERMISTOR_NOMINAL;          // (R/R0)
    steinhart = logf(steinhart);                             // ln(R/R0)
    steinhart /= BETA_COEFF;                                 // 1/B * ln(R/R0)
    steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f);     // + 1/T0
    steinhart = 1.0f / steinhart;                            // invert
    steinhart -= 273.15f;                                    // K → °C

    return steinhart;
}

void updateTemperature() {
    float t_raw = readThermistorOnce();
    if (!isfinite(t_raw)) return;

    // First valid sample
    if (!isfinite(temp_last_valid)) {
        temp_last_valid = t_raw;
        temp_ema        = t_raw;
        temperature_c   = t_raw;
        return;
    }

    // Outlier rejection
    float diff = fabsf(t_raw - temp_last_valid);
    if (diff > TEMP_OUTLIER_THRESH) {
        return;  // ignore this sample
    }

    temp_last_valid = t_raw;

    // EMA smoothing
    if (!isfinite(temp_ema)) {
        temp_ema = t_raw;
    } else {
        temp_ema = TEMP_EMA_ALPHA * t_raw +
                   (1.0f - TEMP_EMA_ALPHA) * temp_ema;
    }

    // Optional final offset if you ever want it:
    const float TEMP_OFFSET = 0.0f;   // change later if needed
    temperature_c = temp_ema + TEMP_OFFSET;
}

// -------------------- Helper: Build STATUS line ----------
String buildStatus() {
    bool charge_on = digitalRead(FET_CHARGE);
    String state = charge_on ? "ON" : "OFF";

    // Temperature string
    String t_str;
    if (isfinite(temperature_c)) {
        t_str = String(temperature_c, 1);   // 1 decimal
    } else {
        t_str = "NaN";
    }

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
    float raw_vpack = ina.readVoltage();
    vpack = raw_vpack * VPACK_SCALE;    // apply pack calibration
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
    if (millis() - last_weld_time < WELD_COOLDOWN) {
        Serial.println("⚠️ Cooldown active");
        return;
    }

    if (vpack < MIN_WELD_VOLTAGE) {
        Serial.printf("⚠️ Voltage too low: %.2fV (min %.2fV)\n", vpack, MIN_WELD_VOLTAGE);
        return;
    }

    Serial.printf("🔥 FIRING %d ms weld! Vpack=%.2fV\n", weld_duration_ms, vpack);

    digitalWrite(FET_CHARGE, LOW);
    delay(5);

    digitalWrite(FET_WELD1, HIGH);
    digitalWrite(FET_WELD2, HIGH);

    unsigned long start = millis();
    delay(weld_duration_ms);
    unsigned long actual_duration = millis() - start;

    digitalWrite(FET_WELD1, LOW);
    digitalWrite(FET_WELD2, LOW);

    delay(10);
    controlCharger();
    last_weld_time = millis();

    Serial.printf("✅ Weld complete! Duration: %lu ms\n", actual_duration);

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
        Serial.println("⚠️ FIRE command ignored (pedal‑only mode)");
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

    Serial.println();
    Serial.println("*****************************");
    Serial.println("***  SETUP() HAS STARTED  ***");
    Serial.println("*****************************");
    Serial.flush();

    Serial.println("\n=== ESP32 Weld Controller v2.4 - TCP LINK ===");

    pinMode(FET_CHARGE, OUTPUT);
    pinMode(FET_WELD1, OUTPUT);
    pinMode(FET_WELD2, OUTPUT);
    pinMode(PEDAL_PIN, INPUT_PULLUP);
    pinMode(THERM_PIN, INPUT);

    digitalWrite(FET_CHARGE, LOW);
    digitalWrite(FET_WELD1, LOW);
    digitalWrite(FET_WELD2, LOW);

    analogReadResolution(12);
    // analogSetPinAttenuation(THERM_PIN, ADC_11db); // optional

    Wire.begin(I2C_SDA, I2C_SCL);
    if (ina.begin(&Wire)) {
        Serial.println("✅ INA226 (pack) initialized");
        Serial.printf("   Calibration: 0x%04X\n", ina.getCalibration());

        if (inaCell1.begin(&Wire)) {
            Serial.println("✅ INA226 cell1 (0x44) initialized");
        } else {
            Serial.println("⚠️ INA226 cell1 (0x44) init failed");
        }
        if (inaCell2.begin(&Wire)) {
            Serial.println("✅ INA226 cell2 (0x41) initialized");
        } else {
            Serial.println("⚠️ INA226 cell2 (0x41) init failed");
        }

        updateBattery();
        updateTemperature();
        Serial.printf("   Initial Vpack: %.2fV\n", vpack);
        Serial.printf("   Initial Current: %.2fA\n", current_charge);
        if (isfinite(temperature_c)) {
            Serial.printf("   Initial Temp: %.1fC\n", temperature_c);
        }

        controlCharger();
    } else {
        Serial.println("⚠️ INA226 init failed - charging disabled");
    }

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
        if (!client || !client.connected()) {
            WiFiClient newClient = server.available();
            if (newClient) {
                client = newClient;
                Serial.println("[TCP] Client connected from " +
                               client.remoteIP().toString());
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

    // -------- Battery readings, temp & charger control ----
    if (millis() - last_battery_read >= BATTERY_READ_INTERVAL) {
        last_battery_read = millis();
        updateBattery();
        updateTemperature();
        controlCharger();

        static unsigned long last_print = 0;
        if (millis() - last_print >= 2000) {
            last_print = millis();
            bool charging = digitalRead(FET_CHARGE);
            Serial.printf("📊 Vpack=%.2fV I=%.2fA %s",
                          vpack, current_charge,
                          charging ? "⚡CHARGING" : "⏸️IDLE");
            if (isfinite(temperature_c)) {
                Serial.printf("  Temp=%.1fC\n", temperature_c);
            } else {
                Serial.println();
            }

            String status = buildStatus();
            sendToPi(status);

            float V1, V2, V3, C1, C2, C3;
            if (readCellsOnce(V1, V2, V3, C1, C2, C3)) {
                // Swap C1 and C3 in the *labels* so UI matches physical layout
                float uiC1 = C3;  // top cell
                float uiC2 = C2;  // middle
                float uiC3 = C1;  // bottom

                char buf[160];
                snprintf(buf, sizeof(buf),
                         "CELLS,V1=%.3f,V2=%.3f,V3=%.3f,C1=%.3f,C2=%.3f,C3=%.3f",
                         V1, V2, V3, uiC1, uiC2, uiC3);
                sendToPi(String(buf));
            } else {
                sendToPi("CELLS,NONE");
            }
        }
    }

    // -------- Pedal handling (active‑low, debounced) ------
    static int pedal_last_raw = HIGH;
    static int pedal_stable   = HIGH;
    static unsigned long pedal_last_change_ms = 0;
    const unsigned long PEDAL_DEBOUNCE_MS = 40;

    int pedal_raw = digitalRead(PEDAL_PIN);
    unsigned long now = millis();

    if (pedal_raw != pedal_last_raw) {
        pedal_last_change_ms = now;
        pedal_last_raw = pedal_raw;
    }

    if ((now - pedal_last_change_ms) >= PEDAL_DEBOUNCE_MS) {
        if (pedal_raw != pedal_stable) {
            int prev_stable = pedal_stable;
            pedal_stable = pedal_raw;

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