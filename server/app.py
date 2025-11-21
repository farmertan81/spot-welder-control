#!/usr/bin/env python3

# Weld cooldown protection
WELD_COOLDOWN_MS = 3000  # 3 seconds minimum between welds
last_weld_time = 0
# Circuit resistance
TOTAL_RESISTANCE_OHM = 0.00296  # 2.96 mΩ total circuit resistance


"""
Spot Welder Control Server
Flask + SocketIO + ESP32 + ADS1256 + Weld Capture
"""

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import threading
from collections import deque
import time
import json
import os
from datetime import datetime
from collections import deque
import math

# Import our drivers
from esp_link import ESP32Link
# ADS1256 now handled by ESP32, not Pi

app = Flask(__name__, 
            template_folder='../webui/templates',
            static_folder='../webui/static')
app.config['SECRET_KEY'] = 'weldctl_secret_2025'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Directories
WELD_HISTORY_DIR = "../weld_history"
SETTINGS_FILE = "settings.json"
PRESETS_FILE = "presets.json"
os.makedirs(WELD_HISTORY_DIR, exist_ok=True)

# Global state
esp_status = {
    "vpack": 0.0,
    "i": 0.0,
    "pulse_ms": 50,
    "state": "IDLE",
    "enabled": False,
    "cooldown_ms": 0
}

cells_status = {
    "V1": 0.0, "V2": 0.0, "V3": 0.0,
    "C1": 0.0, "C2": 0.0, "C3": 0.0
}

temperature = None
pedal_active = False
weld_counter = 0
# Pre-trigger buffer
PRE_TRIGGER_MS = 10.0  # Capture 2ms before weld
SAMPLE_RATE_HZ = 1000  # 1kHz sampling
PRE_BUFFER_SIZE = int(PRE_TRIGGER_MS * SAMPLE_RATE_HZ / 1000)
pre_trigger_buffer = deque(maxlen=PRE_BUFFER_SIZE)
continuous_sampling = True
esp_connected = False

# Weld capture
MAX_WELD_HISTORY = 15
current_weld_data = {"voltage": [], "current": [], "timestamps": []}
is_capturing = False
weld_start_time = None

# Logs buffer
log_buffer = deque(maxlen=500)

# Thread locks
status_lock = threading.Lock()
weld_lock = threading.Lock()

# Hardware instances
esp_link = None
adc = None
esp32_weld_summary = None

def log(msg):
    """Add message to log buffer and print"""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log_msg = f"[{timestamp}] {msg}"
    log_buffer.append(log_msg)
    print(log_msg, flush=True)

def on_esp_status(data):
    """Callback when ESP32 sends STATUS"""
    global esp_connected, temperature
    with status_lock:
        esp_status.update(data)
        esp_connected = True
        # Extract temperature if present
        if 'temp' in data:
            temperature = data['temp']
        # Also merge cell data into cells_status
        if 'C1' in data:
            cells_status['C1'] = data['C1']
        if 'C2' in data:
            cells_status['C2'] = data['C2']
        if 'C3' in data:
            cells_status['C3'] = data['C3']

def on_esp_cells(data):
    """Callback when ESP32 sends CELLS"""
    with status_lock:
        cells_status.update(data)


def on_esp_log(msg):
    """Handle ESP32 log messages including FIRED events"""
    global esp_status, is_capturing, weld_start_time, current_weld_data, pedal_active, esp32_weld_summary

    # Check for weld trigger (start capture IMMEDIATELY)
    if "trigger weld" in msg:
        log(msg)
        global is_capturing, weld_start_time, current_weld_data
        # Start capturing immediately!
        with weld_lock:
            is_capturing = True
            weld_start_time = time.time()
            current_weld_data = []
            log("🔥 Weld capture started immediately on PEDAL")
        # Set state AFTER is_capturing to prevent race condition
        with status_lock:
            esp_status["state"] = "FIRING"

    # Check for FIRED message (end capture)
    # Check for WDATA message (weld current data)
    elif "WDATA," in msg:
        try:
            # Parse: WDATA,voltage,current,time_us
            parts = msg.split(',')
            if len(parts) >= 4:
                v = float(parts[1])
                i = float(parts[2])
                t_us = int(parts[3])
                
                # Add to current weld data if capturing
                if is_capturing:
                    with weld_lock:
                        current_weld_data.append({
                            "t": t_us / 1000.0,  # Convert to ms
                            "v": v,
                            "i": i
                        })
                    log(f"📊 WDATA: t={t_us/1000.0:.2f}ms, v={v:.2f}V, i={i:.1f}A")
        except Exception as e:
            log(f"⚠️ Failed to parse WDATA: {e}")


    # Check for WDATA_END message (weld summary from ESP32)
    elif "WDATA_END," in msg:
        try:
            # Parse: WDATA_END,sample_count,t_actual_us,peak_current,energy_j,v_drop
            parts = msg.split(',')
            if len(parts) >= 6:
                sample_count = int(parts[1])
                t_actual_us = int(parts[2])
                peak_current = float(parts[3])
                energy_j = float(parts[4])
                v_drop = float(parts[5])
                
                log(f"📊 WDATA_END: samples={sample_count}, duration={t_actual_us/1000.0:.1f}ms, peak={peak_current:.1f}A, energy={energy_j:.2f}J, Vdrop={v_drop:.3f}V")
                
                # Store for use when FIRED message arrives
                global esp32_weld_summary
                esp32_weld_summary = {
                    "peak_current": peak_current,
                    "energy": energy_j,
                    "duration_us": t_actual_us,
                    "v_drop": v_drop
                }
        except Exception as e:
            log(f"⚠️ Failed to parse WDATA_END: {e}")
    elif "FIRED," in msg:
        log(msg)
        # Parse actual weld duration from FIRED message
        weld_duration_ms = 0
        try:
            parts = msg.split(',')
            if len(parts) >= 2:
                weld_duration_ms = int(parts[1])
                log(f"Parsed weld duration: {weld_duration_ms}ms")
        except:
            pass

        # Stop capturing and save immediately
        with weld_lock:
            if is_capturing:
                is_capturing = False
                log("✅ Weld ended - saving data")
                weld_record = save_weld_history(current_weld_data, esp32_weld_summary)
            
            pedal_active = False
            socketio.emit('pedal_active', {"active": False})
            socketio.emit('weld_complete', {
                "weld_number": weld_counter,
                "energy_joules": weld_record["energy_joules"],
                "peak_current_amps": weld_record["peak_current_amps"],
                "duration_ms": weld_record["duration_ms"]
            })

        # Update state
        with status_lock:
            esp_status["state"] = "IDLE"
    else:
        log(msg)  # Log all other ESP32 messages


def load_settings():
    """Load settings from JSON"""
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return {
        "mode": 1,
        "d1": 50,
        "gap1": 0,
        "d2": 0,
        "gap2": 0,
        "d3": 0,
        "pulse_ms": 50,
        "weld_counter": 0
    }

def save_settings(data):
    """Save settings to JSON"""
    # Load existing settings to preserve fields like weld_counter
    settings = load_settings()
    
    # Update with new data
    settings.update(data)
    
    # Save back
    with open(SETTINGS_FILE, 'w') as f:
        json.dump(settings, f, indent=2)
    
    log(f"Settings saved: mode={settings.get('mode')}, d1={settings.get('d1')}, active_preset={settings.get('active_preset', 'N/A')}")

def load_presets():
    """Load presets from JSON"""
    if os.path.exists(PRESETS_FILE):
        try:
            with open(PRESETS_FILE, 'r') as f:
                return json.load(f)
        except:
            pass
    return {
        "P1": {"name": "Preset 1", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0},
        "P2": {"name": "Preset 2", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0},
        "P3": {"name": "Preset 3", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0},
        "P4": {"name": "Preset 4", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0},
        "P5": {"name": "Preset 5", "mode": 1, "d1": 50, "gap1": 0, "d2": 0, "gap2": 0, "d3": 0}
    }

def save_presets(data):
    """Save presets to JSON"""
    with open(PRESETS_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    log(f"Presets saved")

def calculate_energy(weld_data):
    """Calculate energy delivered in Joules using I²R integration (only during actual weld)"""
    TOTAL_RESISTANCE_OHM = 0.00296  # 3.49 mΩ total circuit resistance
    CURRENT_THRESHOLD = 100.0  # Only count energy when current > 100A
    energy = 0.0
    timestamps = weld_data["timestamps"]
    currents = weld_data["current"]

    if len(timestamps) < 2:
        return 0.0

    # Integrate only when current is above threshold AND after trigger (t > 0)
    for i in range(1, len(timestamps)):
        i_avg = (currents[i] + currents[i-1]) / 2.0  # Amps
        t_avg = (timestamps[i] + timestamps[i-1]) / 2.0  # seconds
        
        # Only count energy during actual weld (high current AND positive time)
        if abs(i_avg) > CURRENT_THRESHOLD and t_avg > 0:
            dt = abs(timestamps[i] - timestamps[i-1])  # seconds
            power = (i_avg ** 2) * TOTAL_RESISTANCE_OHM  # Watts = I²R
            energy += power * dt   # Joules

    return energy

def save_weld_history(weld_data, esp32_summary=None):
    """Save weld to history, keep last MAX_WELD_HISTORY"""
    global weld_counter
    weld_counter += 1

    # If ESP32 summary is provided, use it directly
    if esp32_summary:
        energy_j = esp32_summary["energy"]
        peak_current = esp32_summary["peak_current"]
        duration_ms = esp32_summary["duration_us"] / 1000.0
        log(f"✅ Using ESP32 weld summary: {peak_current:.1f}A peak, {energy_j:.2f}J, {duration_ms:.1f}ms")
    else:
        # Fallback: calculate from weld_data
        # Handle both formats: list of dicts or dict of lists
        if isinstance(weld_data, list):
            # Format from ESP32 WDATA: [{"t": ..., "v": ..., "i": ...}, ...]
            timestamps = [d["t"] / 1000.0 for d in weld_data]  # Convert ms to seconds
            voltages = [d["v"] for d in weld_data]
            currents = [d["i"] for d in weld_data]
        else:
            # Format from ADC: {"timestamps": [...], "voltage": [...], "current": [...]}
            timestamps = weld_data.get("timestamps", [])
            voltages = weld_data.get("voltage", [])
            currents = weld_data.get("current", [])

        # Calculate stats
        energy_j = 0.0
        if len(currents) > 1 and len(timestamps) > 1:
            # Integrate I²R over time
            for j in range(len(currents) - 1):
                dt = timestamps[j+1] - timestamps[j]
                i_avg = (currents[j] + currents[j+1]) / 2.0
                energy_j += (i_avg ** 2) * TOTAL_RESISTANCE_OHM * dt

        peak_current = max(currents) if currents else 0.0
        duration_ms = (timestamps[-1] - timestamps[0]) * 1000 if len(timestamps) > 1 else 0.0

    # Save weld data
    filename = f"weld_{weld_counter:04d}.json"
    filepath = os.path.join(WELD_HISTORY_DIR, filename)

    weld_record = {
        "weld_number": weld_counter,
        "timestamp": datetime.now().isoformat(),
        "energy_joules": round(energy_j, 2),
        "peak_current_amps": round(peak_current, 1),
        "duration_ms": round(duration_ms, 1),
        "settings": load_settings(),
        "data": weld_data
    }

    with open(filepath, 'w') as f:
        json.dump(weld_record, f, indent=2)

    # Clean up old welds
    weld_files = sorted([f for f in os.listdir(WELD_HISTORY_DIR) if f.startswith("weld_")])
    if len(weld_files) > MAX_WELD_HISTORY:
        for old_file in weld_files[:-MAX_WELD_HISTORY]:
            os.remove(os.path.join(WELD_HISTORY_DIR, old_file))

    # Update counter in settings
    settings = load_settings()
    settings['weld_counter'] = weld_counter
    save_settings(settings)

    log(f"Weld #{weld_counter} saved: {energy_j:.2f}J, {peak_current:.1f}A peak, {duration_ms:.1f}ms")

    return weld_record
    # Update counter in settings
    settings = load_settings()
    settings['weld_counter'] = weld_counter
    save_settings(settings)
    
    log(f"Weld #{weld_counter} saved: {energy_j:.2f}J, {peak_current:.1f}A peak, {duration_ms:.1f}ms")
    
    return weld_record

def get_weld_history_list():
    """Get list of available weld history files"""
    weld_files = sorted([f for f in os.listdir(WELD_HISTORY_DIR) if f.startswith("weld_")], reverse=True)
    return weld_files[:MAX_WELD_HISTORY]

def calculate_cell_balance():
    """Calculate cell balance stats"""
    c1 = cells_status.get("C1", 0.0)
    c2 = cells_status.get("C2", 0.0)
    c3 = cells_status.get("C3", 0.0)
    
    if c1 == 0 and c2 == 0 and c3 == 0:
        return {"delta": 0.0, "status": "unknown", "color": "gray"}
    
    delta = max(c1, c2, c3) - min(c1, c2, c3)
    
    if delta < 0.05:
        status = "balanced"
        color = "green"
    elif delta < 0.10:
        status = "slight_imbalance"
        color = "yellow"
    else:
        status = "imbalanced"
        color = "red"
    
    return {"delta": round(delta, 3), "status": status, "color": color}

# Routes
@app.route('/')
def index():
    return render_template('control.html')

@app.route('/control')
def control():
    return render_template('control.html')

@app.route('/monitor')
def monitor():
    return render_template('monitor.html')

@app.route('/logs')
def logs():
    return render_template('logs.html')

@app.route('/api/status')
def api_status():
    """Current system status"""
    with status_lock:
        balance = calculate_cell_balance()
        return jsonify({
            **esp_status,
            **cells_status,
            "temperature": temperature,
            "pedal_active": pedal_active,
            "weld_counter": weld_counter,
            "esp_connected": esp_connected,
            "cell_balance": balance
        })

@app.route('/api/get_settings')
def get_settings_route():
    """Get saved settings"""
    settings = load_settings()
    return jsonify({"status": "ok", "settings": settings})

@app.route('/api/save_settings', methods=['POST'])
def save_settings_route():
    """Save settings"""
    data = request.get_json()
    save_settings(data)

    # Send to ESP32
    log(f"DEBUG: esp_link={esp_link}, connected={esp_link.connected if esp_link else 'N/A'}")
    if esp_link and esp_link.connected:
        d1 = data.get('d1', 50)
        cmd = f"SET_PULSE,{d1}"
        log(f"DEBUG: Sending command: {cmd}")
        esp_link.send_command(cmd)
    else:
        log("⚠️ ESP32 not connected - cannot send SET_PULSE")

    return jsonify({"status": "ok"})
@app.route('/api/get_presets')
def get_presets_route():
    """Get all presets"""
    presets = load_presets()
    settings = load_settings()
    return jsonify({"status": "ok", "presets": presets, "active_preset": settings.get("active_preset", "P1")})

@app.route('/api/save_preset', methods=['POST'])
def save_preset_route():
    """Save a preset"""
    data = request.get_json()
    preset_id = data.get('preset_id')
    preset_data = data.get('data')
    
    presets = load_presets()
    presets[preset_id] = preset_data
    save_presets(presets)
    
    return jsonify({"status": "ok"})

@app.route('/api/weld_history')
def weld_history():
    """Get list of weld history files"""
    files = get_weld_history_list()
    
    # Get summary info for each weld
    welds = []
    for filename in files:
        filepath = os.path.join(WELD_HISTORY_DIR, filename)
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                welds.append({
                    "filename": filename,
                    "weld_number": data.get("weld_number"),
                    "timestamp": data.get("timestamp"),
                    "energy_joules": data.get("energy_joules"),
                    "peak_current_amps": data.get("peak_current_amps"),
                    "duration_ms": data.get("duration_ms")
                })
        except:
            pass
    
    return jsonify({"status": "ok", "welds": welds})

@app.route('/api/weld_data/<filename>')
def weld_data(filename):
    """Get specific weld data"""
    filepath = os.path.join(WELD_HISTORY_DIR, filename)
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            return jsonify(json.load(f))
    return jsonify({"status": "error", "message": "Weld not found"}), 404

@app.route('/api/logs')
def api_logs():
    """Get recent logs"""
    return jsonify({"logs": list(log_buffer)})

# SocketIO events
@socketio.on('connect')
def handle_connect():
    log("WebSocket client connected")
    with status_lock:
        balance = calculate_cell_balance()
        emit('status_update', {
            **esp_status,
            **cells_status,
            "temperature": temperature,
            "weld_counter": weld_counter,
            "esp_connected": esp_connected,
            "cell_balance": balance
        })

@socketio.on('disconnect')
def handle_disconnect():
    log("WebSocket client disconnected")


@socketio.on('clear_weld_data')
def handle_clear_weld_data():
    global weld_counter
    
    log("[CLEAR] Clearing weld data and resetting counter")
    
    # Reset counter
    weld_counter = 0
    
    # Delete all weld history files
    try:
        weld_files = [f for f in os.listdir(WELD_HISTORY_DIR) if f.startswith("weld_")]
        for weld_file in weld_files:
            os.remove(os.path.join(WELD_HISTORY_DIR, weld_file))
        log(f"✅ Deleted {len(weld_files)} weld history files")
    except Exception as e:
        log(f"⚠️ Error clearing weld files: {e}")
    
    # Update settings file
    settings = load_settings()
    settings['weld_counter'] = 0
    save_settings(settings)
    
    # Notify all clients
    socketio.emit('weld_data_cleared')
    socketio.emit('weld_counter_update', {'count': 0})
    
    log("✅ Weld data cleared and counter reset")
# Background threads
def status_broadcast_thread():
    """Broadcast status updates and capture weld data with pre-trigger buffering"""
    global is_capturing, weld_start_time, current_weld_data, pedal_active, pre_trigger_buffer

    last_state = "IDLE"

    while True:
        time.sleep(0.0001)  # 10kHz loop - run as fast as possible

        with status_lock:
            current_state = esp_status.get("state", "IDLE")

        # CONTINUOUS SAMPLING (always running to fill pre-trigger buffer)
        if continuous_sampling and adc and adc.initialized:
            try:
                # Read voltage from ESP32
                with status_lock:
                    v = esp_status.get("vpack", 0.0)
                
                # Read current from ADS1256
                i = adc.read_current()
                if math.isnan(i):
                    i = 0.0
                
                # Store in pre-trigger buffer with timestamp
                sample = {
                    "t": time.time(),
                    "v": v,
                    "i": i
                }
                pre_trigger_buffer.append(sample)
            except:
                pass

        # Detect weld start
        if current_state == "FIRING" and last_state != "FIRING":
            # Only start capture if not already capturing (pedal already started it)
            with weld_lock:
                already_capturing = is_capturing

            if not already_capturing:
                log(f"🔥 Weld started - capturing data (broadcast thread)")
                with weld_lock:
                    is_capturing = True
                # Find first high-current sample in pre-trigger buffer (>100A = weld started)
                TRIGGER_THRESHOLD = 100.0  # Amps
                first_high_current_time = None
                for sample in pre_trigger_buffer:
                    if abs(sample["i"]) > TRIGGER_THRESHOLD:
                        first_high_current_time = sample["t"]
                        break
                
                # Use first high-current sample as t=0, or current time if none found
                if first_high_current_time is not None:
                    weld_start_time = first_high_current_time
                    log(f"⚡ Trigger detected at first high-current sample ({TRIGGER_THRESHOLD}A threshold)")
                else:
                    weld_start_time = time.time()
                
                # Initialize weld data with pre-trigger samples
                current_weld_data = {"voltage": [], "current": [], "timestamps": []}
                
                # Copy pre-trigger buffer into weld data
                for sample in pre_trigger_buffer:
                    elapsed = sample["t"] - weld_start_time
                    current_weld_data["timestamps"].append(elapsed)
                    current_weld_data["voltage"].append(sample["v"])
                    current_weld_data["current"].append(sample["i"])
                
            pedal_active = True
            socketio.emit('pedal_active', {"active": True})

        # Capture data during weld (data already in pre_trigger_buffer from continuous sampling)
        if is_capturing and current_state == "FIRING":
            # Get the latest sample from pre-trigger buffer
            if len(pre_trigger_buffer) > 0:
                latest = list(pre_trigger_buffer)[-1]
                
                with weld_lock:
                    elapsed = latest["t"] - weld_start_time
                    # Only add if not already added (avoid duplicates)
                    if len(current_weld_data["timestamps"]) == 0 or elapsed > current_weld_data["timestamps"][-1]:
                        current_weld_data["timestamps"].append(elapsed)
                        current_weld_data["voltage"].append(latest["v"])
                        current_weld_data["current"].append(latest["i"])
                        log(f"ADC: raw current = {latest['i']:.3f} A")

                        # Emit live weld data point
                        socketio.emit('weld_data_point', {
                            "t": elapsed,
                            "v": latest["v"],
                            "i": latest["i"]
                        })

        # Detect weld end
        if current_state != "FIRING" and last_state == "FIRING" and is_capturing:
            log("✅ Weld ended - saving data")
            with weld_lock:
                is_capturing = False
                weld_record = save_weld_history(current_weld_data)

            pedal_active = False
            socketio.emit('pedal_active', {"active": False})
            socketio.emit('weld_complete', {
                "weld_number": weld_counter,
                "energy_joules": weld_record["energy_joules"],
                "peak_current_amps": weld_record["peak_current_amps"],
                "duration_ms": weld_record["duration_ms"]
            })

        # Only update last_state if we're not capturing (prevents duplicate triggers from delayed messages)
        if not is_capturing:
            last_state = current_state

        # Broadcast status every 100ms (10Hz)
        if int(time.time() * 10) % 1 == 0:
            with status_lock:
                balance = calculate_cell_balance()
                socketio.emit('status_update', {
                    **esp_status,
                    **cells_status,
                    "temperature": temperature,
                    "pedal_active": pedal_active,
                    "weld_counter": weld_counter,
                    "esp_connected": esp_connected,
                    "cell_balance": balance
                })
def cleanup():
    """Cleanup GPIO on exit"""
    try:
        import RPi.GPIO as GPIO
        GPIO.cleanup()
        log("GPIO cleanup complete")
    except:
        pass

import atexit
atexit.register(cleanup)

if __name__ == '__main__':
    print("=" * 60, flush=True)
    print("🔥 Spot Welder Control Server", flush=True)
    print("=" * 60, flush=True)
    
    # Load weld counter
    settings = load_settings()
    weld_counter = settings.get('weld_counter', 0)
    log(f"Weld counter: {weld_counter}")
    
    # ADS1256 now on ESP32 - no Pi initialization needed
    # ADS1256 now on ESP32 - no Pi initialization needed
      # Initialize ESP32 link
    log("Initializing ESP32 link...")
    esp_link = ESP32Link(
       host='192.168.68.65',
       port=8888,
       status_callback=on_esp_status,
    )    
    if esp_link.start():
        log("✅ ESP32 link started")
        
        # Send saved settings to ESP32
        time.sleep(0.5)  # Brief delay for ESP32 to be ready
        settings = load_settings()
        d1 = settings.get('d1', 50)
        cmd = f"SET_PULSE,{d1}"
        log(f"📤 Syncing settings to ESP32: {cmd}")
        esp_link.send_command(cmd)
    else:
        log("⚠️ ESP32 link failed to start")
    
    # Start background thread
    broadcast_thread = threading.Thread(target=status_broadcast_thread, daemon=True)
    broadcast_thread.start()
    log("Background broadcast thread started")
    
    # Run server
    log("Starting Flask-SocketIO server on port 8080")
    socketio.run(app, host='0.0.0.0', port=8080, debug=False, allow_unsafe_werkzeug=True)

@app.route('/api/reset_counter', methods=['POST'])
def reset_counter_route():
    """Reset weld counter to 0"""
    global weld_counter
    weld_counter = 0
    settings = load_settings()
    settings['weld_counter'] = 0
    save_settings(settings)
    log("Weld counter reset to 0")
    return jsonify({"status": "ok"})
