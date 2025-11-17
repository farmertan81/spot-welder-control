#!/usr/bin/env python3

# Read the original file
with open('main.py', 'r') as f:
    lines = f.readlines()

# Find the do_weld_ms function (first occurrence)
start_idx = None
end_idx = None
for i, line in enumerate(lines):
    if line.strip().startswith('def do_weld_ms('):
        start_idx = i
    elif start_idx is not None and line.strip().startswith('def ') and i > start_idx:
        end_idx = i
        break

if start_idx is None:
    print("ERROR: Could not find do_weld_ms function")
    exit(1)

print(f"Found do_weld_ms at line {start_idx+1}, ends at line {end_idx+1}")

# New function
new_function = '''def do_weld_ms(pulse_ms):
    global next_weld_ok_at, weld_samples
    if pulse_ms < 1: pulse_ms = 1
    if pulse_ms > 5000: pulse_ms = 5000
    try:
        prev = led[0]
    except Exception:
        prev = (0,0,0)
    led_red()

    # Turn OFF charger FET during weld (critical!)
    FET_CHARGE.off()
    time.sleep(0.001)

    # Clear sample buffer
    weld_samples = []
    
    # Stream voltage and current during weld
    t_start_us = time.ticks_us()
    t_start_ms = time.ticks_ms()
    FET_WELD1.on(); FET_WELD2.on()

    # Fast sampling during weld
    sample_count = 0
    v_start = None
    v_end = None
    
    while time.ticks_diff(time.ticks_ms(), t_start_ms) < pulse_ms:
        t_us = time.ticks_diff(time.ticks_us(), t_start_us)
        
        # Read voltage
        raw = i2c_read_u16(REG_BUS_VOLT)
        if raw is not None:
            v = (raw * VBUS_LSB) * VBUS_SCALE.get(INA_ADDR, 1.0)
            if v_start is None:
                v_start = v
            v_end = v
            
            # Read current from ADS1256 (if DRDY ready)
            current = None
            if ADS1256_AVAILABLE and adc_current and adc_current.drdy.value() == 0:
                try:
                    current = adc_current.read_current()
                    weld_samples.append((t_us, v, current))
                except:
                    pass
            
            # Send data to UART
            if current is not None:
                msg = "WDATA,%.3f,%.1f,%d" % (v, current, t_us)
            else:
                msg = "VDATA,%.3f,%d" % (v, t_us)
            print(msg)
            sample_count += 1

    FET_WELD1.off(); FET_WELD2.off()
    t_actual_us = time.ticks_diff(time.ticks_us(), t_start_us)

    # Calculate statistics
    peak_current = 0.0
    energy_j = 0.0
    
    if len(weld_samples) > 1:
        for _, _, i in weld_samples:
            if i is not None and abs(i) > abs(peak_current):
                peak_current = i
        
        R_total = 0.010
        for idx in range(len(weld_samples) - 1):
            t1, v1, i1 = weld_samples[idx]
            t2, v2, i2 = weld_samples[idx + 1]
            if i1 is not None and i2 is not None:
                dt = (t2 - t1) / 1e6
                p_avg = ((i1**2 + i2**2) / 2.0) * R_total
                energy_j += p_avg * dt
    
    v_drop = 0.0
    if v_start is not None and v_end is not None:
        v_drop = v_start - v_end

    print("WDATA_END,%d,%d,%.1f,%.2f,%.3f" % (sample_count, t_actual_us, peak_current, energy_j, v_drop))

    try:
        led[0] = prev
        led.write()
    except Exception:
        pass

    next_weld_ok_at = time.ticks_add(time.ticks_ms(), WELD_LOCKOUT_MS)

'''

# Replace the function
new_lines = lines[:start_idx] + [new_function] + lines[end_idx:]

# Write back
with open('main.py', 'w') as f:
    f.writelines(new_lines)

print("✓ Replaced do_weld_ms function")

