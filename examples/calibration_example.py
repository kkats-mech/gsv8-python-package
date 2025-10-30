#!/usr/bin/env python3
"""
GSV-8 Calibration Example
Demonstrates zero and calibration workflow
"""

from gsv8_ethercat import GSV8Sensor, GSV8Calibration
import time

ADAPTER = r'\Device\NPF_{2315E995-CDA9-4A2D-A9B7-6DFB4C64230C}'

def main():
    print("GSV-8 Calibration Example")
    print("=" * 50)
    
    sensor = GSV8Sensor(ADAPTER)
    
    if not sensor.connect():
        print("Failed to connect")
        return
    
    cal = GSV8Calibration(sensor)
    
    # Step 1: Zero channel 0
    print("\nStep 1: Zeroing Channel 0")
    print("Remove all loads from the sensor...")
    input("Press Enter when ready...")
    
    cal.zero_channel(channel=0, samples=100)
    
    # Step 2: Calibrate with known value
    print("\nStep 2: Calibrating Channel 0")
    known_force = float(input("Enter the known force to apply (N): "))
    print(f"Apply {known_force} N to channel 0...")
    input("Press Enter when force is applied...")
    
    cal.calibrate_channel(channel=0, known_value=known_force, samples=100)
    
    # Step 3: Verify
    print("\nStep 3: Verifying calibration...")
    print("Reading 10 samples:\n")
    
    for i in range(10):
        force = sensor.get_channel_force(0)
        if force is not None:
            print(f"[{i+1:02d}] Force: {force:.3f} N")
        time.sleep(0.5)
    
    # Step 4: Save
    print("\nStep 4: Saving calibration...")
    filename = "my_calibration.txt"
    cal.save_calibration_to_file(filename)
    print(f"Calibration saved to {filename}")
    
    sensor.disconnect()
    print("\nCalibration complete!")

if __name__ == '__main__':
    main()
