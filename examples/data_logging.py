#!/usr/bin/env python3
"""
GSV-8 Data Logging Example
Demonstrates CSV logging
"""

from gsv8_ethercat import GSV8Sensor, GSV8Monitor

ADAPTER = r'\Device\NPF_{2315E995-CDA9-4A2D-A9B7-6DFB4C64230C}'

def main():
    print("GSV-8 Data Logging Example")
    print("=" * 50)
    
    sensor = GSV8Sensor(ADAPTER)
    
    if not sensor.connect():
        print("Failed to connect")
        return
    
    monitor = GSV8Monitor(sensor)
    
    # Get parameters from user
    filename = input("Enter output filename (default: data.csv): ").strip()
    if not filename:
        filename = "data.csv"
    
    duration = input("Enter duration in seconds (default: 10): ").strip()
    duration = float(duration) if duration else 10.0
    
    rate = input("Enter sample rate in Hz (default: 100): ").strip()
    rate = float(rate) if rate else 100.0
    
    # Log data
    print(f"\nLogging to {filename} for {duration}s at {rate} Hz...")
    monitor.log_to_file(filename, duration=duration, sample_rate=rate)
    
    sensor.disconnect()
    
    print(f"\nDone! Data saved to {filename}")
    print("You can analyze it with:")
    print(f"  import pandas as pd")
    print(f"  df = pd.read_csv('{filename}')")
    print(f"  df[['ch0_eng', 'ch1_eng']].plot()")

if __name__ == '__main__':
    main()
