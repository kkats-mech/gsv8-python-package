#!/usr/bin/env python3
"""
Simple GSV-8 Reading Example
Demonstrates basic sensor usage
"""

from gsv8_ethercat import GSV8Sensor
import time

# Configure your adapter
ADAPTER = r'\Device\NPF_{2315E995-CDA9-4A2D-A9B7-6DFB4C64230C}'

def main():
    print("GSV-8 Simple Example")
    print("=" * 50)
    
    # Create sensor instance
    sensor = GSV8Sensor(ADAPTER)
    
    # Connect
    if not sensor.connect():
        print("Failed to connect to sensor")
        return
    
    print("Connected! Reading for 10 seconds...\n")
    
    try:
        for i in range(100):
            # Read all data
            data = sensor.read_data()
            
            if data:
                # Print first 3 channels
                print(f"[{i:03d}] "
                      f"Ch0: {data.engineering_values[0]:>8.3f}  "
                      f"Ch1: {data.engineering_values[1]:>8.3f}  "
                      f"Ch2: {data.engineering_values[2]:>8.3f}  "
                      f"WKC: {data.working_counter}")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    finally:
        sensor.disconnect()
        print("Disconnected")

if __name__ == '__main__':
    main()
