#!/usr/bin/env python3
"""
GSV-8 Monitor Module
Real-time monitoring and data logging
"""

import time
from typing import Optional, List

from .sensor import GSV8Sensor


class GSV8Monitor:
    """Real-time monitoring and logging for GSV-8"""
    
    def __init__(self, sensor: GSV8Sensor):
        """
        Initialize monitor
        
        Args:
            sensor: Connected GSV8Sensor instance
        """
        self.sensor = sensor
    
    def live_monitor(self, channels: Optional[List[int]] = None, 
                    refresh_rate: float = 10.0):
        """
        Display live sensor readings in terminal
        
        Args:
            channels: List of channels to monitor (None = all 8)
            refresh_rate: Update rate in Hz
        """
        if channels is None:
            channels = list(range(8))
        
        print("\nLive Monitor (Press Ctrl+C to stop)")
        print(f"Monitoring channels: {channels}")
        print("-"*80)
        
        try:
            while True:
                data = self.sensor.read_data()
                if data:
                    # Clear line and print
                    print("\r" + " "*80 + "\r", end="")
                    
                    values_str = " | ".join([
                        f"Ch{ch}:{data.engineering_values[ch]:>8.3f}"
                        for ch in channels
                    ])
                    print(f"WKC:{data.working_counter} | {values_str}", end="", flush=True)
                
                time.sleep(1.0 / refresh_rate)
                
        except KeyboardInterrupt:
            print("\n\nMonitoring stopped")
    
    def log_to_file(self, filename: str, duration: float = 10.0,
                   sample_rate: float = 100.0, channels: Optional[List[int]] = None) -> bool:
        """
        Log sensor data to CSV file

        Args:
            filename: Output CSV filename
            duration: How long to log (seconds)
            sample_rate: Sampling rate in Hz
            channels: List of channels to log (None = all 8)

        Returns:
            True if successful, False otherwise
        """
        if channels is None:
            channels = list(range(8))

        print(f"\nLogging to {filename} for {duration} seconds at {sample_rate} Hz...")

        period = 1.0 / sample_rate
        samples = int(duration * sample_rate)

        try:
            with open(filename, 'w') as f:
                # Write header
                f.write("timestamp,wkc,")
                f.write(",".join([f"ch{i}_raw,ch{i}_eng,ch{i}_status" for i in channels]))
                f.write("\n")

                # Collect data
                for i in range(samples):
                    data = self.sensor.read_data()
                    if data:
                        f.write(f"{data.timestamp:.6f},{data.working_counter},")

                        for ch in channels:
                            f.write(f"{data.raw_values[ch]:.6f},"
                                   f"{data.engineering_values[ch]:.6f},"
                                   f"{data.status[ch]},")

                        f.write("\n")

                    time.sleep(period)

                    # Progress
                    if (i + 1) % 100 == 0:
                        print(f"  {i+1}/{samples} samples...", end="\r")

            print(f"\n✓ Logged {samples} samples to {filename}")
            return True

        except Exception as e:
            print(f"\n✗ ERROR logging: {e}")
            return False
