#!/usr/bin/env python3
"""
GSV-8 Calibration Module
Calibration tools for zeroing and scaling channels
"""

import struct
import time
import statistics
from typing import List

from .sensor import GSV8Sensor


class GSV8Calibration:
    """Calibration tools for GSV-8 sensor"""
    
    def __init__(self, sensor: GSV8Sensor):
        """
        Initialize calibration
        
        Args:
            sensor: Connected GSV8Sensor instance
        """
        self.sensor = sensor
    
    def zero_all_channels(self, samples: int = 100) -> bool:
        """
        Auto-zero all channels by averaging current readings and setting as offset

        Args:
            samples: Number of samples to average for zeroing

        Returns:
            True if successful, False otherwise
        """
        print(f"\nZeroing all channels (averaging {samples} samples)...")
        print("Keep sensor unloaded during zeroing!")
        
        # Collect samples
        channel_sums = [0.0] * 8
        valid_samples = 0
        
        for i in range(samples):
            data = self.sensor.read_data()
            if data:
                for ch in range(8):
                    channel_sums[ch] += data.engineering_values[ch]
                valid_samples += 1
            time.sleep(0.01)
        
        if valid_samples == 0:
            print("ERROR: No valid samples collected")
            return False
        
        # Calculate averages
        averages = [s / valid_samples for s in channel_sums]
        
        # New offsets = old offsets - measured average
        # This makes the current reading become zero
        new_offsets = [
            self.sensor.offsets[i] - averages[i]
            for i in range(8)
        ]
        
        print(f"\nZeroing complete ({valid_samples} samples used):")
        print(f"{'Ch':<4} {'Before':<12} {'Measured':<12} {'New Offset':<12}")
        print("-"*50)
        for i in range(8):
            print(f"{i:<4} {self.sensor.offsets[i]:<12.4f} "
                  f"{averages[i]:<12.4f} {new_offsets[i]:<12.4f}")
        
        # Update in-memory offsets (does not write to device)
        self.sensor.offsets = new_offsets

        print("\n✓ Zeroing applied (in-memory only)")
        print("To make permanent, use write_calibration_to_device()")

        return True
    
    def zero_channel(self, channel: int, samples: int = 100) -> bool:
        """
        Zero a specific channel

        Args:
            channel: Channel number (0-7)
            samples: Number of samples to average

        Returns:
            True if successful, False otherwise
        """
        if channel < 0 or channel > 7:
            raise ValueError(f"Invalid channel {channel}, must be 0-7")
        
        print(f"\nZeroing channel {channel} (averaging {samples} samples)...")
        
        values = []
        for i in range(samples):
            data = self.sensor.read_data()
            if data:
                values.append(data.engineering_values[channel])
            time.sleep(0.01)
        
        if not values:
            print("ERROR: No valid samples collected")
            return False

        average = statistics.mean(values)
        new_offset = self.sensor.offsets[channel] - average

        print(f"Channel {channel}: Measured={average:.4f}, New offset={new_offset:.4f}")

        self.sensor.offsets[channel] = new_offset

        print("✓ Zeroing applied (in-memory only)")
        return True
    
    def calibrate_channel(self, channel: int, known_value: float, samples: int = 100) -> bool:
        """
        Calibrate a channel with a known reference value

        Args:
            channel: Channel number (0-7)
            known_value: The known actual value being measured
            samples: Number of samples to average

        Returns:
            True if successful, False otherwise
        """
        if channel < 0 or channel > 7:
            raise ValueError(f"Invalid channel {channel}, must be 0-7")

        if known_value == 0.0:
            raise ValueError("Known value cannot be zero")
        
        print(f"\nCalibrating channel {channel} with known value {known_value}")
        print(f"Averaging {samples} samples...")
        
        values = []
        for i in range(samples):
            data = self.sensor.read_data()
            if data:
                values.append(data.engineering_values[channel])
            time.sleep(0.01)
        
        if not values:
            print("ERROR: No valid samples collected")
            return False

        measured = statistics.mean(values)

        if abs(measured) < 0.0001:
            print("ERROR: Measured value too close to zero, cannot calibrate")
            return False

        # Calculate new scale: new_scale = old_scale * (known / measured)
        old_scale = self.sensor.scales[channel]
        new_scale = old_scale * (known_value / measured)

        print(f"Channel {channel}:")
        print(f"  Measured       : {measured:.4f}")
        print(f"  Known value    : {known_value:.4f}")
        print(f"  Old scale      : {old_scale:.6f}")
        print(f"  New scale      : {new_scale:.6f}")

        self.sensor.scales[channel] = new_scale

        print("✓ Calibration applied (in-memory only)")
        return True
    
    def write_calibration_to_device(self) -> bool:
        """
        Write current in-memory calibration to device (PERMANENT)
        
        WARNING: This writes to device EEPROM/flash. Use with caution!
        
        Returns:
            True if successful
        """
        print("\n" + "!"*60)
        print("WARNING: Writing calibration to device EEPROM/Flash")
        print("This will PERMANENTLY change device calibration!")
        print("!"*60)
        
        response = input("\nType 'YES' to confirm: ")
        if response != "YES":
            print("Aborted.")
            return False
        
        print("\nWriting calibration data...")
        
        try:
            for ch in range(1, 9):  # Channels 1-8 in SDO
                # Write scale (0x6126)
                scale_bytes = struct.pack('<f', self.sensor.scales[ch-1])
                self.sensor._slave.sdo_write(0x6126, ch, scale_bytes)
                
                # Write offset (0x6127)
                offset_bytes = struct.pack('<f', self.sensor.offsets[ch-1])
                self.sensor._slave.sdo_write(0x6127, ch, offset_bytes)
                
                print(f"Channel {ch-1}: Scale={self.sensor.scales[ch-1]:.6f}, "
                      f"Offset={self.sensor.offsets[ch-1]:.6f}")
            
            print("\n✓ Calibration written to device successfully")
            print("Power cycle the device to ensure changes are saved.")
            return True
            
        except Exception as e:
            print(f"\n✗ ERROR writing calibration: {e}")
            return False
    
    def reset_calibration(self):
        """Reset in-memory calibration to defaults (scale=1, offset=0)"""
        print("\nResetting calibration to defaults (scale=1.0, offset=0.0)")

        self.sensor.scales = [1.0] * 8
        self.sensor.offsets = [0.0] * 8

        print("✓ Calibration reset (in-memory only)")
        print("To restore device calibration, reconnect or read from device.")

    def load_sensor_calibration(self) -> bool:
        """
        Load calibration from the sensor device via SDO

        Returns:
            True if successful, False otherwise
        """
        try:
            if self.sensor._slave is None:
                print("\n✗ ERROR: Sensor slave not initialized")
                return False

            for ch in range(1, 9):
                # Read scale (factor) - 0x6126
                scale_bytes = self.sensor._slave.sdo_read(0x6126, ch)
                self.sensor.scales[ch-1] = struct.unpack('<f', scale_bytes)[0]

                # Read offset - 0x6127
                offset_bytes = self.sensor._slave.sdo_read(0x6127, ch)
                self.sensor.offsets[ch-1] = struct.unpack('<f', offset_bytes)[0]

            print("\n✓ Calibration loaded from sensor device")
            return True

        except Exception as e:
            print(f"\n✗ ERROR loading calibration from sensor: {e}")
            return False

    def display_calibration(self):
        """Display current calibration values"""
        print("\nCurrent Calibration:")
        print(f"{'Channel':<15} {'Scale':<15} {'Offset':<15}")
        print("-" * 45)
        for i in range(8):
            print(f"Channel {i:<7} {self.sensor.scales[i]:<15.6f} {self.sensor.offsets[i]:<15.6f}")
    
    def save_calibration_to_file(self, filename: str = "gsv8_calibration.txt") -> bool:
        """Save current calibration to a text file"""
        try:
            with open(filename, 'w') as f:
                f.write("# GSV-8 Calibration File\n")
                f.write("# Channel,Scale,Offset\n")

                for i in range(8):
                    f.write(f"{i},{self.sensor.scales[i]},{self.sensor.offsets[i]}\n")

            print(f"\n✓ Calibration saved to {filename}")
            return True

        except Exception as e:
            print(f"\n✗ ERROR saving calibration: {e}")
            return False
    
    def load_calibration_from_file(self, filename: str = "gsv8_calibration.txt") -> bool:
        """Load calibration from a text file"""
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('#') or not line:
                        continue

                    parts = line.split(',')
                    if len(parts) >= 3:
                        ch = int(parts[0])
                        if 0 <= ch <= 7:
                            self.sensor.scales[ch] = float(parts[1])
                            self.sensor.offsets[ch] = float(parts[2])

            print(f"\n✓ Calibration loaded from {filename}")
            print("This is in-memory only. Use write_calibration_to_device() to make permanent.")
            return True

        except Exception as e:
            print(f"\n✗ ERROR loading calibration: {e}")
            return False
