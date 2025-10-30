#!/usr/bin/env python3
"""
GSV-8 Force Sensor EtherCAT Class
Minimal implementation for easy ROS2 integration
"""

import struct
import pysoem
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class GSV8Data:
    """Container for GSV-8 sensor data"""
    raw_values: List[float]          # 8 raw float values from sensor
    engineering_values: List[float]  # 8 calibrated values (raw * scale + offset)
    status: List[int]                # 8 status bytes (one per channel)
    working_counter: int             # EtherCAT working counter
    timestamp: float                 # Timestamp when data was read


class GSV8Sensor:
    """
    Minimal EtherCAT interface for GSV-8 force/torque sensor amplifier
    
    The GSV-8 provides:
    - 8 channels of measurement data (forces/torques)
    - Each channel has scale and offset calibration
    - 40 bytes total: 32 bytes (8 floats) + 8 bytes (8 status)
    """
    
    def __init__(self, adapter_name: str, slave_position: int = 0):
        """
        Initialize GSV-8 sensor
        
        Args:
            adapter_name: Network adapter
            slave_position: EtherCAT slave position (default: 0 for first slave)
        """
        self.adapter_name = adapter_name
        self.slave_position = slave_position
        
        self._master = pysoem.Master()
        self._slave = None
        
        # Calibration data (will be read from device)
        self.scales = [1.0] * 8
        self.offsets = [0.0] * 8
        self.units = [0] * 8
        
        # State tracking
        self._is_operational = False
    
    def connect(self) -> bool:
        """
        Connect to EtherCAT network and configure GSV-8
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Open adapter
            self._master.open(self.adapter_name)
            
            # Find slaves
            if self._master.config_init() <= 0:
                print("ERROR: No EtherCAT slaves found")
                return False
            
            if len(self._master.slaves) <= self.slave_position:
                print(f"ERROR: Slave position {self.slave_position} not found")
                return False
            
            self._slave = self._master.slaves[self.slave_position]
            print(f"Found GSV-8: {self._slave.name}")
            
            # Go to PREOP for SDO access
            self._master.state = pysoem.PREOP_STATE
            self._master.write_state()
            self._master.state_check(pysoem.PREOP_STATE, 50000)
            
            # Read calibration data from device
            self._read_calibration()
            
            # Map process data
            self._master.config_map()
            
            # Transition to SAFEOP
            self._master.state = pysoem.SAFEOP_STATE
            self._master.write_state()
            self._master.state_check(pysoem.SAFEOP_STATE, 50000)
            
            # Transition to OP
            self._master.state = pysoem.OP_STATE
            self._master.write_state()
            self._master.state_check(pysoem.OP_STATE, 50000)
            
            self._is_operational = True
            print("GSV-8 is operational")
            return True
            
        except Exception as e:
            print(f"ERROR connecting to GSV-8: {e}")
            return False
    
    def _read_calibration(self):
        """Read scale, offset, and unit for each channel from device"""
        try:
            for ch in range(1, 9):
                # Read scale (factor) - 0x6126
                scale_bytes = self._slave.sdo_read(0x6126, ch)
                self.scales[ch-1] = struct.unpack('<f', scale_bytes)[0]
                
                # Read offset - 0x6127
                offset_bytes = self._slave.sdo_read(0x6127, ch)
                self.offsets[ch-1] = struct.unpack('<f', offset_bytes)[0]
                
                # Read unit code - 0x6131
                unit_bytes = self._slave.sdo_read(0x6131, ch)
                self.units[ch-1] = int.from_bytes(unit_bytes, 'little', signed=False)
            
            print("Calibration loaded:")
            print(f"  Scales : {[round(s, 6) for s in self.scales]}")
            print(f"  Offsets: {[round(o, 6) for o in self.offsets]}")
            
        except Exception as e:
            print(f"Warning: Could not read calibration: {e}")
            print("Using default scale=1.0, offset=0.0 for all channels")
    
    def read_data(self) -> Optional[GSV8Data]:
        """
        Read current sensor data
        
        Returns:
            GSV8Data object with raw values, engineering values, and status
            None if read failed
        """
        if not self._is_operational:
            print("ERROR: Sensor not operational. Call connect() first.")
            return None
        
        try:
            # Send and receive process data
            self._master.send_processdata()
            wkc = self._master.receive_processdata(2000)
            
            # Get input buffer
            input_data = self._slave.input
            
            if len(input_data) < 40:
                print(f"ERROR: Expected 40 bytes, got {len(input_data)}")
                return None
            
            # Parse data: 32 bytes (8 floats) + 8 bytes (8 status)
            raw_values = list(struct.unpack('<8f', input_data[0:32]))
            status = list(input_data[32:40])
            
            # Apply calibration: engineering_value = raw * scale + offset
            engineering_values = [
                raw_values[i] * self.scales[i] + self.offsets[i]
                for i in range(8)
            ]
            
            # Import time here to avoid dependency at module level
            import time
            
            return GSV8Data(
                raw_values=raw_values,
                engineering_values=engineering_values,
                status=status,
                working_counter=wkc,
                timestamp=time.time()
            )
            
        except Exception as e:
            print(f"ERROR reading data: {e}")
            return None
    
    def get_channel_force(self, channel: int, use_calibrated: bool = True) -> Optional[float]:
        """
        Get force/torque value for a specific channel
        
        Args:
            channel: Channel number (0-7)
            use_calibrated: If True, return calibrated value; if False, return raw
        
        Returns:
            Force/torque value or None if error
        """
        data = self.read_data()
        if data is None:
            return None
        
        if channel < 0 or channel > 7:
            print(f"ERROR: Channel must be 0-7, got {channel}")
            return None
        
        return data.engineering_values[channel] if use_calibrated else data.raw_values[channel]
    
    def get_all_forces(self, use_calibrated: bool = True) -> Optional[List[float]]:
        """
        Get all 8 force/torque values
        
        Args:
            use_calibrated: If True, return calibrated values; if False, return raw
        
        Returns:
            List of 8 values or None if error
        """
        data = self.read_data()
        if data is None:
            return None
        
        return data.engineering_values if use_calibrated else data.raw_values

    def get_calibration(self, channel: int) -> tuple:
        """
        Get calibration parameters for a specific channel

        Args:
            channel: Channel number (0-7)

        Returns:
            Tuple of (scale, offset)
        """
        if channel < 0 or channel > 7:
            raise ValueError(f"Channel must be 0-7, got {channel}")

        return (self.scales[channel], self.offsets[channel])

    def set_calibration(self, channel: int, scale: float, offset: float):
        """
        Set calibration parameters for a specific channel

        Args:
            channel: Channel number (0-7)
            scale: Scale factor
            offset: Offset value
        """
        if channel < 0 or channel > 7:
            raise ValueError(f"Channel must be 0-7, got {channel}")

        self.scales[channel] = scale
        self.offsets[channel] = offset

    def disconnect(self):
        """Disconnect from EtherCAT and cleanup"""
        if self._is_operational:
            try:
                self._master.state = pysoem.INIT_STATE
                self._master.write_state()
                self._master.close()
                self._is_operational = False
                print("GSV-8 disconnected")
            except Exception as e:
                print(f"Warning during disconnect: {e}")
    
    def __enter__(self):
        """Context manager support"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager cleanup"""
        self.disconnect()


# =============================================================================
# Example Usage
# =============================================================================

def main():
    """Example usage of GSV8Sensor class"""
    
    # Your adapter name
    ADAPTER = r'\Device\NPF_{2315E995-CDA9-4A2D-A9B7-6DFB4C64230C}'
    
    # Create sensor instance
    sensor = GSV8Sensor(ADAPTER, slave_position=0)
    
    # Connect to device
    if not sensor.connect():
        print("Failed to connect to GSV-8")
        return
    
    try:
        print("\nReading sensor data for 10 seconds...")
        print("Press Ctrl+C to stop\n")
        
        import time
        for i in range(100):
            # Read all data
            data = sensor.read_data()
            
            if data:
                print(f"[{i:03d}] WKC={data.working_counter}")
                print(f"  Raw:  {[round(v, 3) for v in data.raw_values]}")
                print(f"  Eng:  {[round(v, 3) for v in data.engineering_values]}")
                print(f"  Stat: {[f'{s:02X}' for s in data.status]}")
                
                # Example: Get specific channel
                force_ch0 = sensor.get_channel_force(0)
                print(f"  Channel 0: {force_ch0:.3f}\n")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    finally:
        # Cleanup
        sensor.disconnect()


# Alternative: using context manager
def main_with_context_manager():
    """Example using context manager (with statement)"""
    ADAPTER = r'\Device\NPF_{2315E995-CDA9-4A2D-A9B7-6DFB4C64230C}'
    
    with GSV8Sensor(ADAPTER) as sensor:
        import time
        for _ in range(10):
            forces = sensor.get_all_forces()
            if forces:
                print(f"Forces: {[round(f, 3) for f in forces]}")
            time.sleep(0.1)
    
    # Automatically disconnects when exiting 'with' block


if __name__ == '__main__':
    main()
    # or: main_with_context_manager()
