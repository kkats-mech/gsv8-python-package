#!/usr/bin/env python3
"""
GSV-8 Diagnostics Module
Device information, communication testing, and data analysis
"""

import time
import statistics
from typing import Dict, Optional
from dataclasses import dataclass
import pysoem

from .sensor import GSV8Sensor, GSV8Data


@dataclass
class ChannelStatistics:
    """Statistical data for a single channel"""
    mean: float
    std_dev: float
    min_val: float
    max_val: float
    range: float
    samples: int


class GSV8Diagnostics:
    """Diagnostic and analysis tools for GSV-8 sensor"""
    
    def __init__(self, sensor: GSV8Sensor):
        """
        Initialize diagnostics

        Args:
            sensor: Connected GSV8Sensor instance
        """
        self.sensor = sensor

    def get_device_info(self) -> Optional[Dict]:
        """
        Get device information as a dictionary

        Returns:
            Dictionary with device information or None if not connected
        """
        if not self.sensor._is_operational:
            return None

        slave = self.sensor._slave

        info = {
            'name': slave.name.decode() if isinstance(slave.name, bytes) else str(slave.name),
            'vendor_id': slave.man,
            'manufacturer': slave.man,
            'product_code': slave.id,
            'revision': slave.revision if hasattr(slave, 'revision') else 0,
            'serial': slave.serial if hasattr(slave, 'serial') else 0,
            'state': slave.state,
            'has_dc': slave.has_dc if hasattr(slave, 'has_dc') else False,
            'input_size': slave.input_size if hasattr(slave, 'input_size') else 0,
            'output_size': slave.output_size if hasattr(slave, 'output_size') else 0,
            'slave_position': self.sensor.slave_position,
        }

        return info
    
    def print_device_info(self):
        """Print detailed device information"""
        if not self.sensor._is_operational:
            print("ERROR: Sensor must be connected first")
            return
        
        slave = self.sensor._slave
        
        print("\n" + "="*60)
        print("GSV-8 DEVICE INFORMATION")
        print("="*60)
        print(f"Device Name      : {slave.name}")
        print(f"Manufacturer     : 0x{slave.man:08X} (ME-Messsysteme GmbH)")
        print(f"Product Code     : 0x{slave.id:08X}")
        print(f"Revision         : {slave.revision}")
        print(f"Serial Number    : {slave.serial if hasattr(slave, 'serial') else 'N/A'}")
        print(f"EtherCAT State   : {self._state_name(slave.state)}")
        print(f"Has DC           : {slave.has_dc}")
        print(f"DC Cycle Time    : {slave.DCcycle if slave.has_dc else 'N/A'}")
        print(f"Input Size       : {slave.input_size} bytes")
        print(f"Output Size      : {slave.output_size} bytes")
        print(f"Slave Position   : {self.sensor.slave_position}")
        
        # Try to read identity object (0x1018)
        try:
            vendor_id = self._read_u32(0x1018, 1)
            product_code = self._read_u32(0x1018, 2)
            revision_no = self._read_u32(0x1018, 3)
            print(f"\nCoE Identity (0x1018):")
            print(f"  Vendor ID      : 0x{vendor_id:08X}")
            print(f"  Product Code   : 0x{product_code:08X}")
            print(f"  Revision       : 0x{revision_no:08X}")
        except:
            print("\nCoE Identity: Not available")
        
        print("="*60 + "\n")
    
    def print_calibration_info(self):
        """Print current calibration data for all channels"""
        print("\n" + "="*60)
        print("CALIBRATION DATA")
        print("="*60)
        print(f"{'Ch':<4} {'Scale':<15} {'Offset':<15} {'Unit Code':<12}")
        print("-"*60)
        
        for i in range(8):
            print(f"{i:<4} {self.sensor.scales[i]:<15.6f} "
                  f"{self.sensor.offsets[i]:<15.6f} "
                  f"0x{self.sensor.units[i]:08X}")
        
        print("="*60)
        print("Engineering Value = (Raw Value × Scale) + Offset")
        print("="*60 + "\n")
    
    def print_channel_status(self, data: GSV8Data):
        """
        Print detailed status for each channel
        
        Args:
            data: Current sensor data
        """
        print("\n" + "="*60)
        print("CHANNEL STATUS")
        print("="*60)
        print(f"Working Counter: {data.working_counter}")
        print(f"Timestamp: {data.timestamp:.6f}")
        print("-"*60)
        print(f"{'Ch':<4} {'Raw':<12} {'Eng':<12} {'Status':<10} {'Decoded':<20}")
        print("-"*60)
        
        for i in range(8):
            status_hex = f"0x{data.status[i]:02X}"
            status_bits = self._decode_status_byte(data.status[i])
            print(f"{i:<4} {data.raw_values[i]:<12.4f} "
                  f"{data.engineering_values[i]:<12.4f} "
                  f"{status_hex:<10} {status_bits:<20}")
        
        print("="*60 + "\n")
    
    def _decode_status_byte(self, status: int) -> str:
        """
        Decode status byte (customize based on GSV-8 documentation)
        
        Args:
            status: Status byte value
        
        Returns:
            Human-readable status string
        """
        if status == 0:
            return "OK"
        
        flags = []
        # Common status bits (adjust based on actual GSV-8 documentation)
        if status & 0x01:
            flags.append("ERR")
        if status & 0x02:
            flags.append("WARN")
        if status & 0x04:
            flags.append("OVLD")  # Overload
        if status & 0x08:
            flags.append("ULOD")  # Underload
        if status & 0x10:
            flags.append("CAL")   # Calibrating
        
        return ",".join(flags) if flags else f"0x{status:02X}"
    
    def check_communication(self, cycles: int = 100) -> Dict:
        """
        Test communication quality
        
        Args:
            cycles: Number of cycles to test
        
        Returns:
            Dictionary with communication statistics
        """
        print(f"\nTesting communication for {cycles} cycles...")
        
        success_count = 0
        wkc_errors = 0
        wkc_values = []
        read_times = []
        
        expected_wkc = self.sensor._master.expected_wkc
        
        for i in range(cycles):
            start = time.perf_counter()
            data = self.sensor.read_data()
            end = time.perf_counter()
            
            if data:
                success_count += 1
                wkc_values.append(data.working_counter)
                read_times.append((end - start) * 1000)  # ms
                
                if data.working_counter < expected_wkc:
                    wkc_errors += 1
            
            time.sleep(0.001)  # 1ms cycle
        
        results = {
            'total_cycles': cycles,
            'successful_reads': success_count,
            'failed_reads': cycles - success_count,
            'success_rate': (success_count / cycles) * 100,
            'wkc_errors': wkc_errors,
            'expected_wkc': expected_wkc,
            'avg_read_time_ms': statistics.mean(read_times) if read_times else 0,
            'max_read_time_ms': max(read_times) if read_times else 0,
            'min_read_time_ms': min(read_times) if read_times else 0,
        }
        
        self._print_communication_results(results)
        return results
    
    def _print_communication_results(self, results: Dict):
        """Print communication test results"""
        print("\n" + "="*60)
        print("COMMUNICATION TEST RESULTS")
        print("="*60)
        print(f"Total Cycles     : {results['total_cycles']}")
        print(f"Successful Reads : {results['successful_reads']}")
        print(f"Failed Reads     : {results['failed_reads']}")
        print(f"Success Rate     : {results['success_rate']:.2f}%")
        print(f"WKC Errors       : {results['wkc_errors']}")
        print(f"Expected WKC     : {results['expected_wkc']}")
        print(f"\nRead Time Statistics:")
        print(f"  Average        : {results['avg_read_time_ms']:.3f} ms")
        print(f"  Min            : {results['min_read_time_ms']:.3f} ms")
        print(f"  Max            : {results['max_read_time_ms']:.3f} ms")
        
        if results['success_rate'] < 99.0:
            print("\n⚠ WARNING: Communication issues detected!")
        else:
            print("\n✓ Communication quality: GOOD")
        
        print("="*60 + "\n")
    
    def analyze_channel_data(self, duration_sec: float = 5.0, 
                            sample_rate: float = 100.0) -> Dict[int, ChannelStatistics]:
        """
        Collect and analyze data from all channels
        
        Args:
            duration_sec: How long to collect data
            sample_rate: Sampling rate in Hz
        
        Returns:
            Dictionary mapping channel number to statistics
        """
        print(f"\nCollecting data for {duration_sec} seconds at {sample_rate} Hz...")
        
        # Storage for each channel
        channel_data = {i: [] for i in range(8)}
        
        period = 1.0 / sample_rate
        samples = int(duration_sec * sample_rate)
        
        for i in range(samples):
            data = self.sensor.read_data()
            if data:
                for ch in range(8):
                    channel_data[ch].append(data.engineering_values[ch])
            
            time.sleep(period)
        
        # Calculate statistics
        stats = {}
        for ch in range(8):
            if channel_data[ch]:
                values = channel_data[ch]
                stats[ch] = ChannelStatistics(
                    mean=statistics.mean(values),
                    std_dev=statistics.stdev(values) if len(values) > 1 else 0.0,
                    min_val=min(values),
                    max_val=max(values),
                    range=max(values) - min(values),
                    samples=len(values)
                )
        
        self._print_channel_statistics(stats)
        return stats
    
    def _print_channel_statistics(self, stats: Dict[int, ChannelStatistics]):
        """Print channel statistics"""
        print("\n" + "="*80)
        print("CHANNEL STATISTICS")
        print("="*80)
        print(f"{'Ch':<4} {'Mean':<12} {'StdDev':<12} {'Min':<12} {'Max':<12} {'Range':<12}")
        print("-"*80)
        
        for ch in range(8):
            if ch in stats:
                s = stats[ch]
                print(f"{ch:<4} {s.mean:<12.4f} {s.std_dev:<12.4f} "
                      f"{s.min_val:<12.4f} {s.max_val:<12.4f} {s.range:<12.4f}")
        
        print("="*80 + "\n")
    
    def _read_u32(self, index: int, subindex: int) -> int:
        """Helper to read 32-bit unsigned SDO"""
        data = self.sensor._slave.sdo_read(index, subindex)
        return int.from_bytes(data, 'little', signed=False)
    
    def _state_name(self, state: int) -> str:
        """Convert state code to name"""
        states = {
            1: "INIT",
            2: "PRE-OP",
            4: "SAFE-OP",
            8: "OP"
        }
        return states.get(state, f"UNKNOWN(0x{state:X})")

    def check_status_codes(self, data: Optional[GSV8Data] = None) -> Dict:
        """
        Check status codes for all channels

        Args:
            data: Sensor data to check (if None, reads current data)

        Returns:
            Dictionary with status information
        """
        if data is None:
            data = self.sensor.read_data()

        if data is None:
            return {
                'all_ok': False,
                'error_channels': [],
                'warning_channels': [],
                'status_codes': []
            }

        error_channels = []
        warning_channels = []

        for ch in range(8):
            status = data.status[ch]
            if status & 0x01:  # Error bit
                error_channels.append(ch)
            elif status != 0:  # Non-zero but not error
                warning_channels.append(ch)

        return {
            'all_ok': len(error_channels) == 0 and len(warning_channels) == 0,
            'error_channels': error_channels,
            'warning_channels': warning_channels,
            'status_codes': data.status
        }

    def get_working_counter_stats(self, cycles: int = 100) -> Dict:
        """
        Get working counter statistics

        Args:
            cycles: Number of cycles to sample

        Returns:
            Dictionary with WKC statistics
        """
        wkc_values = []

        for _ in range(cycles):
            data = self.sensor.read_data()
            if data:
                wkc_values.append(data.working_counter)
            time.sleep(0.001)

        if not wkc_values:
            return {
                'samples': 0,
                'expected': 0,
                'min': 0,
                'max': 0,
                'mean': 0,
                'errors': 0
            }

        expected = self.sensor._master.expected_wkc if hasattr(self.sensor._master, 'expected_wkc') else 1
        errors = sum(1 for wkc in wkc_values if wkc < expected)

        return {
            'samples': len(wkc_values),
            'expected': expected,
            'min': min(wkc_values),
            'max': max(wkc_values),
            'mean': statistics.mean(wkc_values),
            'errors': errors
        }

    def print_channel_statistics(self, stats: Dict[int, ChannelStatistics]):
        """Print channel statistics (alias for _print_channel_statistics)"""
        self._print_channel_statistics(stats)
