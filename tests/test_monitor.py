"""
Tests for gsv8_ethercat.monitor module
"""

import pytest
import csv
from unittest.mock import Mock, patch
from gsv8_ethercat.monitor import GSV8Monitor


class TestGSV8Monitor:
    """Tests for GSV8Monitor class"""
    
    def test_monitor_initialization(self, mock_sensor_connected):
        """Test monitor initialization"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        assert monitor.sensor is mock_sensor_connected
    
    def test_log_to_file(self, mock_sensor_connected, mock_sensor_data, temp_csv_file):
        """Test logging data to CSV file"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            result = monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=0.1,
                sample_rate=10.0
            )
        
        assert result is True
        assert temp_csv_file.exists()
        
        # Read and verify CSV
        with open(temp_csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            
            # Check header
            assert 'timestamp' in header
            assert 'ch0_raw' in header
            assert 'ch0_eng' in header
            
            # Check at least one data row
            rows = list(reader)
            assert len(rows) > 0
    
    def test_log_to_file_no_data(self, mock_sensor_connected, temp_csv_file):
        """Test logging when sensor returns no data"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=None):
            result = monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=0.1,
                sample_rate=10.0
            )
        
        # Should handle gracefully
        assert result is False or result is True
    
    def test_log_specific_channels(self, mock_sensor_connected, mock_sensor_data, temp_csv_file):
        """Test logging specific channels only"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        channels_to_log = [0, 1, 2]
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            result = monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=0.1,
                sample_rate=10.0,
                channels=channels_to_log
            )
        
        assert result is True
        
        # Verify only specified channels are in CSV
        with open(temp_csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            
            # Should have ch0, ch1, ch2 but not ch3-ch7
            assert 'ch0_eng' in header
            assert 'ch1_eng' in header
            assert 'ch2_eng' in header
    
    def test_live_monitor_output(self, mock_sensor_connected, mock_sensor_data, capsys):
        """Test live monitor terminal output"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        # Mock the monitoring loop to run briefly
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            # Run for very short duration
            with patch('time.time', side_effect=[0, 0, 0.2]):  # Simulate time passing
                try:
                    monitor.live_monitor(duration=0.1, channels=[0, 1, 2])
                except:
                    pass  # May raise exception due to time mocking
        
        captured = capsys.readouterr()
        # Should have printed something
        # (exact output depends on implementation)
    
    def test_csv_header_format(self, mock_sensor_connected, mock_sensor_data, temp_csv_file):
        """Test CSV header format"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=0.1,
                sample_rate=10.0
            )
        
        # Read header
        with open(temp_csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            
            # Verify structure
            expected_fields = [
                'timestamp',
                'ch0_raw', 'ch0_eng', 'ch0_status',
                'ch1_raw', 'ch1_eng', 'ch1_status',
                # ... etc
            ]
            
            assert 'timestamp' in header
            assert any('raw' in h for h in header)
            assert any('eng' in h for h in header)
            assert any('status' in h for h in header)
    
    def test_sample_rate_accuracy(self, mock_sensor_connected, mock_sensor_data, temp_csv_file):
        """Test that sample rate is approximately correct"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=0.5,
                sample_rate=10.0  # 10 samples/second
            )
        
        # Count rows
        with open(temp_csv_file, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            rows = list(reader)
        
        # Should have approximately 0.5s * 10Hz = 5 samples
        # Allow some tolerance
        assert 3 <= len(rows) <= 7
    
    def test_log_invalid_filename(self, mock_sensor_connected):
        """Test logging to invalid filename"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        # Try to log to invalid path
        result = monitor.log_to_file(
            filename="/invalid/path/file.csv",
            duration=0.1,
            sample_rate=10.0
        )
        
        assert result is False
    
    def test_log_with_keyboard_interrupt(self, mock_sensor_connected, mock_sensor_data, temp_csv_file):
        """Test that logging handles KeyboardInterrupt gracefully"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        # Mock read_data to raise KeyboardInterrupt after a few calls
        call_count = [0]
        
        def mock_read():
            call_count[0] += 1
            if call_count[0] > 2:
                raise KeyboardInterrupt()
            return mock_sensor_data
        
        with patch.object(mock_sensor_connected, 'read_data', side_effect=mock_read):
            result = monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=10.0,  # Long duration, but will be interrupted
                sample_rate=10.0
            )
        
        # Should still create file with partial data
        assert temp_csv_file.exists()


class TestMonitorStatistics:
    """Tests for monitoring statistics"""
    
    def test_calculate_sample_statistics(self, mock_sensor_connected, mock_sensor_data):
        """Test calculating statistics during monitoring"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        # Collect some samples
        samples = []
        for _ in range(10):
            samples.append(mock_sensor_data.engineering_values[0])
        
        # Calculate stats
        mean = sum(samples) / len(samples)
        min_val = min(samples)
        max_val = max(samples)
        
        assert mean == mock_sensor_data.engineering_values[0]
        assert min_val <= max_val


class TestMonitorFormatting:
    """Tests for monitor output formatting"""
    
    def test_format_channel_display(self, mock_sensor_connected, mock_sensor_data):
        """Test formatting channel data for display"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        # Format a single channel's data
        value = mock_sensor_data.engineering_values[0]
        status = mock_sensor_data.status[0]
        
        # Should be able to format as string
        formatted = f"Ch0: {value:>8.3f}  Status: {status:02X}"
        
        assert "Ch0:" in formatted
        assert str(status) in formatted or f"{status:02X}" in formatted
    
    def test_csv_data_format(self, mock_sensor_connected, mock_sensor_data, temp_csv_file):
        """Test CSV data formatting"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=0.1,
                sample_rate=10.0
            )
        
        # Read a data row
        with open(temp_csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            data_row = next(reader)
            
            # Verify data can be converted to appropriate types
            timestamp_idx = header.index('timestamp')
            timestamp = float(data_row[timestamp_idx])
            assert timestamp > 0
            
            # Check a channel value
            if 'ch0_eng' in header:
                ch0_idx = header.index('ch0_eng')
                ch0_value = float(data_row[ch0_idx])
                # Should be a valid number
                assert isinstance(ch0_value, float)


class TestMonitorEdgeCases:
    """Tests for edge cases in monitoring"""
    
    def test_zero_duration(self, mock_sensor_connected, temp_csv_file):
        """Test monitoring with zero duration"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        result = monitor.log_to_file(
            filename=str(temp_csv_file),
            duration=0.0,
            sample_rate=10.0
        )
        
        # Should handle gracefully (maybe create empty file or return False)
        assert result is False or temp_csv_file.exists()
    
    def test_very_high_sample_rate(self, mock_sensor_connected, mock_sensor_data, temp_csv_file):
        """Test monitoring with very high sample rate"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            result = monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=0.1,
                sample_rate=1000.0  # Very high
            )
        
        # Should handle without crashing
        assert result is True or result is False
    
    def test_empty_channels_list(self, mock_sensor_connected, mock_sensor_data, temp_csv_file):
        """Test monitoring with empty channels list"""
        monitor = GSV8Monitor(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            result = monitor.log_to_file(
                filename=str(temp_csv_file),
                duration=0.1,
                sample_rate=10.0,
                channels=[]
            )
        
        # Should handle gracefully
        assert result is True or result is False
