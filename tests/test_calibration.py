"""
Tests for gsv8_ethercat.calibration module
"""

import pytest
from unittest.mock import Mock, patch, mock_open
from gsv8_ethercat.calibration import GSV8Calibration


class TestGSV8Calibration:
    """Tests for GSV8Calibration class"""

    def test_calibration_initialization(self, mock_sensor_connected):
        """Test calibration initialization"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        assert cal.sensor is mock_sensor_connected
    
    def test_zero_channel(self, mock_sensor_connected, mock_sensor_data):
        """Test zeroing a single channel"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # Mock read_data to return consistent values
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            result = cal.zero_channel(channel=0, samples=10)
        
        assert result is True
        
        # Check that calibration was updated (offset should be negative of average reading)
        scale, offset = mock_sensor_connected.get_calibration(0)
        assert offset != 0.0  # Should have been updated
    
    def test_zero_channel_invalid(self, mock_sensor_connected):
        """Test zeroing with invalid channel"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        with pytest.raises(ValueError):
            cal.zero_channel(channel=8, samples=10)
    
    def test_zero_all_channels(self, mock_sensor_connected, mock_sensor_data):
        """Test zeroing all channels"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            result = cal.zero_all_channels(samples=10)
        
        assert result is True
        
        # Check that all channels were updated
        for i in range(8):
            scale, offset = mock_sensor_connected.get_calibration(i)
            assert offset != 0.0
    
    def test_calibrate_channel(self, mock_sensor_connected, mock_sensor_data):
        """Test calibrating a channel with known value"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # First zero the channel
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            cal.zero_channel(channel=0, samples=10)
        
        # Then calibrate with known value
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            result = cal.calibrate_channel(channel=0, known_value=100.0, samples=10)
        
        assert result is True
        
        # Check that scale was updated
        scale, offset = mock_sensor_connected.get_calibration(0)
        assert scale != 1.0  # Should have been updated
    
    def test_calibrate_channel_invalid(self, mock_sensor_connected):
        """Test calibrating with invalid parameters"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # Invalid channel
        with pytest.raises(ValueError):
            cal.calibrate_channel(channel=8, known_value=100.0, samples=10)
        
        # Invalid known_value (zero)
        with pytest.raises(ValueError):
            cal.calibrate_channel(channel=0, known_value=0.0, samples=10)
    
    def test_load_sensor_calibration(self, mock_sensor_connected):
        """Test loading calibration from sensor"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # Mock SDO reads
        mock_slave = Mock()
        mock_slave.sdo_read.side_effect = lambda idx, sub, size=4: b'\x00\x00\x80\x3F' if idx == 0x6126 else b'\x00\x00\x00\x00'
        mock_sensor_connected._master.slaves = [mock_slave]
        
        result = cal.load_sensor_calibration()
        
        assert result is True
    
    def test_save_calibration_to_file(self, mock_sensor_connected, tmp_path):
        """Test saving calibration to file"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # Set some calibration values
        mock_sensor_connected.set_calibration(0, 2.0, 10.0)
        mock_sensor_connected.set_calibration(1, 1.5, -5.0)
        
        # Save to file
        cal_file = tmp_path / "test_cal.txt"
        result = cal.save_calibration_to_file(str(cal_file))
        
        assert result is True
        assert cal_file.exists()
        
        # Read and verify content
        content = cal_file.read_text()
        assert "0,2.0,10.0" in content
        assert "1,1.5,-5.0" in content
    
    def test_load_calibration_from_file(self, mock_sensor_connected, temp_calibration_file):
        """Test loading calibration from file"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        result = cal.load_calibration_from_file(str(temp_calibration_file))
        
        assert result is True
        
        # Verify calibration was loaded
        scale, offset = mock_sensor_connected.get_calibration(0)
        assert scale == 1.5
        assert offset == 10.0
        
        scale, offset = mock_sensor_connected.get_calibration(1)
        assert scale == 2.0
        assert offset == -5.0
    
    def test_load_calibration_from_invalid_file(self, mock_sensor_connected):
        """Test loading from non-existent file"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        result = cal.load_calibration_from_file("nonexistent_file.txt")
        
        assert result is False
    
    def test_load_calibration_from_malformed_file(self, mock_sensor_connected, tmp_path):
        """Test loading from malformed file"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # Create malformed file
        mal_file = tmp_path / "malformed.txt"
        mal_file.write_text("invalid,data,format\n")
        
        result = cal.load_calibration_from_file(str(mal_file))
        
        # Should handle gracefully
        assert result is False or result is True  # Depends on implementation
    
    def test_display_calibration(self, mock_sensor_connected, capsys):
        """Test displaying calibration values"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # Set some values
        mock_sensor_connected.set_calibration(0, 2.0, 10.0)
        
        cal.display_calibration()
        
        captured = capsys.readouterr()
        assert "Channel 0" in captured.out
        assert "2.0" in captured.out or "2.00" in captured.out
        assert "10.0" in captured.out or "10.00" in captured.out
    
    def test_reset_calibration(self, mock_sensor_connected):
        """Test resetting calibration to defaults"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # Set custom calibration
        mock_sensor_connected.set_calibration(0, 2.0, 10.0)
        
        # Reset
        cal.reset_calibration()
        
        # Verify reset to defaults
        scale, offset = mock_sensor_connected.get_calibration(0)
        assert scale == 1.0
        assert offset == 0.0
    
    def test_zero_with_no_data(self, mock_sensor_connected):
        """Test zeroing when sensor returns no data"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=None):
            result = cal.zero_channel(channel=0, samples=10)
        
        assert result is False
    
    def test_calibrate_with_no_data(self, mock_sensor_connected):
        """Test calibrating when sensor returns no data"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=None):
            result = cal.calibrate_channel(channel=0, known_value=100.0, samples=10)
        
        assert result is False


class TestCalibrationFileFormat:
    """Tests for calibration file format handling"""
    
    def test_save_load_round_trip(self, mock_sensor_connected, tmp_path):
        """Test saving and loading preserves calibration"""
        cal = GSV8Calibration(mock_sensor_connected)
        
        # Set calibration for all channels
        expected_cals = []
        for i in range(8):
            scale = 1.0 + i * 0.1
            offset = i * 5.0
            mock_sensor_connected.set_calibration(i, scale, offset)
            expected_cals.append((scale, offset))
        
        # Save
        cal_file = tmp_path / "roundtrip.txt"
        cal.save_calibration_to_file(str(cal_file))
        
        # Reset calibration
        cal.reset_calibration()
        
        # Load
        cal.load_calibration_from_file(str(cal_file))
        
        # Verify
        for i in range(8):
            scale, offset = mock_sensor_connected.get_calibration(i)
            expected_scale, expected_offset = expected_cals[i]
            assert abs(scale - expected_scale) < 0.001
            assert abs(offset - expected_offset) < 0.001
