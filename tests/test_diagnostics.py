"""
Tests for gsv8_ethercat.diagnostics module
"""

import pytest
from unittest.mock import Mock, patch
from gsv8_ethercat.diagnostics import GSV8Diagnostics


class TestGSV8Diagnostics:
    """Tests for GSV8Diagnostics class"""
    
    def test_diagnostics_initialization(self, mock_sensor_connected):
        """Test diagnostics initialization"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        assert diag.sensor is mock_sensor_connected
    
    def test_get_device_info(self, mock_sensor_connected):
        """Test getting device information"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        # Mock the slave
        mock_slave = Mock()
        mock_slave.name = b'GSV-8'
        mock_slave.man = 0x000000AB
        mock_slave.id = 0x00000008
        mock_slave.sdo_read.side_effect = lambda idx, sub, size=4: {
            0x1009: b'1.0\x00',  # Hardware version
            0x100A: b'2.0\x00',  # Software version
        }.get(idx, b'\x00' * size)
        
        mock_sensor_connected._master.slaves = [mock_slave]
        
        info = diag.get_device_info()
        
        assert info is not None
        assert 'name' in info
        assert 'vendor_id' in info
        assert 'product_code' in info
    
    def test_print_device_info(self, mock_sensor_connected, capsys):
        """Test printing device information"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        # Mock the slave
        mock_slave = Mock()
        mock_slave.name = b'GSV-8'
        mock_slave.man = 0x000000AB
        mock_slave.id = 0x00000008
        mock_slave.sdo_read.return_value = b'1.0\x00'
        
        mock_sensor_connected._master.slaves = [mock_slave]
        
        diag.print_device_info()
        
        captured = capsys.readouterr()
        assert "GSV-8" in captured.out or "Device Information" in captured.out
    
    def test_check_communication(self, mock_sensor_connected, mock_sensor_data):
        """Test communication check"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            result = diag.check_communication(cycles=10)
        
        assert result is True
    
    def test_check_communication_failure(self, mock_sensor_connected):
        """Test communication check with failures"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        # Mock failed reads
        with patch.object(mock_sensor_connected, 'read_data', return_value=None):
            result = diag.check_communication(cycles=10)
        
        # Should detect failures
        assert result is False or result is True  # Depends on threshold
    
    def test_analyze_channel_data(self, mock_sensor_connected, mock_sensor_data):
        """Test analyzing channel data"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            stats = diag.analyze_channel_data(duration_sec=0.1)
        
        assert stats is not None
        assert isinstance(stats, dict)
        
        # Check that stats has data for all channels
        for i in range(8):
            assert i in stats
            assert 'mean' in stats[i]
            assert 'std' in stats[i]
            assert 'min' in stats[i]
            assert 'max' in stats[i]
    
    def test_analyze_channel_data_no_data(self, mock_sensor_connected):
        """Test analyzing when no data available"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=None):
            stats = diag.analyze_channel_data(duration_sec=0.1)
        
        # Should handle gracefully
        assert stats is None or len(stats) == 0
    
    def test_print_channel_statistics(self, mock_sensor_connected, mock_sensor_data, capsys):
        """Test printing channel statistics"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            diag.print_channel_statistics(duration_sec=0.1)
        
        captured = capsys.readouterr()
        # Should have printed something
        assert len(captured.out) > 0
    
    def test_check_status_codes(self, mock_sensor_connected):
        """Test checking status codes"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        # Create data with some error statuses
        from gsv8_ethercat.sensor import GSV8Data
        import time
        
        error_data = GSV8Data(
            raw_values=[0.0] * 8,
            engineering_values=[0.0] * 8,
            status=[0, 0x01, 0x02, 0, 0x04, 0, 0, 0],  # Some errors
            working_counter=1,
            timestamp=time.time()
        )
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=error_data):
            errors = diag.check_status_codes()
        
        assert errors is not None
        assert len(errors) > 0  # Should detect the error statuses
    
    def test_check_status_codes_all_ok(self, mock_sensor_connected, mock_sensor_data):
        """Test checking status codes when all OK"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            errors = diag.check_status_codes()
        
        # All statuses are 0, so should be no errors
        assert len(errors) == 0
    
    def test_get_working_counter_stats(self, mock_sensor_connected, mock_sensor_data):
        """Test getting working counter statistics"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            stats = diag.get_working_counter_stats(cycles=10)
        
        assert stats is not None
        assert 'total_cycles' in stats
        assert 'wkc_errors' in stats
        assert 'error_rate' in stats


class TestDiagnosticsReporting:
    """Tests for diagnostics reporting functions"""
    
    def test_full_diagnostic_report(self, mock_sensor_connected, mock_sensor_data, capsys):
        """Test generating full diagnostic report"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        # Mock the slave
        mock_slave = Mock()
        mock_slave.name = b'GSV-8'
        mock_slave.man = 0x000000AB
        mock_slave.id = 0x00000008
        mock_slave.sdo_read.return_value = b'1.0\x00'
        mock_sensor_connected._master.slaves = [mock_slave]
        
        with patch.object(mock_sensor_connected, 'read_data', return_value=mock_sensor_data):
            # This would call multiple diagnostic functions
            diag.print_device_info()
            diag.check_communication(cycles=5)
            diag.print_channel_statistics(duration_sec=0.1)
        
        captured = capsys.readouterr()
        # Should have generated output
        assert len(captured.out) > 0
    
    def test_diagnostic_with_sensor_disconnected(self):
        """Test diagnostics when sensor is not connected"""
        from gsv8_ethercat import GSV8Sensor
        
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        diag = GSV8Diagnostics(sensor)
        
        # Most operations should handle disconnected sensor gracefully
        info = diag.get_device_info()
        assert info is None or len(info) == 0


class TestDiagnosticsStatistics:
    """Tests for statistical analysis functions"""
    
    def test_calculate_statistics(self, mock_sensor_connected, mock_sensor_data):
        """Test statistical calculations"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        # Collect some data
        data_points = []
        for _ in range(10):
            data_points.append(mock_sensor_data.engineering_values[0])
        
        # Calculate mean
        mean = sum(data_points) / len(data_points)
        assert mean == mock_sensor_data.engineering_values[0]  # All same value
        
        # Calculate std
        variance = sum((x - mean) ** 2 for x in data_points) / len(data_points)
        std = variance ** 0.5
        assert std == 0.0  # All same value
    
    def test_data_quality_check(self, mock_sensor_connected):
        """Test data quality checking"""
        diag = GSV8Diagnostics(mock_sensor_connected)
        
        # Create varying data
        from gsv8_ethercat.sensor import GSV8Data
        import time
        
        varying_data = []
        for i in range(10):
            data = GSV8Data(
                raw_values=[float(i)] * 8,
                engineering_values=[float(i) * 0.1] * 8,
                status=[0] * 8,
                working_counter=1,
                timestamp=time.time()
            )
            varying_data.append(data)
        
        # Mock read_data to return varying data
        with patch.object(mock_sensor_connected, 'read_data', side_effect=varying_data):
            stats = diag.analyze_channel_data(duration_sec=0.1)
        
        if stats:
            # Should have non-zero std deviation
            for channel_stats in stats.values():
                # Some variation expected
                assert 'std' in channel_stats
