"""
Tests for gsv8_ethercat.sensor module
"""

import pytest
import time
from unittest.mock import Mock, patch
from gsv8_ethercat.sensor import GSV8Sensor, GSV8Data


class TestGSV8Data:
    """Tests for GSV8Data dataclass"""
    
    def test_data_creation(self):
        """Test creating GSV8Data object"""
        data = GSV8Data(
            raw_values=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0],
            engineering_values=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
            status=[0, 0, 0, 0, 0, 0, 0, 0],
            working_counter=1,
            timestamp=time.time()
        )
        
        assert len(data.raw_values) == 8
        assert len(data.engineering_values) == 8
        assert len(data.status) == 8
        assert data.working_counter == 1
        assert data.timestamp > 0
    
    def test_data_immutable(self):
        """Test that GSV8Data is immutable (frozen dataclass)"""
        data = GSV8Data(
            raw_values=[1.0] * 8,
            engineering_values=[0.1] * 8,
            status=[0] * 8,
            working_counter=1,
            timestamp=time.time()
        )
        
        with pytest.raises(AttributeError):
            data.working_counter = 2


class TestGSV8Sensor:
    """Tests for GSV8Sensor class"""
    
    def test_sensor_initialization(self):
        """Test sensor initialization"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER', slave_position=0)
        
        assert sensor._adapter_name == r'\Device\NPF_TEST_ADAPTER'
        assert sensor._slave_position == 0
        assert sensor._is_connected is False
        assert sensor._master is None
        assert len(sensor._scales) == 8
        assert len(sensor._offsets) == 8
    
    def test_sensor_default_calibration(self):
        """Test default calibration values"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        for scale in sensor._scales:
            assert scale == 1.0
        
        for offset in sensor._offsets:
            assert offset == 0.0
    
    def test_set_calibration(self):
        """Test setting calibration values"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        sensor.set_calibration(channel=0, scale=2.5, offset=10.0)
        
        assert sensor._scales[0] == 2.5
        assert sensor._offsets[0] == 10.0
    
    def test_set_calibration_invalid_channel(self):
        """Test setting calibration with invalid channel"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        with pytest.raises(ValueError):
            sensor.set_calibration(channel=8, scale=1.0, offset=0.0)
        
        with pytest.raises(ValueError):
            sensor.set_calibration(channel=-1, scale=1.0, offset=0.0)
    
    def test_get_calibration(self):
        """Test getting calibration values"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        sensor.set_calibration(channel=0, scale=2.0, offset=5.0)
        
        scale, offset = sensor.get_calibration(channel=0)
        
        assert scale == 2.0
        assert offset == 5.0
    
    def test_apply_calibration(self):
        """Test applying calibration to raw value"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        sensor.set_calibration(channel=0, scale=2.0, offset=10.0)
        
        # engineering = raw * scale + offset
        # engineering = 100 * 2.0 + 10.0 = 210.0
        result = sensor._apply_calibration(100.0, channel=0)
        
        assert result == 210.0
    
    def test_is_connected_property(self):
        """Test is_connected property"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        assert sensor.is_connected is False
        
        sensor._is_connected = True
        assert sensor.is_connected is True
    
    def test_connect_requires_pysoem(self):
        """Test that connect requires pysoem"""
        with patch('gsv8_ethercat.sensor.pysoem', None):
            sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
            result = sensor.connect()
            assert result is False
    
    def test_read_data_when_not_connected(self):
        """Test reading data when not connected"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        data = sensor.read_data()
        
        assert data is None
    
    def test_get_channel_force_when_not_connected(self):
        """Test getting channel force when not connected"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        force = sensor.get_channel_force(channel=0)
        
        assert force is None
    
    def test_get_all_forces_when_not_connected(self):
        """Test getting all forces when not connected"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        forces = sensor.get_all_forces()
        
        assert forces is None
    
    def test_disconnect_when_not_connected(self):
        """Test disconnect when not connected"""
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        # Should not raise exception
        sensor.disconnect()
    
    def test_context_manager(self, mock_pysoem):
        """Test using sensor as context manager"""
        from gsv8_ethercat import GSV8Sensor
        
        with patch.object(GSV8Sensor, 'connect', return_value=True):
            with patch.object(GSV8Sensor, 'disconnect'):
                with GSV8Sensor(r'\Device\NPF_TEST_ADAPTER') as sensor:
                    assert sensor is not None
    
    def test_read_data_with_mock(self, mock_sensor_connected):
        """Test reading data with mocked sensor"""
        # Mock the _parse_process_data method
        with patch.object(mock_sensor_connected, '_parse_process_data') as mock_parse:
            mock_data = GSV8Data(
                raw_values=[10.0] * 8,
                engineering_values=[1.0] * 8,
                status=[0] * 8,
                working_counter=1,
                timestamp=time.time()
            )
            mock_parse.return_value = mock_data
            
            data = mock_sensor_connected.read_data()
            
            assert data is not None
            assert len(data.engineering_values) == 8
            assert data.working_counter == 1
    
    def test_get_channel_force_with_mock(self, mock_sensor_connected):
        """Test getting single channel force"""
        with patch.object(mock_sensor_connected, 'read_data') as mock_read:
            mock_read.return_value = GSV8Data(
                raw_values=[10.0] * 8,
                engineering_values=[5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0],
                status=[0] * 8,
                working_counter=1,
                timestamp=time.time()
            )
            
            force = mock_sensor_connected.get_channel_force(channel=2)
            
            assert force == 15.0
    
    def test_get_all_forces_with_mock(self, mock_sensor_connected):
        """Test getting all channel forces"""
        with patch.object(mock_sensor_connected, 'read_data') as mock_read:
            expected_forces = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
            mock_read.return_value = GSV8Data(
                raw_values=[10.0] * 8,
                engineering_values=expected_forces,
                status=[0] * 8,
                working_counter=1,
                timestamp=time.time()
            )
            
            forces = mock_sensor_connected.get_all_forces()
            
            assert forces == expected_forces


class TestGSV8SensorIntegration:
    """Integration tests (require actual hardware or full mocking)"""
    
    def test_full_workflow_mock(self, mock_pysoem):
        """Test complete workflow with mocked pysoem"""
        from gsv8_ethercat import GSV8Sensor
        
        # This test verifies the sensor can be instantiated and basic operations work
        # with mocked pysoem
        sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER')
        
        # Test that sensor is created
        assert sensor is not None
        assert not sensor.is_connected
        
        # Set calibration
        sensor.set_calibration(0, 2.0, 10.0)
        scale, offset = sensor.get_calibration(0)
        assert scale == 2.0
        assert offset == 10.0
