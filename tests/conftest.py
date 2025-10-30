"""
Test fixtures and mocks for gsv8-ethercat tests
"""

import pytest
from unittest.mock import Mock, MagicMock
from dataclasses import dataclass


@dataclass
class MockEtherCATMaster:
    """Mock EtherCAT master for testing"""
    expected_wkc: int = 1
    
    def __init__(self):
        self.in_op = False
        self.slaves = [MockSlave()]
        self.expected_wkc = 1
    
    def config_init(self):
        """Mock config initialization"""
        return len(self.slaves)
    
    def config_map(self):
        """Mock config mapping"""
        return True
    
    def state_check(self, state, timeout=5000):
        """Mock state check"""
        return state
    
    def send_processdata(self):
        """Mock send process data"""
        pass
    
    def receive_processdata(self, timeout=2000):
        """Mock receive process data"""
        return self.expected_wkc
    
    def write_state(self):
        """Mock write state"""
        pass
    
    def read_state(self):
        """Mock read state"""
        pass
    
    def close(self):
        """Mock close"""
        pass


@dataclass
class MockSlave:
    """Mock EtherCAT slave for testing"""

    def __init__(self):
        self.man = 0x000000AB  # ME manufacturer ID
        self.id = 0x00000008   # GSV-8 product code
        self.state = 0x08      # OP state
        self.output = bytearray(320)  # Output buffer (8 channels * 40 bytes)
        self.input = bytearray(320)   # Input buffer
        self.name = b'GSV-8'
        self.revision = 0x00010001
        self.serial = 12345
        self.has_dc = False
        self.DCcycle = 0
        self.input_size = 40
        self.output_size = 0
    
    def sdo_read(self, index, subindex, size=4):
        """Mock SDO read"""
        # Return mock data based on index
        if index == 0x1000:  # Device type
            return b'\x08\x00\x00\x00'
        elif index == 0x1008:  # Device name
            return b'GSV-8\x00'
        elif index == 0x1009:  # Hardware version
            return b'1.0\x00'
        elif index == 0x100A:  # Software version
            return b'1.0\x00'
        elif index == 0x1018:  # Identity
            if subindex == 1:  # Vendor ID
                return b'\xAB\x00\x00\x00'
            elif subindex == 2:  # Product code
                return b'\x08\x00\x00\x00'
        elif index == 0x6126:  # Scale factors
            return b'\x00\x00\x80\x3F'  # 1.0 as float
        elif index == 0x6127:  # Offsets
            return b'\x00\x00\x00\x00'  # 0.0 as float
        return b'\x00' * size
    
    def sdo_write(self, index, subindex, data):
        """Mock SDO write"""
        return len(data)


@pytest.fixture
def mock_pysoem(monkeypatch):
    """Mock pysoem module"""
    mock_module = Mock()
    mock_module.PREOP = 0x02
    mock_module.SAFEOP = 0x04
    mock_module.OP = 0x08
    
    # Mock find_adapters
    mock_adapter = Mock()
    mock_adapter.name = r'\Device\NPF_TEST_ADAPTER'
    mock_adapter.desc = 'Test EtherCAT Adapter'
    mock_module.find_adapters.return_value = [mock_adapter]
    
    # Mock Master class
    mock_module.Master = MagicMock(return_value=MockEtherCATMaster())
    
    monkeypatch.setattr('gsv8_ethercat.sensor.pysoem', mock_module)
    return mock_module


@pytest.fixture
def mock_sensor_connected(mock_pysoem):
    """Create a mocked connected sensor"""
    from gsv8_ethercat import GSV8Sensor

    sensor = GSV8Sensor(r'\Device\NPF_TEST_ADAPTER', slave_position=0)
    sensor._master = MockEtherCATMaster()
    sensor._master.in_op = True
    sensor._is_operational = True
    sensor._slave = sensor._master.slaves[0]

    return sensor


@pytest.fixture
def mock_sensor_data():
    """Create mock sensor data"""
    from gsv8_ethercat.sensor import GSV8Data
    import time
    
    return GSV8Data(
        raw_values=[100.0, -50.0, 75.5, 0.0, 25.3, -10.2, 88.8, 123.4],
        engineering_values=[10.0, -5.0, 7.55, 0.0, 2.53, -1.02, 8.88, 12.34],
        status=[0, 0, 0, 0, 0, 0, 0, 0],
        working_counter=1,
        timestamp=time.time()
    )


@pytest.fixture
def temp_calibration_file(tmp_path):
    """Create a temporary calibration file"""
    cal_file = tmp_path / "test_calibration.txt"
    
    content = """# GSV-8 Calibration File
# Channel,Scale,Offset
0,1.5,10.0
1,2.0,-5.0
2,1.0,0.0
3,0.5,2.5
4,1.2,1.0
5,0.8,-1.5
6,1.1,0.5
7,0.9,3.0
"""
    cal_file.write_text(content)
    return cal_file


@pytest.fixture
def temp_csv_file(tmp_path):
    """Create a temporary CSV file path"""
    return tmp_path / "test_data.csv"
