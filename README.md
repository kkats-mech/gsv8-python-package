# gsv8-ethercat

Python interface for **ME-Messsysteme GSV-8** force/torque sensor amplifier via EtherCAT.

## Features

- Simple, intuitive API for sensor communication
- Automatic calibration handling (scale, offset)
- Real-time data acquisition from 8 channels
- Diagnostic tools and communication testing
- Calibration utilities (zero, calibrate with known values)
- Data logging and monitoring
- Command-line tools for quick operations

## Installation

```bash
pip install gsv8-ethercat
```

### From source

```bash
git clone https://github.com/kkats-mech/gsv8-ethercat.git
cd gsv8-ethercat
pip install -e .
```

## Quick Start

```python
from gsv8_ethercat import GSV8Sensor

# Initialize sensor
sensor = GSV8Sensor(r'\Device\NPF_{YOUR_ADAPTER_ID}')

# Connect
if sensor.connect():
    # Read data
    data = sensor.read_data()
    print(f"Forces: {data.engineering_values}")
    print(f"Status: {data.status}")
    
    # Disconnect
    sensor.disconnect()
```

### Using Context Manager

```python
from gsv8_ethercat import GSV8Sensor

with GSV8Sensor(adapter_name) as sensor:
    forces = sensor.get_all_forces()
    print(forces)
```

## Command-Line Tools

After installation, three CLI tools are available:

```bash
# Run diagnostics
gsv8-diagnostics 'YOUR_ADAPTER_NAME'

# Interactive calibration wizard
gsv8-calibrate 'YOUR_ADAPTER_NAME'

# Monitor and log data
gsv8-monitor 'YOUR_ADAPTER_NAME' --log data.csv --duration 60
```

## API Overview

### Core Classes

#### GSV8Sensor
Main interface to the sensor hardware.

```python
from gsv8_ethercat import GSV8Sensor

sensor = GSV8Sensor(adapter_name, slave_position=0)
sensor.connect()

# Read all channels
data = sensor.read_data()

# Get specific channel
force = sensor.get_channel_force(channel=0)

# Get all forces
forces = sensor.get_all_forces()

sensor.disconnect()
```

#### GSV8Diagnostics
Device diagnostics and testing.

```python
from gsv8_ethercat import GSV8Sensor, GSV8Diagnostics

sensor = GSV8Sensor(adapter_name)
sensor.connect()

diag = GSV8Diagnostics(sensor)
diag.print_device_info()
diag.check_communication(cycles=100)
diag.analyze_channel_data(duration_sec=5.0)
```

#### GSV8Calibration
Calibration and zeroing utilities.

```python
from gsv8_ethercat import GSV8Sensor, GSV8Calibration

sensor = GSV8Sensor(adapter_name)
sensor.connect()

cal = GSV8Calibration(sensor)

# Zero all channels (no load)
cal.zero_all_channels(samples=100)

# Calibrate with known value
cal.calibrate_channel(channel=0, known_value=100.0, samples=100)

# Save/load calibration
cal.save_calibration_to_file("my_cal.txt")
cal.load_calibration_from_file("my_cal.txt")
```

#### GSV8Monitor
Real-time monitoring and data logging.

```python
from gsv8_ethercat import GSV8Sensor, GSV8Monitor

sensor = GSV8Sensor(adapter_name)
sensor.connect()

monitor = GSV8Monitor(sensor)

# Live terminal display
monitor.live_monitor(channels=[0, 1, 2])

# Log to CSV
monitor.log_to_file("data.csv", duration=60.0, sample_rate=100.0)
```

### Data Structure

```python
@dataclass
class GSV8Data:
    raw_values: List[float]          # 8 raw process values
    engineering_values: List[float]  # 8 calibrated values
    status: List[int]                # 8 status bytes
    working_counter: int             # EtherCAT working counter
    timestamp: float                 # Unix timestamp
```

## Examples

### Simple Force Reading

```python
from gsv8_ethercat import GSV8Sensor
import time

with GSV8Sensor(adapter_name) as sensor:
    for i in range(100):
        force = sensor.get_channel_force(0)
        print(f"Force: {force:.3f} N")
        time.sleep(0.01)
```

### Multi-Channel Monitoring

```python
from gsv8_ethercat import GSV8Sensor
import time

with GSV8Sensor(adapter_name) as sensor:
    while True:
        data = sensor.read_data()
        print(f"Fx:{data.engineering_values[0]:>8.2f}  "
              f"Fy:{data.engineering_values[1]:>8.2f}  "
              f"Fz:{data.engineering_values[2]:>8.2f}")
        time.sleep(0.1)
```

### Calibration Workflow

```python
from gsv8_ethercat import GSV8Sensor, GSV8Calibration

sensor = GSV8Sensor(adapter_name)
sensor.connect()

cal = GSV8Calibration(sensor)

# Step 1: Zero (no load)
print("Remove all loads...")
input("Press Enter...")
cal.zero_channel(0, samples=200)

# Step 2: Calibrate (known load)
print("Apply 100 N...")
input("Press Enter...")
cal.calibrate_channel(0, known_value=100.0, samples=200)

# Step 3: Verify
for i in range(10):
    force = sensor.get_channel_force(0)
    print(f"Reading: {force:.3f} N")
    time.sleep(0.5)

# Step 4: Save
cal.save_calibration_to_file("calibrated.txt")

sensor.disconnect()
```

### Data Logging

```python
from gsv8_ethercat import GSV8Sensor, GSV8Monitor

with GSV8Sensor(adapter_name) as sensor:
    monitor = GSV8Monitor(sensor)
    monitor.log_to_file("experiment.csv", duration=60.0, sample_rate=200.0)

# Later, analyze with pandas
import pandas as pd
df = pd.read_csv("experiment.csv")
df[['ch0_eng', 'ch1_eng', 'ch2_eng']].plot()
```

## Requirements

- Python 3.7+
- pysoem (EtherCAT communication)

### System Requirements

**Windows:**
- Npcap driver (https://npcap.com/)

**Linux:**
- Root privileges or `CAP_NET_RAW` capability
- EtherCAT-capable network adapter

## Hardware Support

- **Device**: ME-Messsysteme GSV-8 amplifier
- **Manufacturer**: ME-Messsysteme GmbH
- **Protocol**: EtherCAT (CoE)
- **Channels**: 8 analog measurement channels
- **Data Rate**: Up to 10 kHz per channel

### PDO Mapping

- **Process Values** (0x6130:1-8): 8 × 32-bit floats
- **Status Bytes** (0x6150:1-8): 8 × 8-bit status
- **Scale Factors** (0x6126:1-8): 8 × 32-bit floats
- **Offsets** (0x6127:1-8): 8 × 32-bit floats

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ME-Messsysteme GmbH for the GSV-8 hardware
- OpenEtherCATsociety for SOEM
- pysoem developers

## Support

- **Hardware**: [ME-Messsysteme Website](https://www.me-systeme.de)
- **pysoem**: [pysoem GitHub](https://github.com/bnjmnp/pysoem)


**Handle with care. Use wisely. Do not drink and drive. Be responsible.**
