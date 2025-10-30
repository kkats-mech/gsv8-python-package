# GSV8 ROS2 Package

ROS2 package for interfacing with the GSV-8 EtherCAT force-torque sensor. This package provides a ROS2 node that reads data from the sensor and publishes it as standard ROS2 messages.

## Features

- Publishes 6-axis force-torque data as `geometry_msgs/WrenchStamped`
- Optionally publishes all 8 channels as `std_msgs/Float32MultiArray`
- Configurable publishing rate
- Automatic calibration data loading from sensor
- Easy launch file configuration

## Installation

### Prerequisites

1. **Install gsv8_ethercat package:**
   ```bash
   pip install gsv8-ethercat
   ```

2. **ROS2 dependencies:**
   - ROS2 (Humble, Iron, or newer)
   - `rclpy`
   - `geometry_msgs`
   - `std_msgs`

### Build the Package

1. Copy the `gsv8_ros2` folder to your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   cp -r /path/to/gsv8_ros2 .
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select gsv8_ros2
   source install/setup.bash
   ```

## Usage

### Method 1: Using Launch File

Edit the adapter name in the launch command:

**Linux:**
```bash
ros2 launch gsv8_ros2 gsv8_launch.py adapter_name:=eth0
```

**Windows:**
```bash
ros2 launch gsv8_ros2 gsv8_launch.py adapter_name:='\\Device\\NPF_{YOUR-ADAPTER-GUID}'
```

Additional launch arguments:
```bash
ros2 launch gsv8_ros2 gsv8_launch.py \
  adapter_name:=eth0 \
  slave_position:=0 \
  publish_rate:=100.0 \
  frame_id:=ft_sensor \
  publish_all_channels:=true
```

### Method 2: Using Parameters File

1. Copy and edit the config file:
   ```bash
   cp src/gsv8_ros2/config/gsv8_params.yaml my_gsv8_config.yaml
   # Edit my_gsv8_config.yaml with your adapter name
   ```

2. Launch with config:
   ```bash
   ros2 run gsv8_ros2 gsv8_publisher --ros-args --params-file my_gsv8_config.yaml
   ```

### Method 3: Direct Node Run

```bash
ros2 run gsv8_ros2 gsv8_publisher \
  --ros-args \
  -p adapter_name:='\\Device\\NPF_{YOUR-ADAPTER-GUID}' \
  -p slave_position:=0 \
  -p publish_rate:=100.0 \
  -p frame_id:=ft_sensor
```

## Finding Your Adapter Name

**Windows:**
```bash
python -m gsv8_ethercat.cli find-adapter
```

**Linux:**
```bash
ip link show
# or
ifconfig
```

## Topics

### Published Topics

- **`/ft_sensor/wrench`** (`geometry_msgs/WrenchStamped`)
  - 6-axis force-torque data
  - Channels 0-2: Force (Fx, Fy, Fz)
  - Channels 3-5: Torque (Tx, Ty, Tz)

- **`/ft_sensor/raw_channels`** (`std_msgs/Float32MultiArray`)
  - All 8 channels of calibrated sensor data
  - Only published if `publish_all_channels:=true`

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `adapter_name` | string | '' | EtherCAT adapter name (required) |
| `slave_position` | int | 0 | EtherCAT slave position |
| `publish_rate` | double | 100.0 | Publishing rate in Hz |
| `frame_id` | string | 'ft_sensor' | Frame ID for sensor messages |
| `publish_all_channels` | bool | true | Publish all 8 channels |

## Example: Viewing Data

### View WrenchStamped data:
```bash
ros2 topic echo /ft_sensor/wrench
```

### View all 8 channels:
```bash
ros2 topic echo /ft_sensor/raw_channels
```

### Monitor publishing rate:
```bash
ros2 topic hz /ft_sensor/wrench
```

## Channel Mapping

The GSV-8 provides 8 channels. The default mapping is:

| Channel | WrenchStamped Field | Typical Sensor |
|---------|---------------------|----------------|
| 0 | force.x | Fx |
| 1 | force.y | Fy |
| 2 | force.z | Fz |
| 3 | torque.x | Tx (Mx) |
| 4 | torque.y | Ty (My) |
| 5 | torque.z | Tz (Mz) |
| 6 | - | Extra channel 1 |
| 7 | - | Extra channel 2 |

**Note:** Channels 6-7 are only available via the `/ft_sensor/raw_channels` topic.

## Troubleshooting

### "No EtherCAT slaves found"
- Check your adapter name is correct
- Verify the sensor is powered and connected
- On Linux, you may need sudo privileges: `sudo -E ros2 run gsv8_ros2 gsv8_publisher ...`

### "Failed to connect to GSV-8 sensor"
- Ensure `gsv8-ethercat` is installed: `pip install gsv8-ethercat`
- Verify the sensor is in OP state
- Check EtherCAT network configuration

### Permission Issues (Linux)
Add your user to the appropriate group or run with sudo:
```bash
sudo -E bash -c 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run gsv8_ros2 gsv8_publisher ...'
```

## License

MIT License - Same as the parent gsv8-ethercat package

## Related Packages

- [gsv8-ethercat](https://pypi.org/project/gsv8-ethercat/) - Python interface for GSV-8 sensor
