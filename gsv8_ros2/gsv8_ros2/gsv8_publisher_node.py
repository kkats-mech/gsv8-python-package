#!/usr/bin/env python3
"""
GSV8 Force-Torque Sensor ROS2 Publisher Node

This node reads data from the GSV-8 EtherCAT sensor and publishes it as:
- geometry_msgs/WrenchStamped for 6-axis FT data (Fx, Fy, Fz, Tx, Ty, Tz)
- std_msgs/Float32MultiArray for all 8 channels (optional)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from gsv8_ethercat import GSV8Sensor
import traceback


class GSV8PublisherNode(Node):
    """ROS2 node for publishing GSV-8 force-torque sensor data"""

    def __init__(self):
        super().__init__('gsv8_publisher')

        # Declare parameters
        self.declare_parameter('adapter_name', '')
        self.declare_parameter('slave_position', 0)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('frame_id', 'ft_sensor')
        self.declare_parameter('publish_all_channels', True)

        # Get parameters
        self.adapter_name = self.get_parameter('adapter_name').value
        self.slave_position = self.get_parameter('slave_position').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_all_channels = self.get_parameter('publish_all_channels').value

        # Validate parameters
        if not self.adapter_name:
            self.get_logger().error('Parameter "adapter_name" is required!')
            raise ValueError('adapter_name parameter is empty')

        self.get_logger().info(f'Adapter: {self.adapter_name}')
        self.get_logger().info(f'Slave position: {self.slave_position}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Frame ID: {self.frame_id}')

        # Create publishers
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            'ft_sensor/wrench',
            10
        )

        if self.publish_all_channels:
            self.raw_pub = self.create_publisher(
                Float32MultiArray,
                'ft_sensor/raw_channels',
                10
            )

        # Initialize sensor
        self.sensor = None
        self.is_connected = False

        # Connect to sensor
        self._connect_sensor()

        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('GSV8 publisher node started')

    def _connect_sensor(self):
        """Connect to the GSV-8 sensor"""
        try:
            self.get_logger().info('Connecting to GSV-8 sensor...')
            self.sensor = GSV8Sensor(self.adapter_name, self.slave_position)

            if self.sensor.connect():
                self.is_connected = True
                self.get_logger().info('Successfully connected to GSV-8 sensor')

                # Log calibration info
                self.get_logger().info('Calibration data:')
                for i in range(8):
                    scale, offset = self.sensor.get_calibration(i)
                    self.get_logger().info(f'  Channel {i}: scale={scale:.6f}, offset={offset:.6f}')
            else:
                self.get_logger().error('Failed to connect to GSV-8 sensor')
                self.is_connected = False

        except Exception as e:
            self.get_logger().error(f'Error connecting to sensor: {e}')
            self.get_logger().error(traceback.format_exc())
            self.is_connected = False

    def timer_callback(self):
        """Timer callback to read and publish sensor data"""
        if not self.is_connected:
            return

        try:
            # Read sensor data
            data = self.sensor.read_data()

            if data is None:
                self.get_logger().warn('Failed to read sensor data')
                return

            # Create timestamp
            timestamp = self.get_clock().now().to_msg()

            # Publish WrenchStamped message (6 channels: Fx, Fy, Fz, Tx, Ty, Tz)
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = timestamp
            wrench_msg.header.frame_id = self.frame_id

            # Map channels 0-5 to force (x,y,z) and torque (x,y,z)
            wrench_msg.wrench.force.x = data.engineering_values[0]
            wrench_msg.wrench.force.y = data.engineering_values[1]
            wrench_msg.wrench.force.z = data.engineering_values[2]
            wrench_msg.wrench.torque.x = data.engineering_values[3]
            wrench_msg.wrench.torque.y = data.engineering_values[4]
            wrench_msg.wrench.torque.z = data.engineering_values[5]

            self.wrench_pub.publish(wrench_msg)

            # Optionally publish all 8 channels
            if self.publish_all_channels:
                raw_msg = Float32MultiArray()
                raw_msg.data = [float(v) for v in data.engineering_values]

                # Add dimension info
                dim = MultiArrayDimension()
                dim.label = 'channels'
                dim.size = 8
                dim.stride = 8
                raw_msg.layout.dim.append(dim)

                self.raw_pub.publish(raw_msg)

        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
            self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.sensor and self.is_connected:
            self.get_logger().info('Disconnecting from sensor...')
            self.sensor.disconnect()

        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = GSV8PublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
