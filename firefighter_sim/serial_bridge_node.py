#!/usr/bin/env python3
"""
serial_bridge_node — reads ESP32-CAM UART output and republishes as ROS2 topics.

Replaces landmark_node.py when using real ESP32-CAM hardware.
Publishes the same /landmark_id and /landmark_bearing topics so
navigation_node needs zero changes.

ESP32-CAM serial format (115200 baud):
  LM:0 bearing:-0.12 px:380     → id=0, bearing=-0.12
  LM:1 bearing:+0.05 px:290     → id=1, bearing=+0.05
  LM:0 REACHED px:1240          → id=0, bearing=0.0  (already centered)
  NO_LM                          → id=-1, bearing=0.0

Run:
  ros2 run firefighter_midterm serial_bridge_node --ros-args -p port:=/dev/ttyUSB0

WSL2 note: attach the ESP32-CAM USB port first —
  Windows (PowerShell admin): usbipd bind --busid <id>
                               usbipd attach --wsl --busid <id>
  Then in WSL2: ls /dev/ttyUSB*   (or /dev/ttyACM*)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import serial
import threading


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.pub_id      = self.create_publisher(Int32,   '/landmark_id',      10)
        self.pub_bearing = self.create_publisher(Float32, '/landmark_bearing', 10)

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Opened {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            self.ser = None
            return

        # Read serial in a background thread so the ROS spin isn't blocked
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while rclpy.ok():
            try:
                raw = self.ser.readline()
            except Exception:
                break

            if not raw:
                continue

            line = raw.decode('utf-8', errors='ignore').strip()
            self._parse_and_publish(line)

    def _parse_and_publish(self, line: str):
        """
        Parses one line from the ESP32-CAM and publishes id + bearing.

        Examples:
          "LM:0 bearing:-0.12 px:380"
          "LM:1 REACHED px:1240"
          "NO_LM"
        """
        msg_id      = Int32()
        msg_bearing = Float32()

        if line.startswith('LM:'):
            try:
                parts = line.split()          # ['LM:0', 'bearing:-0.12', 'px:380']
                lm_id = int(parts[0].split(':')[1])

                if parts[1] == 'REACHED':
                    bearing = 0.0             # already centered — go straight
                else:
                    bearing = float(parts[1].split(':')[1])

                msg_id.data      = lm_id
                msg_bearing.data = bearing
                self.pub_id.publish(msg_id)
                self.pub_bearing.publish(msg_bearing)

                self.get_logger().debug(
                    f'LM:{lm_id} bearing={bearing:+.3f}')
            except (IndexError, ValueError) as e:
                self.get_logger().warn(f'Parse error on "{line}": {e}')

        elif line == 'NO_LM':
            msg_id.data      = -1
            msg_bearing.data = 0.0
            self.pub_id.publish(msg_id)
            self.pub_bearing.publish(msg_bearing)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
