# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer

TIMER_PERIOD_SECONDS = 1
LINEAR_X = 0.5  # Speed
ANGULAR_Z = 0.5


class SelfDriver(Node):

    def __init__(self, x: float, y: float):
        super().__init__('self_driver')
        self.get_logger().info(f'Initializing Self Driver with x={x}, y={y}')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
        self.timer = self.create_timer(TIMER_PERIOD_SECONDS, self.timer_callback)
        self.z = ANGULAR_Z

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Timer to periodically check TF
        self.create_timer(0.5, self.get_robot_pose)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = LINEAR_X
        msg.angular.z = self.z
        self.z = self.z * -1

        self.get_logger().info(f'Publishing: {msg}')
        self.publisher.publish(msg)

    def get_robot_pose(self):
        try:
            # Replace 'world' and 'base_link' with your actual frames
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'car_world', 'base_link', now)

            pos = trans.transform.translation
            rot = trans.transform.rotation
            self.get_logger().info(f'Position: x={pos.x}, y={pos.y}, z={pos.z}')
            self.get_logger().info(f'Orientation: x={rot.x}, y={rot.y}, z={rot.z}, w={rot.w}')
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')


def main(args=None):
    rclpy.init(args=args)

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    self_driver = SelfDriver(x=x, y=y)

    rclpy.spin(self_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    self_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
