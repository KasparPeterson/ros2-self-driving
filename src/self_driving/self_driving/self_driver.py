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

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

TIMER_PERIOD_SECONDS = 1
LINEAR_X = 0.5  # Speed
ANGULAR_Z = 0.5


class SelfDriver(Node):

    def __init__(self):
        super().__init__('self_driver')
        # self.publisher = self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(TIMER_PERIOD_SECONDS, self.timer_callback)
        self.z = ANGULAR_Z

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = LINEAR_X
        msg.angular.z = self.z
        self.z = self.z * -1

        self.get_logger().info(f'Publishing: {msg}')
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    self_driver = SelfDriver()

    rclpy.spin(self_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    self_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
