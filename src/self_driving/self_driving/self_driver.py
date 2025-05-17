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
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer

from self_driving.domain import twist_calculator
from self_driving.domain.entities import DesiredTwist
from self_driving.domain.entities import Odometer
from self_driving.domain.entities import Target

LIDAR_TOO_CLOSE = 2.0


class SelfDriver(Node):

    def __init__(self, x: float, y: float):
        super().__init__('self_driver')
        self.get_logger().info(f'Initializing Self Driver with x={x}, y={y}')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
        # self.timer = self.create_timer(TIMER_PERIOD_SECONDS, self.timer_callback)
        self.target = Target(x=x, y=y)
        self.odometer = Odometer(x=0, y=0, orientation_z=0)

        odometry_subscriber = Subscriber(self, Odometry, "/model/vehicle_blue/odometry")
        lidar_subscriber = Subscriber(self, LaserScan, "/lidar")

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer(
            [odometry_subscriber, lidar_subscriber],
            queue_size, max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        # TODO: add lidar data to twist calculator for going around the obstacles
        self.is_lidar_too_close = False

    def sync_callback(self, odom, lidar):
        if self.is_lidar_too_close:
            return

        ranges = lidar.ranges
        # Process LIDAR data (360 degrees, 0 = front, 180 = rear)
        min_distance = min(ranges[0:60] + ranges[300:360])  # Check front 120 degrees
        self.get_logger().info(f'Lidar min_distance: {min_distance}')
        if min_distance < LIDAR_TOO_CLOSE:
            self.is_lidar_too_close = True
            msg = Twist()
            self.get_logger().info(f'Publish stop Twist msg: {msg}')
            self.publisher.publish(msg)
            return

        position = odom.pose.pose.position
        orientation_q = odom.pose.pose.orientation
        # Convert quaternion to tuple
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )

        # Convert to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        self.odometer = Odometer(x=position.x, y=position.y, orientation_z=yaw)
        # self.get_logger().info(f'Position: x={position.x}, y={position.y}, z={position.z}')
        # self.get_logger().info(f'Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')
        self.get_logger().info(f'Odometer: {self.odometer}')
        self.publish_twist()

    def publish_twist(self):
        desired_twist: DesiredTwist = twist_calculator.execute(self.target, self.odometer)
        if desired_twist:
            self.get_logger().info(f'Got DesiredTwist: {desired_twist}')
            msg = Twist()
            msg.linear.x = desired_twist.x
            msg.angular.z = desired_twist.orientation_z

            self.get_logger().info(f'Publishing: {msg}')
            self.get_logger().info(f'DEBUG, x: {self.odometer.x:.2f}, y: {self.odometer.y:.2f}, '
                                   f'orientation_z: {self.odometer.orientation_z:.2f} AND target x: {msg.linear.x:.2f}'
                                   f', z: {msg.angular.z:.2f}')
            self.publisher.publish(msg)


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
