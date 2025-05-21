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
from rclpy.action import ActionServer, GoalResponse
from self_driving.domain import twist_calculator
from self_driving.domain.entities import DesiredTwist
from self_driving.domain.entities import Odometer
from self_driving.domain.entities import Target

from self_driving.domain.entities import Lidar

from self_driving_action_interfaces.action import Drive

DRIVE_FEEDBACK_SLEEP = 0.1


class SelfDriver(Node):

    def __init__(self, x: float, y: float):
        super().__init__('self_driver')
        self.get_logger().info(f'Initializing Self Driver with x={x}, y={y}')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
        self.target_reached = False
        self.target = Target(x=x, y=y)
        self.odometer = Odometer(x=0, y=0, orientation_z=0)
        self.lidar = Lidar(angle_min=0, angle_max=0, ranges=[])

        odometry_subscriber = Subscriber(self, Odometry, "/model/vehicle_blue/odometry")
        lidar_subscriber = Subscriber(self, LaserScan, "/lidar")

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer(
            [odometry_subscriber, lidar_subscriber],
            queue_size, max_delay)
        self.time_sync.registerCallback(self.sync_callback)

        self.feedback_timer = None
        self._action_server = ActionServer(
            self,
            Drive,
            'drive',
            execute_callback=self.drive_callback)

    def sync_callback(self, odom, lidar):
        self.lidar = Lidar(angle_min=lidar.angle_min, angle_max=lidar.angle_max, ranges=lidar.ranges)

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
        self.publish_twist()

    def publish_twist(self):
        desired_twist: DesiredTwist = twist_calculator.execute(self.target, self.odometer, self.lidar)
        if desired_twist:
            msg = Twist()
            msg.linear.x = desired_twist.x
            msg.angular.z = desired_twist.orientation_z

            self.get_logger().info(f"Moving, target x={self.target.x:.2f}, "
                                   f"y={self.target.y:.2f}, "
                                   f"twist x={desired_twist.x:.2f}, "
                                   f"z={desired_twist.orientation_z:.2f},"
                                   f"odom x={self.odometer.x:.2f}, y={self.odometer.y:.2f}, "
                                   f"orientation z={self.odometer.orientation_z:.2f},")

            self.publisher.publish(msg)

            if desired_twist.x == 0.0 and desired_twist.orientation_z == 0.0:
                self.get_logger().info(f'TARGET REACHED!')
                self.target_reached = True

    async def drive_callback(self, drive_action_handle):
        self.get_logger().info('Executing drive goal...')
        x = drive_action_handle.request.target_x
        y = drive_action_handle.request.target_y
        self.target_reached = False
        self.target = Target(x=x, y=y)
        self.get_logger().info(f'New Target: {self.target}')

        # TODO: async callback?
        """def feedback_loop():
            print("feedback_loop")
            if self.target_reached:
                print("  self.target_reached")
                _result = Drive.Result()
                _result.total_distance = 456.0  # Replace with actual computation
                drive_action_handle.succeed()
                print("    set_result:", _result)
                drive_action_handle.set_result(_result)
                self.feedback_timer.cancel()
                return

            feedback_msg = Drive.Feedback()
            feedback_msg.remaining_distance = 123.0  # Replace with actual computation
            feedback_msg.current_x = self.odometer.x
            feedback_msg.current_y = self.odometer.y
            print("  feedback_msg:", feedback_msg)
            drive_action_handle.publish_feedback(feedback_msg)"""

        # Store the timer so we can cancel it later
        # self.feedback_timer = self.create_timer(DRIVE_FEEDBACK_SLEEP, feedback_loop)
        result = Drive.Result()
        result.total_distance = 456.0
        return result


def main():
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    rclpy.init(args=None)
    self_driver = SelfDriver(x=x, y=y)

    rclpy.spin(self_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    self_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
