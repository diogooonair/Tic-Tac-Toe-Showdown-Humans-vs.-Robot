import os
import json
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.executors import SingleThreadedExecutor
import sys
from time import sleep


#Somente usado no gradie devido a nao necessitar de inicializacao do gripper
class SimpleExecutor(Node):
    def __init__(self):
        super().__init__('simple_executor')

        self.current_positions = None
        self.target_positions = None
        self.tolerance = 0.05

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        self.current_positions = msg.position

    def is_target_reached(self, target_positions):
        if self.current_positions is None:
            return False

        order = [5, 0, 1, 2, 3, 4]  # reorder if necessary
        current = np.round(np.array(self.current_positions), 2)[order]
        target = np.round(np.array(target_positions), 2)

        return np.all(np.abs(current - target) <= self.tolerance)

    def send_joint_trajectory(self, positions):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 5
        msg.points.append(point)

        self.publisher.publish(msg)

    def move_to_waypoint(self, name):
        subpasta = os.path.dirname(__file__)

        # Go up one level to the parent directory
        pasta = os.path.dirname(subpasta) + "/Defaults/posicoes.json"

        if os.path.exists(pasta):
            with open(pasta, 'r') as f:
                posicoes = json.load(f)

        data = posicoes[name]


        joint_order = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.target_positions = [data[j] for j in joint_order]
        print(data)

        self.send_joint_trajectory(self.target_positions)

        self.get_logger().info("Command sent. Waiting for robot to reach position...")

        timeout = 20  # seconds
        start = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.is_target_reached(self.target_positions):
                self.get_logger().info("Target reached!")
                return True
            if (self.get_clock().now().nanoseconds / 1e9 - start) > timeout:
                self.get_logger().warn("Timeout: Robot did not reach the target.")
                return False
        return False

def main(waypoint_name='ponto_escondido'):

    rclpy.init()

    node = SimpleExecutor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # Read waypoint name from standard input
        print("Waiting for waypoint name via stdin...")
        #waypoint_name = sys.stdin.readline().strip()


        if not waypoint_name:
            node.get_logger().warn("No waypoint name received from stdin.")
        else:
            success = node.move_to_waypoint(waypoint_name)
            if success:
                node.get_logger().info(f"Successfully moved to waypoint: {waypoint_name}")
            else:
                node.get_logger().warn(f"Failed to move to waypoint: {waypoint_name}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
