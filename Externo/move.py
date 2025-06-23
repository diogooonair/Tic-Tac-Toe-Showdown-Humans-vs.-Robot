import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time import sleep
import pyRobotiqGripper

#Usado para todos os movimentos do robo UR3e
class MinimalPoseCommander(Node):
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('minimal_pose_commander')

        # Publisher: sends joint commands to the robot controller
        # Ensure this topic name matches your robot's controller.
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory', # Default topic
            10
        )

        # Define the names and order of joints your controller expects for the trajectory.
        # This is crucial and must match the robot's configuration and the target pose.
        self.controlled_joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Configuration for the Robotiq Gripper
        # sudo chmod a+rw /dev/ttyACM0
        GRIPPER_PORT = '/dev/ttyACM0'
        # Initialize the Robotiq Gripper
        self.gripper = pyRobotiqGripper.RobotiqGripper(portname=GRIPPER_PORT)
        
        #self.closeGripper(85)
        # Brief pause to ensure publisher is registered before sending a message immediately
        # This is often good practice, especially for nodes that publish and then exit quickly.
        sleep(0.5) # 0.5 seconds

        self.get_logger().info('MinimalPoseCommander node initialized and publisher ready.')

    def send_pose_command(self, target_joint_angles, duration_sec=5.0):
        """
        Constructs and sends a single JointTrajectory command to the robot.

        Args:
            target_joint_angles (list of float): The desired joint angles (in radians).
                                                 Order must match self.controlled_joint_names.
            duration_sec (float): The time (in seconds) the trajectory should take.
        Returns:
            bool: True if the command was published, False if input was invalid.
        """
        if len(target_joint_angles) != len(self.controlled_joint_names):
            self.get_logger().error(
                f"Target joint angles count ({len(target_joint_angles)}) does not match "
                f"controlled joints count ({len(self.controlled_joint_names)}). Cannot send command."
            )
            return False

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.controlled_joint_names # Set the joint names for the trajectory

        point = JointTrajectoryPoint()
        point.positions = [float(angle) for angle in target_joint_angles] # Ensure values are float
        point.time_from_start.sec = int(duration_sec)
        # Handle fractional seconds by converting to nanoseconds
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info(f"Published target joint trajectory: {target_joint_angles} "
                               f"to be reached in {duration_sec} seconds.")
        return True

    def openGripper(self, amp):
        # self.gripper.goTo(50, speed=100, force=100)  # Open
        ampTo255 = (amp * 255) / 100
        print("Open Gripper: %", amp, ampTo255)
        self.gripper.goTo(ampTo255, speed=100, force=100)  # open

        return True

    def closeGripper(self, amp):
        # self.gripper.goTo(200, speed=100, force=100)  # Close

        ampTo255 = (amp * 255) / 100
        print("Close Gripper: %", amp, ampTo255)
        self.gripper.goTo(ampTo255, speed=100, force=100)  # Close

def main(data, open=False,args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    pose_commander = MinimalPoseCommander()
    print(data)
    # --- Define your target pose here ---
    # This is the list of joint angles (in radians) you requested.
    # [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
    #target_pose = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]


    if(len(data) > 3):
        target_pose = [float(data['shoulder_pan_joint']), float(data['shoulder_lift_joint']), float(data['elbow_joint']), float(data['wrist_1_joint']), float(data['wrist_2_joint']), float(data['wrist_3_joint'])]

        # Send the command to move to the defined target pose
        pose_commander.get_logger().info(f"Requesting move to target pose: {target_pose}")
        success = pose_commander.send_pose_command(
            target_joint_angles=target_pose,
            duration_sec=2.0  # How long the movement should take (e.g., 5 seconds)
        )

        if success:
            pose_commander.get_logger().info("Movement command sent successfully.")
        else:
            pose_commander.get_logger().error("Failed to send movement command due to input mismatch.")

        # Give a brief moment for the message to be sent over the network,
        # especially if the node shuts down very quickly.
        sleep(0.5) # Adjust if necessary, or rely on ROS 2's publisher QoS settings

    if len(data) < 3:
        if(open):
            pose_commander.openGripper(35)
        else:
            pose_commander.closeGripper(85)

    # Cleanly shutdown the node and ROS
    pose_commander.destroy_node()
    rclpy.shutdown()
    print("MinimalPoseCommander has shut down.")





if __name__ == '__main__':
    main()

