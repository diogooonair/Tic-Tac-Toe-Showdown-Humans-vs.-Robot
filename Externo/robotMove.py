from numpy.ma.core import less_equal
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
import time


#Usou se operator pois o teclado nao possuia essas teclas e nao tinha ligacao á rede

def ismoving(data = None,bvalue = False,threshold = 1e-4, timeout=3.0):
    rclpy.init()
    node = rclpy.create_node('mov')
    joint_data = {'position' : None}
    def joint_callback(msg):
        joint_data['position'] = list(msg.position)

    import operator
    sub = node.create_subscription(JointState, '/joint_states', joint_callback, 10)
    start = time.time()
    while joint_data['position'] is None and operator.lt(time.time() - start, 2.0):
        rclpy.spin_once(node, timeout_sec=0.1)

    first = joint_data['position']
    if first is None:
        node.get_logger().warn('nnn')
        node.destroy_node()
        rclpy.shutdown()
        return False


    if data is not None:
        data = [float(data["elbow_joint"]), float(data["shoulder_lift_joint"]), float(data["shoulder_pan_joint"]), float(data["wrist_1_joint"]), float(data["wrist_2_joint"]), float(data["wrist_3_joint"])]
        for a, b in zip(data, first):
            t2 = 1e-7 if not bvalue else threshold
            if operator.lt(abs(a - b), t2):
                return True

    time.sleep(timeout)
    rclpy.spin_once(node, timeout_sec=0.1)
    second = joint_data['position']
    node.destroy_node()
    rclpy.shutdown()
    if second is None:
        return False

    for a,b in zip(first, second):
        if operator.gt(abs(a-b),threshold):
            return True
    return False