import rclpy
import json
import os
import sys
import threading
from rclpy.node import Node
from sensor_msgs.msg import JointState

#Usado para setup das posicoes do jogo no gradie pois o mesmo usa posicoes fixas

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')


        # Joint names of interest
        self.target_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # Subscription to the /joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

        self.keep_alive = self.create_timer(5.0, lambda: None)
        # Thread-safe storage for latest joint state
        self.latest_joint_state = None
        self.joint_state_lock = threading.Lock()

    def listener_callback(self, msg):
        with self.joint_state_lock:
            self.latest_joint_state = msg

    def getPosicao(self):
        with self.joint_state_lock:
            if not self.latest_joint_state:
                return None

            name_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
            return {name: round(name_pos.get(name, 0.0), 2) for name in self.target_joint_names}

    def SalvarPosicao(self, nome):
        joint_data = self.getPosicao()
        if joint_data is None:
            self.get_logger().warn("Indefinido.")
            return

        subpasta = os.path.dirname(__file__)

        # Go up one level to the parent directory
        pasta = os.path.dirname(subpasta) + "/Defaults/posicoes.json"

        if os.path.exists(pasta):
            with open(pasta, 'r') as f:
                posicoes = json.load(f)
        else:
            posicoes = {}

        posicoes[nome] = joint_data

        with open(pasta, 'w') as f:
            json.dump(posicoes, f, indent=4)




def main():
    rclpy.init()
    node = JointStateSubscriber()

    def command_loop():
        try:
            for line in sys.stdin:
                parts = line.strip().split()
                if parts and parts[0] == "salvarposicao" and len(parts) > 1:
                    node.SalvarPosicao(parts[1])
        except Exception as e:
            print(f"[command_loop] error: {e}", file=sys.stderr)

    threading.Thread(target=command_loop, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
