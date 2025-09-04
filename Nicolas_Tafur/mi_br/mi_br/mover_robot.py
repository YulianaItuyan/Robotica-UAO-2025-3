import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.move_robot)  # 20 Hz

        # Lista de posiciones objetivo (radianes) → [joint1, joint2]
        self.targets = [
            [0.0, 0.0],       # Inicio
            [0.5, 0.8],       # Recoger
            [-0.5, 0.3],      # Mover
            [0.0, -0.5]       # Soltar
        ]
        self.current_target_index = 0
        self.current_position = [0.0, 0.0]
        self.step = 0
        self.steps_total = 50  # pasos para cada movimiento (~2.5s)
        self.pause_counter = 0

    def move_robot(self):
        target = self.targets[self.current_target_index]

        # Si estamos en pausa
        if self.pause_counter > 0:
            self.pause_counter -= 1
        else:
            # Interpolación suave con perfil seno
            s = (1 - math.cos(math.pi * self.step / self.steps_total)) / 2
            self.current_position[0] = self.current_position[0] + (target[0] - self.current_position[0]) * s
            self.current_position[1] = self.current_position[1] + (target[1] - self.current_position[1]) * s

            self.step += 1
            if self.step > self.steps_total:
                self.step = 0
                self.current_target_index = (self.current_target_index + 1) % len(self.targets)
                self.pause_counter = 10  # pausa de medio segundo

        # Publicar estado de articulaciones
        msg = JointState()
        msg.name = ['joint1', 'joint2']
        msg.position = self.current_position
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

