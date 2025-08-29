import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import termios
import tty

# Límites según tu URDF
JOINT_LIMITS = [
    (-1.57, 1.57),  # joint1
    (-1.57, 1.57),  # joint2
    (-1.57, 1.57)   # joint3
]

def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_positions = [0.0, 0.0, 0.0]
        self.get_logger().info("Controles:\n"
                               "q/a: joint1 +/-, w/s: joint2 +/-, e/d: joint3 +/-")
        # Publicar estado inicial
        self.publish_state()

    def run(self):
        while rclpy.ok():
            try:
                key = self.get_key()
            except Exception as e:
                self.get_logger().error(f"Error al leer tecla: {e}")
                break

            if key == 'q':
                self.joint_positions[0] += 0.05
            elif key == 'a':
                self.joint_positions[0] -= 0.05
            elif key == 'w':
                self.joint_positions[1] += 0.05
            elif key == 's':
                self.joint_positions[1] -= 0.05
            elif key == 'e':
                self.joint_positions[2] += 0.05
            elif key == 'd':
                self.joint_positions[2] -= 0.05
            elif key == '\x03':  # Ctrl+C
                break

            for i in range(3):
                old = self.joint_positions[i]
                self.joint_positions[i] = clamp(
                    self.joint_positions[i],
                    JOINT_LIMITS[i][0],
                    JOINT_LIMITS[i][1]
                )
                if self.joint_positions[i] != old:
                    self.get_logger().warn(f"Articulación joint{i+1} alcanzó su límite: {self.joint_positions[i]:.2f}")

            self.publish_state()

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = self.joint_positions
        self.pub.publish(msg)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Restaurar terminal por si quedó en modo raw
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

