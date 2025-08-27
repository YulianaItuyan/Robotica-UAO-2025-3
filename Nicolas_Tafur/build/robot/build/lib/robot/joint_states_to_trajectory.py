#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class JointStateToTrajectory(Node):
    def __init__(self):
        super().__init__('joint_state_to_trajectory')
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.last_positions = None
        self.duration = 2.0  # segundos para el movimiento

        self.sub = self.create_subscription(JointState, 'joint_states', self.cb_joint_states, 10)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/joint_trajectory_controller/follow_joint_trajectory')

    def cb_joint_states(self, msg: JointState):
        # Filtrar solo los joints que nos interesan
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            else:
                return  # si falta alguno, no hacemos nada

        # Si es la primera vez, solo guardamos
        if self.last_positions is None:
            self.last_positions = positions
            return

        # Si hay cambio, enviamos trayectoria
        if positions != self.last_positions:
            self.send_trajectory(self.last_positions, positions)
            self.last_positions = positions

    def send_trajectory(self, start_pos, goal_pos):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # Punto inicial
        p0 = JointTrajectoryPoint()
        p0.positions = start_pos
        p0.time_from_start.sec = 0

        # Punto final
        p1 = JointTrajectoryPoint()
        p1.positions = goal_pos
        p1.time_from_start.sec = int(self.duration)

        goal.trajectory.points = [p0, p1]

        self.client.wait_for_server()
        self.client.send_goal_async(goal)
        self.get_logger().info(f"Trayectoria enviada: {goal_pos} en {self.duration}s")

def main():
    rclpy.init()
    node = JointStateToTrajectory()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
