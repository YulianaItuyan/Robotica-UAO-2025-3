#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time

class ArmTestController(Node):
    def __init__(self):
        super().__init__('arm_test_controller')
        
        # Publisher para el controlador de trayectorias
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Timer para enviar comandos periódicamente
        self.timer = self.create_timer(3.0, self.send_trajectory)
        self.get_logger().info('Nodo de prueba iniciado. Moviendo brazo...')
        
        # Contador para alternar entre posiciones
        self.counter = 0
        
    def send_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint1', 'joint2', 'joint3']
        
        # Crear punto de trayectoria
        point = JointTrajectoryPoint()
        
        # Alternar entre diferentes posiciones
        if self.counter % 4 == 0:
            point.positions = [0.0, 0.0, 0.0]  # Posición home
        elif self.counter % 4 == 1:
            point.positions = [1.0, 0.5, -0.3]  # Posición 1
        elif self.counter % 4 == 2:
            point.positions = [-0.5, -0.8, 0.5]  # Posición 2
        else:
            point.positions = [0.8, -0.2, 0.8]   # Posición 3
            
        point.velocities = []
        point.accelerations = []
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        msg.points = [point]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Enviando trayectoria {self.counter}: {point.positions}')
        
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    
    node = ArmTestController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
