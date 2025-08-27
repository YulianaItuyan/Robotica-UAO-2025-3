#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import threading

class CmdDegreePublisher(Node):
    def __init__(self):
        super().__init__('cmd_degree_publisher')
        
        # Create publisher for /cmd_deg topic
        self.publisher = self.create_publisher(
            Float32MultiArray, 
            'cmd_deg', 
            10
        )
        
        # Timer for periodic publishing (optional - can be removed if only manual input)
        # self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Command Degree Publisher Node Started')
        self.get_logger().info('Publishing to topic: /cmd_deg')
        self.get_logger().info('Message format: [joint1, joint2, joint3, part_id]')
        self.print_help()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
    
    def print_help(self):
        help_text = """
=== ROS2 Command Degree Publisher ===
Commands:
  - Enter 4 values: joint1 joint2 joint3 part_id
  - Example: 90 45 120 0
  - Part IDs:
    0 = Left Arm (LA)
    1 = Right Arm (RA) 
    2 = Rear Left Leg (RLL)
    3 = Rear Right Leg (RRL)
    4 = Front Left Leg (FLL)
    5 = Front Right Leg (FRL)
  
  - Type 'help' for this message
  - Type 'quit' or 'exit' to stop
  - Type 'test' for test sequence
=====================================
"""
        print(help_text)
    
    def input_loop(self):
        """Background thread for handling user input"""
        while rclpy.ok():
            try:
                user_input = input("\nEnter command (4 values or 'help'): ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    self.get_logger().info('Shutting down...')
                    rclpy.shutdown()
                    break
                
                elif user_input.lower() == 'help':
                    self.print_help()
                    continue
                
                elif user_input.lower() == 'test':
                    self.run_test_sequence()
                    continue
                
                # Parse input values
                values = user_input.split()
                
                if len(values) != 4:
                    print("❌ Error: Please enter exactly 4 values (joint1 joint2 joint3 part_id)")
                    continue
                
                # Convert to float
                try:
                    float_values = [float(val) for val in values]
                except ValueError:
                    print("❌ Error: All values must be numbers")
                    continue
                
                # Validate part_id
                part_id = int(float_values[3])
                if part_id < 0 or part_id > 5:
                    print("❌ Error: part_id must be between 0 and 5")
                    continue
                
                # Validate joint angles (0-180 degrees)
                for i in range(3):
                    if float_values[i] < 0 or float_values[i] > 180:
                        print(f"⚠️  Warning: Joint {i+1} angle {float_values[i]} is outside 0-180 range")
                
                # Publish the message
                self.publish_degrees(float_values)
                
            except EOFError:
                # Handle Ctrl+D
                self.get_logger().info('EOF received, shutting down...')
                rclpy.shutdown()
                break
            except KeyboardInterrupt:
                # Handle Ctrl+C
                self.get_logger().info('Keyboard interrupt, shutting down...')
                rclpy.shutdown()
                break
            except Exception as e:
                print(f"❌ Error: {str(e)}")
    
    def publish_degrees(self, values):
        """Publish degree values to /cmd_deg topic"""
        msg = Float32MultiArray()
        msg.data = values
        
        self.publisher.publish(msg)
        
        # Log the published message
        part_names = ['Left Arm', 'Right Arm', 'Rear Left Leg', 'Rear Right Leg', 'Front Left Leg', 'Front Right Leg']
        part_id = int(values[3])
        part_name = part_names[part_id] if 0 <= part_id <= 5 else f"Unknown({part_id})"
        
        self.get_logger().info(
            f'📤 Published: [{values[0]:.1f}, {values[1]:.1f}, {values[2]:.1f}, {values[3]:.0f}] '
            f'→ {part_name}'
        )
        print(f"✅ Sent to {part_name}: J1={values[0]:.1f}° J2={values[1]:.1f}° J3={values[2]:.1f}°")
    
    def run_test_sequence(self):
        """Run a predefined test sequence"""
        test_commands = [
            [90, 90, 90, 0],    # Left Arm - center position
            [90, 90, 90, 1],    # Right Arm - center position
            [90, 90, 90, 2],    # Rear Left Leg - center position
            [90, 90, 90, 3],    # Rear Right Leg - center position
            [90, 90, 90, 4],    # Front Left Leg - center position
            [90, 90, 90, 5],    # Front Right Leg - center position
        ]
        
        print("\n🧪 Running test sequence - All parts to center position (90°)...")
        
        for i, cmd in enumerate(test_commands):
            print(f"Test {i+1}/6: ", end="")
            self.publish_degrees(cmd)
            # Small delay between commands
            import time
            time.sleep(0.5)
        
        print("🎉 Test sequence completed!")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CmdDegreePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()