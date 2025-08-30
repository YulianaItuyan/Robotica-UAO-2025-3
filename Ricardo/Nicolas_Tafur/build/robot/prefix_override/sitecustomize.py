import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/richilzz/ros2_ws/src/git/Robotica-UAO-2025-3/Nicolas_Tafur/install/robot'
