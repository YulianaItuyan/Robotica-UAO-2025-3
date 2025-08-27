import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jean/ros2_ws/src/Robotica-UAO-2025-3/Nicolas_Tafur/robot/install/robot'
