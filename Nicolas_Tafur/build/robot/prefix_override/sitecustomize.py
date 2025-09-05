import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/felipe/robótica/Robotica-UAO-2025-3/Nicolas_Tafur/install/robot'
