import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/everd/universidad/git/Robotica-UAO-2025-3/Nicolas_Tafur/install/robot'
