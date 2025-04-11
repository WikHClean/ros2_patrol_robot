import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wch/chapt7/chapt7_ws/src/install/fishbot_application'
