import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/falinux/CBL_twinning/install/tb3_safety_stop'
