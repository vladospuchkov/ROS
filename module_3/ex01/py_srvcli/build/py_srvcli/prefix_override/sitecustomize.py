import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vladislave/ros2_ws/py_srvcli/install/py_srvcli'
