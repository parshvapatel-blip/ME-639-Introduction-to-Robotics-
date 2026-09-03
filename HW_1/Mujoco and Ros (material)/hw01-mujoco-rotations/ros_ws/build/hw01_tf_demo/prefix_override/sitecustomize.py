import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/parshva/ITR/HW_1/Mujoco and Ros (material)/hw01-mujoco-rotations/ros_ws/install/hw01_tf_demo'
