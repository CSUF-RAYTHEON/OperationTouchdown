import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ugv/Desktop/OperationTouchdown/ugv_ws/install/my_ugv_hardware'
