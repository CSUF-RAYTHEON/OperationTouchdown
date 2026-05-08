import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ground/ugv_repo/OperationTouchdown/ugv_ws/src/install/my_ugv_vision'
