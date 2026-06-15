import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/backend_app_tiago/src/app_tiago/install/app_tiago'
