import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mike54pfeiffer/Desktop/School/ROB599/project_ws/install/interface_pkg'
