import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sidd/ROB599_NarrowCorridorNavigation/install/interface_pkg'
