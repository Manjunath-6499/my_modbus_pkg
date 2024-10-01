import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dynaspede/Desktop/my_modbus_pkg/install/my_modbus_pkg'
