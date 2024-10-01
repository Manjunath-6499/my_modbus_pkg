import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException
from pymodbus.pdu import ExceptionResponse

class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 1)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        self.client = ModbusClient(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity='N',
            stopbits=1,
            bytesize=8
        )

        if not self.client.connect():
            self.get_logger().error('Failed to connect to the Modbus device.')
            return

        self.enable_motor()

    def enable_motor(self):
        try:
            # Assuming that the enable motor command is written to register 40125
            # This is an example, you should check your drive's documentation
            address = 40125 - 40001  # Modbus address offset for holding registers
            value = 159 # Value to enable the motor, adjust if necessary

            # Call the write_register without the 'unit' argument
            result = self.client.write_register(address, value)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to enable motor: {result}")
            else:
                self.get_logger().info("Motor enabled successfully.")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")
        finally:
            self.client.close()

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if (__name__ == '__main__'):
    main()