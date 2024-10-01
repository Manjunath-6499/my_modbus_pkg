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

    def enable_motor(self, motor_id):
        if motor_id not in [1, 2]:
            self.get_logger().error("Invalid motor ID. Use 1 or 2.")
            return
        self._write_register(motor_id, 158)
        self.get_logger().info(f"Motor {motor_id} enabled successfully.")

    def disable_motor(self, motor_id):
        if motor_id not in [1, 2]:
            self.get_logger().error("Invalid motor ID. Use 1 or 2.")
            return
        self._write_register(motor_id, 0)
        self.get_logger().info(f"Motor {motor_id} disabled successfully.")

    def _write_register(self, motor_id, value):
        try:
            # Modbus address offset for holding registers
            address = 40125 - 40001
            
            # Select the Modbus unit ID (device address)
            self.client.unit_id = motor_id

            # Call the write_register without the 'unit' argument
            result = self.client.write_register(address, value)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to write to register: {result}")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")
        finally:
            self.client.close()

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    while rclpy.ok():
        command = input("Enter command (enable/disable/exit): ").strip().lower()
        motor_id = int(input("Enter motor ID (1 or 2): ").strip())

        if command == 'enable':
            node.enable_motor(motor_id)
        elif command == 'disable':
            node.disable_motor(motor_id)
        elif command == 'exit':
            break
        else:
            print("Unknown command. Please enter 'enable', 'disable', or 'exit'.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
