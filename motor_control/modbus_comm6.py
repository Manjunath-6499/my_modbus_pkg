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

        # Motor addresses
        self.motor_1_address = 1
        self.motor_2_address = 2

    def enable_motor(self, motor_address):
        self._write_register(40125, 159, motor_address)
        self.get_logger().info(f"Motor {motor_address} enabled successfully.")

    def disable_motor(self, motor_address):
        self._write_register(40125, 158, motor_address)
        self.get_logger().info(f"Motor {motor_address} disabled successfully.")

    def start_jog(self, motor_address):
        self._write_register(40125, 150, motor_address)
        self.get_logger().info(f"Motor {motor_address} jog started successfully.")

    def stop_jog(self, motor_address):
        self._write_register(40125, 216, motor_address)
        self.get_logger().info(f"Motor {motor_address} jog stopped successfully.")

    def _write_register(self, address, value, motor_address):
        try:
            modbus_address = address - 40001  # Modbus address offset for holding registers
            result = self.client.write_register(modbus_address, value, slave=motor_address)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to write to register {address} for motor {motor_address}: {result}")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    while rclpy.ok():
        motor_input = input("Enter motor number (1 or 2) to control: ").strip()
        if motor_input not in ['1', '2']:
            print("Invalid motor number. Please enter 1 or 2.")
            continue

        motor_address = int(motor_input)

        command = input("Enter command (enable/startjog/stopjog/disable/exit): ").strip().lower()
        if command == 'enable':
            node.enable_motor(motor_address)
        elif command == 'disable':
            node.disable_motor(motor_address)
        elif command == 'startjog':
            node.start_jog(motor_address)
        elif command == 'stopjog':
            node.stop_jog(motor_address)
        elif command == 'exit':
            break
        else:
            print("Unknown command. Please enter 'enable', 'disable', 'startjog', 'stopjog', or 'exit'.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
