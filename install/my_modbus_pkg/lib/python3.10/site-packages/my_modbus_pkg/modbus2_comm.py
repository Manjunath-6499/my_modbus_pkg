import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException
from pymodbus.pdu import ExceptionResponse
import struct

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
            self.client = None

    def enable_motor(self):
        if self.client:
            self._write_register(40125, 158)
            self.get_logger().info("Motor enabled successfully.")
        else:
            self.get_logger().error("Modbus client is not connected.")

    def disable_motor(self):
        if self.client:
            self._write_register(40125, 0)
            self.get_logger().info("Motor disabled successfully.")
        else:
            self.get_logger().error("Modbus client is not connected.")

    def start_jog(self):
        if self.client:
            self._write_register(40125, 159)
            self.get_logger().info("Motor jog started successfully.")
        else:
            self.get_logger().error("Modbus client is not connected.")

    def stop_jog(self):
        if self.client:
            self._write_register(40125, 160)
            self.get_logger().info("Motor jog stopped successfully.")
        else:
            self.get_logger().error("Modbus client is not connected.")

    def set_jog_velocity(self, velocity):
        if self.client:
            # Convert the integer to a 32-bit long integer
            long_value = int(velocity)
            high = (long_value >> 16) & 0xFFFF
            low = long_value & 0xFFFF

            # Write high and low parts to consecutive registers
            try:
                address = 40343 - 40001
                result_high = self.client.write_register(address, high)
                result_low = self.client.write_register(address + 1, low)
                
                if isinstance(result_high, ExceptionResponse) or isinstance(result_low, ExceptionResponse):
                    self.get_logger().error(f"Failed to write to register: High - {result_high}, Low - {result_low}")
                else:
                    self.get_logger().info(f"Jog velocity set to {velocity} successfully.")
            except ModbusException as e:
                self.get_logger().error(f"Modbus Exception: {e}")
        else:
            self.get_logger().error("Modbus client is not connected.")

    def _write_register(self, base_address, value):
        try:
            address = base_address - 40001  # Modbus address offset for holding registers
            result = self.client.write_register(address, value)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to write to register: {result}")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    while rclpy.ok():
        command = input("Enter command (enable/startjog/stopjog/disable/setvelocity/exit): ").strip().lower()
        if command == 'enable':
            node.enable_motor()
        elif command == 'disable':
            node.disable_motor()
        elif command == 'startjog':
            node.start_jog()
        elif command == 'stopjog':
            node.stop_jog()
        elif command == 'setvelocity':
            try:
                velocity = int(input("Enter jog velocity (as a 32-bit integer): ").strip())
                node.set_jog_velocity(velocity)
            except ValueError:
                print("Invalid velocity value. Please enter a number.")
        elif command == 'exit':
            break
        else:
            print("Unknown command. Please enter 'enable', 'disable', 'startjog', 'stopjog', 'setvelocity', or 'exit'.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

