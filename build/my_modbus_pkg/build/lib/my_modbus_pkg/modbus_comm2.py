import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException
from pymodbus.pdu import ExceptionResponse
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian

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

    def enable_motor(self):
        self._write_register(40125, 159)
        self.get_logger().info("Motor enabled successfully.")

    def disable_motor(self):
        self._write_register(40125, 158)
        self.get_logger().info("Motor disabled successfully.")

    def start_jog(self):
        self._write_register(40125, 150)
        self.get_logger().info("Motor jog started successfully.")

    def stop_jog(self):
        self._write_register(40125, 216)
        self.get_logger().info("Motor jog stopped successfully.")

    def set_jog_velocity(self, velocity):
        # Check if the velocity is within the allowed range
        if velocity < -24000 or velocity > 24000:
            self.get_logger().error("Jog velocity out of range. Must be between -24000 and 24000.")
            return
        
        # Write the 32-bit long integer value to the Modbus registers
        try:
            builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)
            builder.add_32bit_int(velocity)
            payload = builder.to_registers()
            
            # No 'unit' argument needed for write_registers
            result = self.client.write_registers(40343, payload)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to write to registers 40343 and 40344: {result}")
            else:
                self.get_logger().info(f"Jog velocity set to {velocity}.")

        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")

    def _write_register(self, address, value):
        try:
            modbus_address = address - 40001  # Modbus address offset for holding registers
            result = self.client.write_register(modbus_address, value)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to write to register {address}: {result}")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    while rclpy.ok():
        command = input("Enter command (enable/startj/stopjog/disable/setvelocityyyyyy/exit): ").strip().lower()
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
                velocity = int(input("Enter jog velocity (-24000 to 24000): ").strip())
                node.set_jog_velocity(velocity)
            except ValueError:
                print("Invalid input. Please enter an integer value for velocity.")
        elif command == 'exit':
            break
        else:
            print("Unknown command. Please enter 'enable', 'disable', 'startjog', 'stopjog', 'setvelocity', or 'exit'.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
