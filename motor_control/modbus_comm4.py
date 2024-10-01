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

        self.current_velocity = 0  # Keep track of the last set velocity

    def enable_motor(self):
        self._write_register(159)
        self.get_logger().info("Motor enabled successfully.")

    def disable_motor(self):
        self._write_register(158)
        self.get_logger().info("Motor disabled successfully.")

    def start_jog(self):
        self._write_register(150)
        self.get_logger().info("Motor jog started successfully.")

    def stop_jog(self):
        self._write_register(216)
        self.get_logger().info("Motor jog stopped successfully.")

    def set_jog_velocity(self, velocity):
        try:
            # If changing direction, stop motor first
            if (self.current_velocity >= 0 > velocity) or (self.current_velocity < 0 <= velocity):
                self.stop_jog()
                self.get_logger().info("Direction change detected, stopping motor before setting new velocity.")

            # Handle jog velocity with 32-bit long integer format (2 registers)
            address = 40343 - 40001  # Modbus address offset for holding registers
            address = 40344 - 40001
            
            # Correct handling of signed 32-bit integers
            if velocity < 0:
                velocity = (1 << 32) + velocity

            high_word = (velocity >> 16) & 0xFFFF
            low_word = velocity & 0xFFFF

            # Write high and low words to consecutive registers
            result_high = self.client.write_register(address, high_word)
            result_low = self.client.write_register(address + 1, low_word)

            if isinstance(result_high, ExceptionResponse) or isinstance(result_low, ExceptionResponse):
                self.get_logger().error(f"Failed to write jog velocity to registers: {result_high}, {result_low}")
            else:
                self.get_logger().info(f"Jog velocity set to {velocity - (1 << 32) if velocity >= (1 << 31) else velocity} successfully.")
                self.current_velocity = velocity - (1 << 32) if velocity >= (1 << 31) else velocity

            # Restart jog if velocity is non-zero
            if self.current_velocity != 0:
                self.start_jog()

        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")

    def _write_register(self, value):
        try:
            address = 40125 - 40001  # Modbus address offset for holding registers
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
        command = input("Enter command (enable/startjog/stopjog/disable/velocity/exit): ").strip().lower()
        if command == 'enable':
            node.enable_motor()
        elif command == 'disable':
            node.disable_motor()
        elif command == 'startjog':
            node.start_jog()
        elif command == 'stopjog':
            node.stop_jog()
        elif command == 'velocity':
            try:
                velocity = int(input("Enter jog velocity (range -24000 to 24000, in 1/240 rps units): ").strip())
                if -24000 <= velocity <= 24000:
                    node.set_jog_velocity(velocity)
                else:
                    print("Velocity out of range. Please enter a value between -24000 and 24000.")
            except ValueError:
                print("Invalid velocity. Please enter an integer value.")
        elif command == 'exit': 
            break
        else:
            print("Unknown command. Please enter 'enable', 'disable', 'startjog', 'stopjog', 'velocity', or 'exit'.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()