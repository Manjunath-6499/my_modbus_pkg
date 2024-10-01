import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException
from pymodbus.pdu import ExceptionResponse
from struct import pack

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
        self._write_register(159, motor_id)
        self.get_logger().info(f"Motor {motor_id} enabled successfully.")

    def disable_motor(self, motor_id):
        self._write_register(158, motor_id)
        self.get_logger().info(f"Motor {motor_id} disabled successfully.")

    def start_jog(self, motor_id):
        self._write_register(150, motor_id)
        self.get_logger().info(f"Motor {motor_id} jog started successfully.")

    def stop_jog(self, motor_id):
        self._write_register(216, motor_id)
        self.get_logger().info(f"Motor {motor_id} jog stopped successfully.")

    def set_jog_velocity(self, motor_id, velocity):
        if not (-24000 <= velocity <= 24000):
            self.get_logger().error("Jog velocity out of range (-24000 to 24000).")
            return

        # Convert the velocity to 32-bit long integer format
        velocity_bytes = pack('>l', velocity)  # '>l' for big-endian 32-bit signed integer
        high_word = int.from_bytes(velocity_bytes[:2], byteorder='big', signed=False)
        low_word = int.from_bytes(velocity_bytes[2:], byteorder='big', signed=False)

        try:
            # Write to registers for the specified motor
            base_address = 40343 if motor_id == 1 else 40345
            address_high = base_address - 40001
            address_low = (base_address + 1) - 40001

            result_high = self.client.write_register(address_high, high_word)
            result_low = self.client.write_register(address_low, low_word)

            if isinstance(result_high, ExceptionResponse) or isinstance(result_low, ExceptionResponse):
                self.get_logger().error(f"Failed to write to registers for motor {motor_id}: {result_high} {result_low}")
            else:
                self.get_logger().info(f"Jog velocity for motor {motor_id} set to {velocity} successfully.")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception for motor {motor_id}: {e}")
        finally:
            self.client.close()

    def _write_register(self, value, motor_id):
        try:
            # Assuming that the enable motor command is written to register 40125
            base_address = 40125 if motor_id == 1 else 40127
            address = base_address - 40001

            result = self.client.write_register(address, value)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to write to register for motor {motor_id}: {result}")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception for motor {motor_id}: {e}")
        finally:
            self.client.close()

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    while rclpy.ok():
        command = input("Enter command (enable/startjog/stopjog/setvelocity/disable/exit): ").strip().lower()
        if command in ['enable', 'disable', 'startjog', 'stopjog', 'setvelocity']:
            try:
                motor_id = int(input("Enter motor ID (1 or 2): ").strip())
                if motor_id not in [1, 2]:
                    print("Invalid motor ID. Please enter 1 or 2.")
                    continue

                if command == 'enable':
                    node.enable_motor(motor_id)
                elif command == 'disable':
                    node.disable_motor(motor_id)
                elif command == 'startjog':
                    node.start_jog(motor_id)
                elif command == 'stopjog':
                    node.stop_jog(motor_id)
                elif command == 'setvelocity':
                    try:
                        velocity = int(input(f"Enter jog velocity for motor {motor_id} (-24000 to 24000): ").strip())
                        node.set_jog_velocity(motor_id, velocity)
                    except ValueError:
                        print("Invalid input. Please enter an integer value for velocity.")
            except ValueError:
                print("Invalid input. Please enter an integer value for motor ID.")
        elif command == 'exit':
            break
        else:
            print("Unknown command. Please enter 'enable', 'disable', 'startjog', 'stopjog', 'setvelocity', or 'exit'.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
