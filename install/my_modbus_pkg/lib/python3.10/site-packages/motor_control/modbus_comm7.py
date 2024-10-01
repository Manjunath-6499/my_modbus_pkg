import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException
from pymodbus.pdu import ExceptionResponse
import time

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

        # Jog velocity registers
        self.jog_velocity_register_high = 40343
        self.jog_velocity_register_low = 40344

        # Encoder position registers (assuming registers, replace with actual)
        self.encoder_position_register_high = 40005 # Replace with actual register address
        self.encoder_position_register_low = 40006  # Replace with actual register address

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

    def set_jog_velocity(self, motor_address, velocity):
        if not (-24000 <= velocity <= 24000):
            self.get_logger().error(f"Jog velocity {velocity} is out of range. It must be between -24000 and 24000.")
            return

        # Convert velocity to a 32-bit signed integer
        if velocity < 0:
            velocity = (1 << 32) + velocity

        high_word = (velocity >> 16) & 0xFFFF
        low_word = velocity & 0xFFFF

        # Write the high and low words to the respective Modbus registers
        self._write_register(self.jog_velocity_register_high, high_word, motor_address)
        self._write_register(self.jog_velocity_register_low, low_word, motor_address)

        # Verify if the values were correctly written by reading them back
        written_high = self._read_register(self.jog_velocity_register_high, motor_address)
        written_low = self._read_register(self.jog_velocity_register_low, motor_address)

        if written_high == high_word and written_low == low_word:
            self.get_logger().info(f"Jog velocity for motor {motor_address} set to {velocity} (1/240 rps).")
        else:
            self.get_logger().error(f"Failed to set jog velocity for motor {motor_address}. Written values do not match.")

    def get_encoder_position(self, motor_address):
        high_word = self._read_register(self.encoder_position_register_high, motor_address)
        low_word = self._read_register(self.encoder_position_register_low, motor_address)

        if high_word is None or low_word is None:
            self.get_logger().error(f"Failed to get encoder position for motor {motor_address}")
            return None  # Return None if there's an error

        encoder_position = (high_word << 16) | low_word
        return encoder_position

    def rotate_to_position(self, motor_address, target_position):
        current_position = self.get_encoder_position(motor_address)
        if current_position is None:
            self.get_logger().error(f"Failed to rotate motor {motor_address} to position {target_position}")
            return  # Exit if there's an error

        # Calculate the difference between the target and current position
        position_difference = target_position - current_position
        
        # Set jog velocity and direction based on the position difference
        velocity = self.calculate_velocity_from_position_difference(position_difference)
        self.set_jog_velocity(motor_address, velocity)
        
        # Jog until the target position is reached
        while current_position != target_position:
            current_position = self.get_encoder_position(motor_address)
            if current_position is None:
                break  # Exit loop if there's an error
            time.sleep(0.1)

        self.stop_jog(motor_address)
        self.get_logger().info(f"Motor {motor_address} rotated to position {target_position} successfully.")

    def calculate_velocity_from_position_difference(self, position_difference):
        # This function calculates jog velocity based on position difference
        kP = 100  # Proportional gain (adjust as necessary)
        velocity = kP * position_difference

        # Ensure the velocity is within the allowable range
        if velocity > 24000:
            velocity = 24000
        elif velocity < -24000:
            velocity = -24000

        return velocity

    def _write_register(self, address, value, motor_address):
        try:
            modbus_address = address - 40001  # Modbus address offset for holding registers
            result = self.client.write_register(modbus_address, value, slave=motor_address)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to write to register {address} for motor {motor_address}: {result}")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")

    def _read_register(self, address, motor_address):
        try:
            modbus_address = address - 40001  # Modbus address offset for holding registers
            result = self.client.read_holding_registers(modbus_address, 1, slave=motor_address)
            if isinstance(result, ExceptionResponse) or not hasattr(result, 'registers'):
                self.get_logger().error(f"Failed to read from register {address} for motor {motor_address}: {result}")
                return None  # Return None if there's an error
            return result.registers[0]
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")
            return None  # Return None if there's an exception

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    while rclpy.ok():
        motor_input = input("Enter motor numbersxzzz (1, 2, or bothhh) to control: ").strip().lower()
        if motor_input not in ['1', '2', 'both']:
            print("Invalid motor number. Please enter 1, 2, or both.")
            continue

        motor_addresses = []
        if motor_input == '1':
            motor_addresses = [node.motor_1_address]
        elif motor_input == '2':
            motor_addresses = [node.motor_2_address]
        elif motor_input == 'both':
            motor_addresses = [node.motor_1_address, node.motor_2_address]

        command = input("Enter command (enable/startjog/stopjog/disable/setjogvelocity/rotate/exit): ").strip().lower()
        if command == 'enable':
            for motor_address in motor_addresses:
                node.enable_motor(motor_address)
                time.sleep(0.1)  # Add delay between commands
        elif command == 'disable':
            for motor_address in motor_addresses:
                node.disable_motor(motor_address)
                time.sleep(0.1)
        elif command == 'startjog':
            for motor_address in motor_addresses:
                node.start_jog(motor_address)
                time.sleep(0.1)
        elif command == 'stopjog':
            for motor_address in motor_addresses:
                node.stop_jog(motor_address)
                time.sleep(0.1)
        elif command == 'setjogvelocity':
            try:
                velocity = int(input("Enter jog velocity (-24000 to 24000): ").strip())
                if motor_input == '1' and velocity > 0:
                    velocity = -velocity  # Ensure motor 1 starts in anti-clockwise direction
                for motor_address in motor_addresses:
                    node.set_jog_velocity(motor_address, velocity)
                    time.sleep(0.1)
            except ValueError:
                print("Invalid velocity. Please enter an integer value.")
        elif command == 'rotate':
            try:
                target_position = int(input("Enter target encoder position: ").strip())
                for motor_address in motor_addresses:
                    node.rotate_to_position(motor_address, target_position)
                    time.sleep(0.1)
            except ValueError:
                print("Invalid position. Please enter an integer value.")
        elif command == 'exit':
            break
        else:
            print("Invalid command. Please enter a valid command.")

    node.client.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
