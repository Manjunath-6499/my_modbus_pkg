import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException
from pymodbus.pdu import ExceptionResponse
import time
import struct

class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 10)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Initialize Modbus client with given parameters
        self.client = ModbusClient(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity='N',  # Default, modify if needed
            stopbits=1,
            bytesize=8
        )

        # Attempt to connect to the Modbus device
        if not self.client.connect():
            self.get_logger().error('Failed to connect to the Modbus device.')
            return

        # Motor addresses
        self.motor_1_address = 1
        self.motor_2_address = 2

        # Command register addresses
        self.command_register = 40125
        self.jog_velocity_high_register = 40343
        self.jog_velocity_low_register = 40344
        self.encoder_position_high_register = 40009 # MSB for encoder position
        self.encoder_position_low_register = 40007  # LSB for encoder position

    def enable_motor(self, motor_address):
        self._write_register(self.command_register, 159, motor_address)
        self.get_logger().info(f"Motor {motor_address} enabled successfully.")

    def disable_motor(self, motor_address):
        self._write_register(self.command_register, 158, motor_address)
        self.get_logger().info(f"Motor {motor_address} disabled successfully.")

    def start_jog(self, motor_address):
        self._write_register(self.command_register, 150, motor_address)
        self.get_logger().info(f"Motor {motor_address} jog started successfully.")
        
    def stop_jog(self, motor_address):
        self._write_register(self.command_register, 216, motor_address)
        self.get_logger().info(f"Motor {motor_address} jog stopped successfully.")

    def set_jog_velocity(self, motor_address, velocity):
        if not (-24000 <= velocity <= 24000):
            self.get_logger().error(f"Invalid jog velocity: {velocity}. Must be between -24000 and 24000.")
            return

        # Convert the signed 32-bit velocity to two 16-bit values (high and low words)
        packed_value = struct.pack('>i', velocity)  # '>i' for big-endian signed int
        high_word, low_word = struct.unpack('>HH', packed_value)

        # Send the high and low words to respective registers
        self.get_logger().debug(f"Setting jog velocity {velocity} (high word: {high_word}, low word: {low_word}) "
                                f"for motor {motor_address}.")
        self.client.write_registers(self.jog_velocity_high_register - 40001, [high_word, low_word], slave=motor_address)
        self.get_logger().info(f"Jog velocity for motor {motor_address} set to {velocity}.")
        time.sleep(0.5)  # Delay to ensure the drive processes the velocity command

    def read_encoder_position(self, motor_address):
        """Read the 32-bit encoder position from the motor controller."""
        try:
            for attempt in range(10):  # Retry logic with a maximum of 3 attempts
                self.get_logger().debug(f"Attempt {attempt + 1}: Reading encoder position for motor {motor_address}.")

                # Read the high and low words (16-bit registers) for the encoder position
                result = self._read_register(self.encoder_position_high_register, 2, motor_address)

                if result is None:
                    self.get_logger().error(f"Failed to read encoder registers for motor {motor_address}. Retrying...")
                    time.sleep(1)  # Wait before retrying
                    continue

                # Combine the two 16-bit registers into a 32-bit integer
                high_word = result[0]
                low_word = result[1]
                encoder_position = (high_word << 16) | low_word

                self.get_logger().info(f"Encoder position for motor {motor_address}: {encoder_position} pulses.")
                return encoder_position

            self.get_logger().error(f"Failed to read encoder position for motor {motor_address} after 3 attempts.")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")
        except Exception as e:
            self.get_logger().error(f"General Exception: {e}")
        return None

    # Helper method to read registers
    def _read_register(self, address, count, motor_address):
        try:
            modbus_address = address - 40001  # Convert to zero-based Modbus register address
            self.get_logger().debug(f"Reading {count} registers starting at Modbus address {modbus_address} "
                                    f"(register {address}) for motor {motor_address}.")

            # Perform the read operation
            result = self.client.read_holding_registers(modbus_address, count, slave=motor_address)

            if result.isError():
                self.get_logger().error(f"Modbus Error while reading register {address} for motor {motor_address}: {result}")
                return None

            self.get_logger().debug(f"Registers read successfully: {result.registers}")
            return result.registers
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")
        except Exception as e:
            self.get_logger().error(f"General Exception: {e}")
        return None

    def _write_register(self, address, value, motor_address):
        try:
            modbus_address = address - 40001  # Modbus register address
            self.get_logger().debug(f"Writing value {value} to Modbus address {modbus_address} "
                                    f"(register {address}) for motor {motor_address}.")
            result = self.client.write_register(modbus_address, value, slave=motor_address)
            if isinstance(result, ExceptionResponse):
                self.get_logger().error(f"Failed to write to register {address} for motor {motor_address}: {result}")
        except ModbusException as e:
            self.get_logger().error(f"Modbus Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    while rclpy.ok():
        motor_input = input("Enter motor number (1, 2, or both): ").strip().lower()
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

        command = input("Enter command (enable/startjog/stopjog/setvelocity/encoderposition/disable/exit): ").strip().lower()
        if command == 'enable':
            for motor_address in motor_addresses:
                node.enable_motor(motor_address)
                time.sleep(0.1)
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
        elif command == 'setvelocity':
            velocity = int(input("Enter jog velocity (-24000 to 24000): ").strip())
            for motor_address in motor_addresses:
                node.set_jog_velocity(motor_address, velocity)
                time.sleep(0.1)
        elif command == 'encoderposition':
            for motor_address in motor_addresses:
                encoder_position = node.read_encoder_position(motor_address)
                if encoder_position is not None:
                    print(f"Motor {motor_address} encoder position: {encoder_position} pulses.")
                time.sleep(0.1)
        elif command == 'exit':
            break
        else:
            print("Invalid command.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
