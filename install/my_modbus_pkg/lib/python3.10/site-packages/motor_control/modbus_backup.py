import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusException
import struct
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

        # Command register 
        self.command_register = 40125
        # Jog velocity registers
        self.jog_velocity_high_register = 40343
        self.jog_velocity_low_register = 40344

        # Acceleration registers
        self.acceleration_high_register = 40345
        self.acceleration_low_register = 40346

        # Deceleration registers
        self.deceleration_high_register = 40347
        self.deceleration_low_register = 40348

        # Velocity registers
        self.velocity_high_register = 40349
        self.velocity_low_register = 40350

        # Distance registers
        self.distance_high_register = 40351
        self.distance_low_register = 40352

        # Encoder registers
        self.encoder_high_register = 40011
        self.encoder_low_register = 40012

        # Feed to length command
        self.feed_to_length_command = 102

    def enable_motor(self, motor_address):
        self._write_register(self.command_register, 159, motor_address)

    def disable_motor(self, motor_address):
        self._write_register(self.command_register, 158, motor_address)

    def start_jog(self, motor_address):
        self._write_register(self.command_register, 150, motor_address)

    def stop_jog(self, motor_address):
        self._write_register(self.command_register, 216, motor_address)

    def set_jog_velocity(self, motor_address, velocity):
        if not (-24000 <= velocity <= 24000):
            self.get_logger().error(f"Invalid jog velocity: {velocity}. Must be between -24000 and 24000.")
            return

        packed_value = struct.pack('>i', velocity)
        high_word, low_word = struct.unpack('>HH', packed_value)

        self.client.write_registers(self.jog_velocity_high_register - 40001, [high_word, low_word], slave=motor_address)
        self.get_logger().info(f"Jog velocity for motor {motor_address} set to {velocity}.")

    def set_acceleration(self, motor_address, acceleration):
        acceleration_in_units = int(acceleration * 240)

        if not (1 <= acceleration_in_units <= 30000):
            self.get_logger().error(f"Invalid acceleration: {acceleration_in_units}. Must be between 1 and 30000.")
            return

        packed_value = struct.pack('>i', acceleration_in_units)
        high_word, low_word = struct.unpack('>HH', packed_value)

        self.client.write_registers(self.acceleration_high_register - 40001, [high_word, low_word], slave=motor_address)
        self.get_logger().info(f"Acceleration for motor {motor_address} set to {acceleration} (in rps²).")

    def set_deceleration(self, motor_address, deceleration):
        deceleration_in_units = int(deceleration * 240)

        if not (1 <= deceleration_in_units <= 30000):
            self.get_logger().error(f"Invalid deceleration: {deceleration_in_units}. Must be between 1 and 30000.")
            return

        packed_value = struct.pack('>i', deceleration_in_units)
        high_word, low_word = struct.unpack('>HH', packed_value)

        self.client.write_registers(self.deceleration_high_register - 40001, [high_word, low_word], slave=motor_address)
        self.get_logger().info(f"Deceleration for motor {motor_address} set to {deceleration} (in rps²).")

    def set_velocity(self, motor_address, velocity_rps):
        velocity_rpm = velocity_rps * 60

        if not (0 <= velocity_rpm <= 24000):
            self.get_logger().error(f"Invalid velocity: {velocity_rpm} RPM. Must be between 0 and 24000 RPM.")
            return

        velocity_in_units = int((velocity_rpm / 60) * 240)

        packed_value = struct.pack('>i', velocity_in_units)
        high_word, low_word = struct.unpack('>HH', packed_value)

        self.client.write_registers(self.velocity_high_register - 40001, [high_word, low_word], slave=motor_address)
        self.get_logger().info(f"Velocity for motor {motor_address} set to {velocity_rps} rps.")

    def set_distance(self, motor_address, distance_degrees):
        distance = int((distance_degrees / 360.0) * 10000)

        if not (-10000 <= distance <= 10000):
            self.get_logger().error(f"Invalid distance: {distance_degrees} degrees.")
            return

        packed_value = struct.pack('>i', distance)
        high_word, low_word = struct.unpack('>HH', packed_value)

        self.client.write_registers(self.distance_high_register - 40001, [high_word, low_word], slave=motor_address)
        self.get_logger().info(f"Distance for motor {motor_address} set to {distance_degrees} degrees.")

    def feed_to_length(self, motor_address):
        self._write_register(self.command_register, self.feed_to_length_command, motor_address)

    def get_encoder_position(self, motor_address):
        try:
            result = self.client.read_holding_registers(self.encoder_high_register - 40001, 2, slave=motor_address)
            if result.isError():
                raise ValueError(f"Modbus error: {result}")
            
            encoder_value = self.decode_long_integer(result.registers)
            self.get_logger().info(f"Motor {motor_address} encoder position: {encoder_value}")

        except Exception as e:
            self.get_logger().error(f"Error reading encoder position for motor {motor_address}: {e}")
            time.sleep(1)  # Retry after a longer delay

        try:
            result = self.client.read_holding_registers(self.encoder_high_register - 40001, 2, slave=motor_address)
            if not result.isError():
                encoder_value = self.decode_long_integer(result.registers)
                self.get_logger().info(f"Motor {motor_address} encoder position (retry): {encoder_value}")
            else:
                self.get_logger().error(f"Retry failed for motor {motor_address}")
                
        except Exception as retry_e:
            self.get_logger().error(f"Retry error for motor {motor_address}: {retry_e}")

    def _write_register(self, address, value, motor_address):
        try:
            modbus_address = address - 40001
            self.client.write_register(modbus_address, value, slave=motor_address)
            self.get_logger().info(f"Wrote value {value} to register {address} for motor {motor_address}")
        except ModbusException as e:
            self.get_logger().error(f"Failed to write value {value} to register {address} for motor {motor_address}: {e}")

    def decode_long_integer(self, registers):
        return struct.unpack('>i', struct.pack('>HH', registers[0], registers[1]))[0]

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    motor_addresses = []
    while True:
        if not motor_addresses:
            motor_choice = input("Select motor (1, 2, both): ").strip().lower()

            if motor_choice == '1':
                motor_addresses = [node.motor_1_address]
            elif motor_choice == '2':
                motor_addresses = [node.motor_2_address]
            elif motor_choice == 'both':
                motor_addresses = [node.motor_1_address, node.motor_2_address]

        command = input("Enter command (enable, disable, jog, stop, velocity, acceleration, deceleration, distance, feed, encoder, change motor, exit): ").strip().lower()

        if command == 'enable':
            for addr in motor_addresses:
                node.enable_motor(addr)
                time.sleep(0.1)

        elif command == 'disable':
            for addr in motor_addresses:
                node.disable_motor(addr)
                time.sleep(0.1)

        elif command == 'jog':
            for addr in motor_addresses:
                node.start_jog(addr)
                time.sleep(0.1)

        elif command == 'stop':
            for addr in motor_addresses:
                node.stop_jog(addr)
                time.sleep(0.1)

        elif command == 'velocity':
            velocity = int(input("Enter velocity in rps (positive for forward, negative for reverse): "))
            for addr in motor_addresses:
                node.set_jog_velocity(addr, velocity)
                time.sleep(0.1)

        elif command == 'acceleration':
            acceleration = float(input("Enter acceleration in rps^2: "))
            for addr in motor_addresses:
                node.set_acceleration(addr, acceleration)
                time.sleep(0.1)

        elif command == 'deceleration':
            deceleration = float(input("Enter deceleration in rps^2: "))
            for addr in motor_addresses:
                node.set_deceleration(addr, deceleration)
                time.sleep(0.1)

        elif command == 'distance':
            distance = float(input("Enter distance in degrees: "))
            for addr in motor_addresses:
                node.set_distance(addr, distance)
                time.sleep(0.1)

        elif command == 'feed':
            for addr in motor_addresses:
                node.feed_to_length(addr)
                time.sleep(0.1)

        elif command == 'encoder':
            for addr in motor_addresses:
                node.get_encoder_position(addr)
                time.sleep(0.1)

        elif command == 'change motor':
            motor_addresses = []
            continue

        elif command == 'exit':
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
