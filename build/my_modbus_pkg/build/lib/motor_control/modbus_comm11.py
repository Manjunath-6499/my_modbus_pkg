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

        # Current, Torque, and RPM registers
        self.current_register = 40013
        self.torque_register = 40014
        self.rpm_register = 40015

        # Feed to length command
        self.feed_to_length_command = 102

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
        self.get_logger().info(f"Feed to length command executed for motor {motor_address}.")

    def get_encoder_position(self, motor_address):
        try:
            result = self.client.read_holding_registers(self.encoder_high_register, 2, slave=motor_address)
            if result.isError():
                raise ValueError(f"Modbus error: {result}")
            encoder_value = self.decode_long_integer(result.registers)
            self.get_logger().info(f"Motor {motor_address} encoder position: {encoder_value}")
        except Exception as e:
            self.get_logger().error(f"Error reading encoder position for motor {motor_address}: {e}")
            time.sleep(0.5)  # Retry after a short delay
        try:
            result = self.client.read_holding_registers(self.encoder_high_register, 2, slave=motor_address)
            if not result.isError():
                encoder_value = self.decode_long_integer(result.registers)
                self.get_logger().info(f"Motor {motor_address} encoder position (retry): {encoder_value}")
            else:
                self.get_logger().error(f"Retry failed for motor {motor_address}")
        except Exception as retry_e:
            self.get_logger().error(f"Retry error for motor {motor_address}: {retry_e}")

    def get_current(self, motor_address):
        try:
            result = self.client.read_holding_registers(self.current_register, 1, slave=motor_address)
            if result.isError():
                raise ValueError(f"Modbus error: {result}")
            current = result.registers[0]
            self.get_logger().info(f"Motor {motor_address} current: {current} A")
        except Exception as e:
            self.get_logger().error(f"Error reading current for motor {motor_address}: {e}")

    def get_torque(self, motor_address):
        try:
            result = self.client.read_holding_registers(self.torque_register, 1, slave=motor_address)
            if result.isError():
                raise ValueError(f"Modbus error: {result}")
            torque = result.registers[0]
            self.get_logger().info(f"Motor {motor_address} torque: {torque} Nm")
        except Exception as e:
            self.get_logger().error(f"Error reading torque for motor {motor_address}: {e}")

    def get_rpm(self, motor_address):
        try:
            result = self.client.read_holding_registers(self.rpm_register, 1, slave=motor_address)
            if result.isError():
                raise ValueError(f"Modbus error: {result}")
            rpm = result.registers[0]
            self.get_logger().info(f"Motor {motor_address} RPM: {rpm}")
        except Exception as e:
            self.get_logger().error(f"Error reading RPM for motor {motor_address}: {e}")

    def _write_register(self, register, value, motor_address):
        try:
            result = self.client.write_register(register - 40001, value, slave=motor_address)
            if result.isError():
                raise ValueError(f"Modbus error: {result}")
        except Exception as e:
            self.get_logger().error(f"Error writing register {register} for motor {motor_address}: {e}")

    def decode_long_integer(self, registers):
        if len(registers) != 2:
            raise ValueError(f"Invalid number of registers: {len(registers)}")
        return (registers[0] << 16) | registers[1]

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()

    # Example usage
    node.enable_motor(node.motor_1_address)
    node.set_jog_velocity(node.motor_1_address, 1000)
    node.get_encoder_position(node.motor_1_address)
    node.get_current(node.motor_1_address)
    node.get_torque(node.motor_1_address)
    node.get_rpm(node.motor_1_address)

    rclpy.spin(node)

    # Shutdown
    node.client.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
