import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Modbus client setup (remove the 'method' argument)
        self.client = ModbusClient(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
        self.client.connect()

        # Modbus addresses for motors
        self.motor_1_address = 1
        self.motor_2_address = 2

        # Register address for enabling/disabling the motor
        self.enable_disable_register = 40125  # Assuming this is the correct register

        # Timer to repeatedly call the enable/disable function
        self.timer = self.create_timer(5.0, self.control_motors)

    def control_motors(self):
        # Example: Enable motor 1 and disable motor 2
        self.enable_motor(self.motor_1_address)
        self.disable_motor(self.motor_2_address)

    def enable_motor(self, motor_address):
        self.get_logger().info(f'Enabling motor {motor_address}')
        self.client.write_register(self.enable_disable_register, 159, unit=motor_address)

    def disable_motor(self, motor_address):
        self.get_logger().info(f'Disabling motor {motor_address}')
        self.client.write_register(self.enable_disable_register, 158, unit=motor_address)

    def destroy_node(self):
        self.client.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
