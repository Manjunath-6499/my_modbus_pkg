import rclpy #importing ROS2 python library
from rclpy.node import Node #import Node class from rclpy which is the basic class of creating ROS library 
from pymodbus.client import ModbusSerialClient as ModbusClient #import the modbusSerialClient class from pymodbus library for modbus RTU Communication over the serial
from pymodbus.exceptions import ModbusException #import the ModbusException to handle modbus related exceptions 
from pymodbus.pdu import ExceptionResponse #import the exception responses indicating modbus errors 

class MOdbusNode(Node):
    def __init__(self):
        super().__init__("modbus_node")
        self.declare_parameter('port', 'dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 1)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value_value

        self.client = ModbusClient(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity='N',
            stopbits=1,
            bytesize=8
        )

        if not self.client.connect():
            self.get_logger().error('Failed to connect to the modbus device...')
            return
        

        #Motor addresses
        self.motor_1_address = 1
        self.motor_2_address = 2

        #Jog velocity registers
        self.jog_velocity_register_high = 40343
        self.jog_velocity_register_low = 40344


        def enable_motor(self, motor_address):
            self._write_register(40125, 159, motor_address)
            self.get_logger().info(f"Motor {motor_address} enabled successfully... ")

