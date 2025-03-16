import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time  # Add this import

class HyperionChlorophyllDriver:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.sensor_name = "Hyperion Chlorophyll a"
        self.serial_connection = serial.Serial(port, baudrate)

    def read_data(self):
        if self.serial_connection.in_waiting > 0:
            data = self.serial_connection.readline().decode('utf-8').strip()
            return data
        return None

    def read_data_continuous(self, callback): 
        
        ''''
        This function reads data continuously from the serial port and calls the callback function with the data as an argument.
        Deprecated in favor of faster and more efficient timer-based reading. 
        '''
        while True:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode('utf-8').strip()
                if data:  # Check if data is not empty
                    callback(data)
            time.sleep(0.1)  # Add a small delay

    def close_connection(self):
        self.serial_connection.close()

class HyperionChlorophyllNode(Node):
    def __init__(self):
        super().__init__('hyperion_chlorophyll_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 10)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        self.driver = HyperionChlorophyllDriver(port, baudrate)
        self.publisher_ = self.create_publisher(String, 'chlorophyll/raw_data', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)  # Add a timer
        self.rate = rclpy.Rate(self.publish_rate)

    def timer_callback(self):
        raw_data = self.driver.read_data()
        if raw_data:
            msg = String()
            msg.data = raw_data
            self.publisher_.publish(msg)

    def publish_data(self, raw_data):
        msg = String()
        msg.data = raw_data
        self.publisher_.publish(msg)
        self.rate.sleep()

    def destroy_node(self):
        self.driver.close_connection()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HyperionChlorophyllNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()