import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

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
        while True:
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().decode('utf-8').strip()
                callback(data)

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
        self.thread = threading.Thread(target=self.driver.read_data_continuous, args=(self.publish_data,))
        self.thread.daemon = True
        self.thread.start()
        self.rate = rclpy.Rate(self.publish_rate)

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