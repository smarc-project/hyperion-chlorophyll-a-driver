import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hyperion_chlorophyll_a_interfaces.msg import ChlorophyllData

class ChlorophyllDecoder(Node):
    def __init__(self):
        super().__init__('chlorophyll_decoder')

        self.declare_parameter("input_topic", "chlorophyll_raw")
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value

        self.declare_parameter("output_topic", "chlorophyll_parsed")
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.publisher = self.create_publisher(ChlorophyllData, self.output_topic, 10)
        self.subscriber = self.create_subscription(String, self.input_topic, self.callback, 10)

    def callback(self, data):
        msg = str(data.data)
        msg = msg.strip('\r\n')
        fields = msg.split(',')
        if len(fields) != 11:
            self.get_logger().error(f"something is wrong with data: {len(fields)}")
            return
        sample = {}
        sample['nmea_header'] = fields[0]
        sample['instrumentAddress'] = int(fields[1])
        sample['parameterID'] = int(fields[2])
        sample['val'] = float(fields[3])
        sample['val_sd'] = float(fields[4]) if fields[4] != '' else 0.0
        sample['val_unit'] = fields[5]
        sample['operatingMode'] = fields[9]
        sample['checkSum'] = fields[10]

        msg = ChlorophyllData()
        msg.time = self.get_clock().now().to_msg()
        msg.nmea_header = sample['nmea_header']
        msg.instrument_address = sample['instrumentAddress']
        msg.parameter_id = sample['parameterID']
        msg.val = sample['val']
        msg.val_sd = sample['val_sd']
        msg.val_unit = sample['val_unit']
        msg.operating_mode = sample['operatingMode']
        msg.check_sum = sample['checkSum']

        self.publisher.publish(msg)

        self.get_logger().info(f"Published: {sample}")

def main(args=None):
    rclpy.init(args=args)
    decoder = ChlorophyllDecoder()
    rclpy.spin(decoder)
    decoder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
