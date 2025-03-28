import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from msg import ChlorophyllData

class ChlorophyllDecoder(Node):
    def __init__(self, input_topic, output_topic):
        super().__init__('chlorophyll_decoder')
        self.publisher = self.create_publisher(ChlorophyllData, output_topic, 10)
        self.subscriber = self.create_subscription(String, input_topic, self.callback, 10)

    def callback(self, data):
        msg = str(data.data)
        msg = msg.strip('\r\n')
        fields = msg.split(',')
        if len(fields) != 11:
            self.get_logger().error(f"something is wrong with data: {len(fields)}")
            return
        sample = {}
        sample['NMEA_header'] = fields[0]
        sample['instrumentAddress'] = int(fields[1])
        sample['parameterID'] = int(fields[2])
        sample['val'] = float(fields[3])
        sample['val_sd'] = float(fields[4]) if fields[4] != '' else 0
        sample['val_unit'] = fields[5]
        sample['operatingMode'] = fields[9]
        sample['checkSum'] = fields[10]

        msg = ChlorophyllData()
        msg.time = self.get_clock().now().to_msg()
        msg.NMEA_header = sample['NMEA_header']
        msg.instrumentAddress = sample['instrumentAddress']
        msg.parameterID = sample['parameterID']
        msg.val = sample['val']
        msg.val_sd = sample['val_sd']
        msg.val_unit = sample['val_unit']
        msg.operatingMode = sample['operatingMode']
        msg.checkSum = sample['checkSum']

        self.publisher.publish(msg)

        self.get_logger().info(f"Published: {sample}")

def main(args=None):
    rclpy.init(args=args)
    decoder = ChlorophyllDecoder('/chlorophyll/raw_data', '/chlorophyll/processed_data')
    rclpy.spin(decoder)
    decoder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
