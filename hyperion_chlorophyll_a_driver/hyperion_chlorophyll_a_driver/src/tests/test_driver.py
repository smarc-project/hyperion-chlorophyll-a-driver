import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from msg import ChlorophyllData

class TestHyperionChlorophyllDriver(Node):
    def __init__(self):
        super().__init__('test_driver')
        # Create a client for the reset service
        self.cli = self.create_client(Trigger, '/fluorescein/reset')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()
        self.sensor_name = "Hyperion Chlorophyll a"
        
        # Subscribe to the raw data topic
        self.subscription = self.create_subscription(
            String,
            'chlorophyll/raw_data',
            self.raw_data_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Subscribe to the processed data topic
        self.subscription_processed = self.create_subscription(
            ChlorophyllData,
            'chlorophyll/processed_data',
            self.processed_data_callback,
            10
        )
        self.subscription_processed  # prevent unused variable warning
        
        self.raw_data_received = False
        self.processed_data_received = False

    def send_request(self):
        # Send a reset service request
        self.future = self.cli.call_async(self.req)

    def raw_data_callback(self, msg):
        # Callback for raw data topic
        self.get_logger().info(f'Received raw data: {msg.data}')
        self.raw_data_received = True

    def processed_data_callback(self, msg):
        # Callback for processed data topic
        self.get_logger().info(f'Received processed data: {msg}')
        self.processed_data_received = True

def main(args=None):
    rclpy.init(args=args)
    test_driver = TestHyperionChlorophyllDriver()
    test_driver.send_request()

    while rclpy.ok():
        rclpy.spin_once(test_driver)
        if test_driver.future.done():
            try:
                response = test_driver.future.result()
            except Exception as e:
                test_driver.get_logger().info(f'Service call failed: {e}')
            else:
                if response.success:
                    test_driver.get_logger().info('Driver is up and running.')
                else:
                    test_driver.get_logger().warn('Driver reset service call failed.')
            break

    # Wait until both raw and processed data are received
    while not test_driver.raw_data_received or not test_driver.processed_data_received:
        rclpy.spin_once(test_driver)

    test_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
