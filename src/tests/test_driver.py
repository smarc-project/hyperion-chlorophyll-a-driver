import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class TestHyperionChlorophyllDriver(Node):
    def __init__(self):
        super().__init__('test_driver')
        self.cli = self.create_client(Trigger, '/fluorescein/reset')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()
        self.sensor_name = "Hyperion Chlorophyll a"

    def send_request(self):
        self.future = self.cli.call_async(self.req)

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

    test_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
