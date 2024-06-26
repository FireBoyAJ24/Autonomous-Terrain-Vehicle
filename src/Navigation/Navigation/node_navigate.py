import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Sensor, CurrentHeading, NewHeading

from std_msgs.msg import String

class NavigationNode(Node):
    """
    This class is a ROS2 node that controls the vehicle.
    
    """
    def __init__(self):
        super().__init__('NavigationNode')
        
        self.__init_attributes()
        self.__init_publishers()
        self.__init_subscriptions()
        self.__init_timer_callbacks()
        self.get_logger().info("Navigation Node is Initialization is complete.")
        

    def __init_attributes(self):
        self.__sensor = Sensor()
        self.__new_heading = NewHeading()
        self.__current_heading = CurrentHeading()

    def __init_subscriptions(self):
        
        self.__sensor = self.create_subscription(
            msg_type=Sensor,
            topic='sonar_sensor',
            callback=self.__sensor_callback,
            qos_profile=1,
        )
        
        self.__current_heading = self.create_subscription(
            msg_type=CurrentHeading,
            topic='current_heading',
            callback=self.__current_heading_callback,
            qos_profile=1,
        )
    
    def __sensor_callback(self, msg: Sensor) -> None:
        self.get_logger().info('Sonar Sensor: %d' % msg.sonar_sensor)
        self.__sensor.sonar_data = msg.sonar_data
    
    def __current_heading_callback(self, msg: CurrentHeading) -> None:
        self.get_logger().info('Current Heading: %d' % msg.current_heading)
        self.__current_heading.current_heading = msg.current_heading
    
    def __init_publishers(self):
        
        self.__new_heading_pub = self.create_publisher(
            msg_type=NewHeading,
            topic='new_heading',
            qos_profile=1,
        )
        

    def __publish(self):
        
        new_heading_msg = NewHeading()
        
        self.__new_heading = self.__get_navigation_values()
        
        new_heading_msg.new_heading = self.__new_heading


        self.__new_heading_pub.publish(new_heading_msg)
        
        self.get_logger().info(f"Publishing new heading: {new_heading_msg.new_heading}")

    ## TODO
    def __get_navigation_values(self):
        return 1.0

    def __init_timer_callbacks(self):
        
        self.get_logger().info("Initializing timer callbacks")
        self.timer = self.create_timer(0.5, self.__publish)
        
        
def main(args=None):
    rclpy.init(args=args)

    navigator = NavigationNode()

    rclpy.spin(navigator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()