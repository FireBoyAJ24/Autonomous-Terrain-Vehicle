import rclpy
from rclpy.node import Node
from custom_interfaces.msg import MotorPower, Steering, CurrentHeading, NewHeading

from std_msgs.msg import String


class VehicleControllerNode(Node):
    """
    This class is a ROS2 node that controls the vehicle.
    
    """
    def __init__(self):
        super().__init__('VehicleControllerNode')
        
        self.__init_attributes()
        self.__init_publishers()
        self.__init_subscriptions()
        self.__init_timer_callbacks()
        self.get_logger().info("Vehicle Controller Node is Initialization is complete.")
        

    def __init_attributes(self):
        self.__motor_power = MotorPower()
        self.__steering_angle = Steering()
        self.__current_heading = CurrentHeading()

    def __init_subscriptions(self):
        
        self.__current_heading_sub = self.create_subscription(
            msg_type=CurrentHeading,
            topic='current_heading',
            callback=self.__current_heading_callback,
            qos_profile=1,
        )
    
    def __current_heading_callback(self, msg: CurrentHeading) -> None:
        self.get_logger().info('New Heading: %d' % msg.current_heading)
        self.__current_heading.current_heading = msg.current_heading
    
    def __init_publishers(self):
        
        self.__motor_power_pub = self.create_publisher(
            msg_type=MotorPower,
            topic='motor_power',
            qos_profile=1,
        )
        
        self.__steering_angle_pub = self.create_publisher(
            msg_type=Steering,
            topic='steering_angle',
            qos_profile=1,
        )

    def __publish(self):
        
        motor_power_msg = MotorPower()
        steering_angle_msg = Steering()
        
        self.__motor_power, self.__steering_angle = self.__get_control_values()
        
        motor_power_msg.power = self.__motor_power
        steering_angle_msg.steering = self.__steering_angle

        self.__motor_power_pub.publish(motor_power_msg)
        self.__steering_angle_pub.publish(steering_angle_msg)
        
        self.get_logger().info(f"Publishing Motor Power and Steering Angle: {motor_power_msg.power}, {steering_angle_msg.steering}")

    ##TODO
    def __get_control_values(self):
        
        return 0.0, 0.0

    def __init_timer_callbacks(self):
        
        self.get_logger().info("Initializing timer callbacks")
        self.timer = self.create_timer(0.5, self.__publish)

class Controller:
    def __init__(self):
        node = VehicleControllerNode()
        self.__motor_power = node.__motor_power 
        self.__steering_angle = 0.0



def main(args=None):
    rclpy.init(args=args)

    vehicle_controller = VehicleControllerNode()

    rclpy.spin(vehicle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()