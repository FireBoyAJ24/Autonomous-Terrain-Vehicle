import rclpy
from controls.node_control import VehicleControllerNode

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
