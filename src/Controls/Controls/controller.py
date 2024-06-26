from node_control import VehicleControllerNode
import math

ERROR_SENSITIVITY = 2
MAX_POWER = 1.0
TURNING_POWER = 0.6
WHEELBASE = 16/100
TURNING_RADIUS = 80/100
TIRE_WIDTH = 5/100

class Controller:
    def __init__(self):
        node = VehicleControllerNode()
        self.__motor_power = node.__motor_power.motor_power
        self.__steering_angle: float
        
        self.__current_heading = node.__current_heading.current_heading
        self.__new_heading = node.__new_heading.new_heading
        

    def _heading_diff(self) -> float:
        return self.__new_heading - self.__current_heading

    def get_motor_speed(self) -> float:
        heading_diff = self._heading_diff()
        
        if self.__motor_power == 0:
            # Stop
            return self.__motor_power
        elif (-ERROR_SENSITIVITY < math.abs(heading_diff) < ERROR_SENSITIVITY) or math.abs(heading_diff) == 180:
            # continue going straight
            return MAX_POWER
        else:
            # turn left or right
            return TURNING_POWER
        
    def get_steering_angle(self):
        heading_diff = self._heading_diff()
        
        if math.abs(heading_diff) == 180:
            self.__new_heading = self.__current_heading
        else:
            self.__new_heading = math.arcsin(WHEELBASE/(TURNING_RADIUS - (TIRE_WIDTH)/2))
        
        return self.__new_heading