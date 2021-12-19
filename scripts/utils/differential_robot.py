import numpy as np

from math import cos, sin
from . import sim
from .laser_sensor import LaserSensor
from .sim_object import SimObject


class DifferentialRobot(SimObject):
    def __init__(self, object_name: str, clientID):
        self.clientID = clientID

        self.wheel_radius = 0.0975
        self.l = 0.381  # distance between the centers of the two wheels
        self.d = 0.2  # distance from the the center of the robot to the front

        # Left wheel handle
        self.left_wheel = SimObject('Pioneer_p3dx_leftMotor', clientID)
        # Right wheel handle
        self.right_wheel = SimObject('Pioneer_p3dx_rightMotor', clientID)

        # Laser handle
        self.laser = LaserSensor(clientId=self.clientID, handle_name="fastHokuyo")
        super().__init__(object_name, self.clientID)
    
    def set_wheel_speeds(self, v: float, w: float):
        """
        Calculates and sets the wheels velocities given a desired linear and angular velocities
        Args:
            v: linear velocity
            w: angular velocity

        Returns: wheel velocities
        """
        wl = v/self.wheel_radius - (w * self.l)/(2 * self.wheel_radius)
        wr = v/self.wheel_radius + (w * self.l) /(2 * self.wheel_radius)
        # Enviando velocidades
        _ = sim.simxSetJointTargetVelocity(self.clientID, self.left_wheel.handle, wl, sim.simx_opmode_streaming + 5)
        _ = sim.simxSetJointTargetVelocity(self.clientID, self.right_wheel.handle, wr, sim.simx_opmode_streaming + 5)

        # print(f"v: {round(v, 2)} | w: {round(w, 2)} | wl: {round(wl, 2)} | wr: {round(wr, 2)}\r", end="")

        return wl, wr

    def feedback_linearization(self, Ux, Uy):
        """
        Calculates the linear and angular velocities given the velocities in the inertial frame
        Args:
            Ux: velocity in the x axis
            Uy: velocity in the y axis

        Returns: linear and angular velocities
        """
        theta = self.get_orientation()[-1]
        v = cos(theta) * Ux + sin(theta) * Uy
        w = -(sin(theta) * Ux)/ self.d + (cos(theta) * Uy) / self.d

        return v, w
    
    def closest_obstacle(self):
        """
        Calculates the distance and the angle to the closest obstacle
        """
        laser_data = self.laser.read_sensor_data()
        min_dist = np.min(laser_data[0])
        angle_of_closest_obstacle = laser_data[1][np.where(laser_data[0] == min_dist)[0][0]]
        # print(f"min_dist: {round(min_dist, 2)} | angle: {round(angle_of_closest_obstacle, 2)}\r", end="")
        return min_dist, angle_of_closest_obstacle
