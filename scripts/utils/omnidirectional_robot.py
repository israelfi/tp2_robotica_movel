import numpy as np

from math import cos, sin, sqrt
from . import sim
from .laser_sensor import LaserSensor
from .sim_object import SimObject
from .utils import Rz


class OmnidirectionallRobot(SimObject):
    def __init__(self, object_name: str, clientID):
        self.clientID = clientID

        self.wheel_radius = 0.040
        self.l = 0.135  # distance between the centers of the two wheels

        # Left wheel handle
        self.left_wheel = SimObject('wheel0_joint', clientID)
        # Back wheel handle
        self.back_wheel = SimObject('wheel1_joint', clientID)
        # Right wheel handle
        self.right_wheel = SimObject('wheel2_joint', clientID)

        self.Mdir = np.array([
            [-self.wheel_radius/np.sqrt(3), 0, self.wheel_radius/np.sqrt(3)], 
            [self.wheel_radius/3, (-2*self.wheel_radius)/3, self.wheel_radius/3], 
            [self.wheel_radius/(3*self.l), self.wheel_radius/(3*self.l), self.wheel_radius/(3*self.l)]]
        )

        super().__init__(object_name, self.clientID)
    
    def set_wheel_speeds(self, vx: float, vy: float, vtheta: float = 0):
        """
        Calculates and sets the wheels velocities given the desired velocities in world frame
        Args:
            vx: linear velocity
            w: angular velocity

        Returns: wheel velocities
        """
        v = np.array([vx, vy, vtheta])
        theta = self.get_orientation()[-1]
        Minv = np.linalg.inv(Rz(theta) @ self.Mdir)
        u = Minv @ v
        _ = sim.simxSetJointTargetVelocity(self.clientID, self.left_wheel.handle, u[0], sim.simx_opmode_streaming + 5)
        _ = sim.simxSetJointTargetVelocity(self.clientID, self.back_wheel.handle, u[1], sim.simx_opmode_streaming + 5)
        _ = sim.simxSetJointTargetVelocity(self.clientID, self.right_wheel.handle, u[2], sim.simx_opmode_streaming + 5)
