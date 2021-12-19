import numpy as np
import matplotlib.pyplot as plt

from . import sim
from .utils import Rz, plot_frame

class SimObject:
    """
    Classe para lidar com operações gerais relacionadas à objetos da cena
    """
    
    def __init__(self, object_name: str, clientID):
        self.object_name = object_name
        self.clientID = clientID
        _, self.handle = sim.simxGetObjectHandle(self.clientID, self.object_name, sim.simx_opmode_oneshot_wait)
        
        _, self.pos = sim.simxGetObjectPosition(self.clientID, self.handle, -1, sim.simx_opmode_oneshot_wait)        
        _, self.orientation = sim.simxGetObjectOrientation(self.clientID, self.handle, -1, sim.simx_opmode_oneshot_wait)      
        self.r = Rz(self.get_orientation()[-1])        
    
    def get_position(self):
        returnCode, self.pos = sim.simxGetObjectPosition(self.clientID, self.handle, -1, sim.simx_opmode_oneshot_wait)
        return self.pos
    
    def set_position(self, position=[0, 0, 0]):
        returnCode = sim.simxSetObjectPosition(self.clientID, self.handle, -1, position, sim.simx_opmode_oneshot)
        self.pos = position
        return returnCode
    
    def get_orientation(self):
        returnCode, self.orientation = sim.simxGetObjectOrientation(self.clientID, self.handle, -1, sim.simx_opmode_oneshot_wait)
        return self.orientation
    
    def set_orientation(self, angles=[0, 0, 0]):
        returnCode = sim.simxSetObjectOrientation(self.clientID, self.handle, -1, angles, sim.simx_opmode_oneshot)
        self.orientation = angles
        self.r = Rz(self.get_orientation()[-1])
        return returnCode
    
    def get_rotation_matrix_to_world_frame(self):
        self.r = Rz(self.get_orientation()[-1])
        return self.r
    
    def homogeneous_transformation_in_world_frame(self):
        aux = np.array([0, 0, 0, 1])     
        
        R = self.get_rotation_matrix_to_world_frame()
        p = self.get_position()
        
        T = np.column_stack((R, p))
        T = np.row_stack((T, aux))        
        return T
    
    def plot_frame_in_world_frame(self):
        # Frame do mundo
        pw = [0, 0, 0]
        Rw = Rz(0)
        plot_frame(pw, Rw, ['r', 'r'])
        
        plot_frame(self.pos, self.r, ['g', 'g'])
        plt.axis('scaled')
        plt.axis((-5, 5, -5, 5))


class SimulationError(BaseException):
    pass