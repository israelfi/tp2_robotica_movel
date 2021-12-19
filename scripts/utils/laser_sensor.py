import numpy as np
import matplotlib.pyplot as plt

from . import sim
from .sim_object import SimObject


class LaserSensor(SimObject):
    """
    Class that deals with sensor data and operations
    """
    def __init__(self, clientId, range_data="hokuyo_range_data", angle_data="hokuyo_angle_data", handle_name=None):
        self.laser_data = None
        self.clientId = clientId
        self.laser_range_data = range_data
        self.laser_angle_data = angle_data
        
        # Geralmente a primeira leitura é inválida (atenção ao Operation Mode)
        # Em loop até garantir que as leituras serão válidas
        returnCode = 1
        while returnCode != 0:
            returnCode, range_data = sim.simxGetStringSignal(self.clientId, self.laser_range_data, sim.simx_opmode_streaming + 10)
        
        # Prosseguindo com as leituras
        raw_range_data, raw_angle_data = self.read_sensor_data()
        self.laser_data = np.array([raw_angle_data, raw_range_data]).T
        
        super().__init__(object_name=handle_name, clientID=clientId)
    
    def read_sensor_data(self):
        '''
        It will try to capture the range and angle data from the simulator. The request for the range data
        is sent in streaming mode to force it to sync with the angle data request which acts as a mutex.

        Args:
            -clientId: simulator client id obtained through a successfull connection with the simulator.
            -range_data_signal_id: string containing the range data signal pipe name.
            -angle_data_signal_id: string containing the angle data signal pipe name.
        Returns:
            -None if no data is recovered.
            -two arrays, one with data range and the other with their angles, if data was 
            retrieved successfully.
        '''
        returnCodeRanges, string_range_data = sim.simxGetStringSignal(self.clientId, self.laser_range_data, sim.simx_opmode_streaming)        
        returnCodeAngles, string_angle_data = sim.simxGetStringSignal(self.clientId, self.laser_angle_data, sim.simx_opmode_blocking)
        
        if returnCodeRanges == 0 and returnCodeAngles == 0:
            raw_range_data = sim.simxUnpackFloats(string_range_data)
            raw_angle_data = sim.simxUnpackFloats(string_angle_data)            
            self.laser_data = np.array([raw_angle_data, raw_range_data]).T
            return raw_range_data, raw_angle_data
        
        return None, None
