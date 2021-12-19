import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, sqrt

from utils import sim
from utils.utils import log_message
from utils.sim_object import SimulationError
from utils.differential_robot import DifferentialRobot

CAVE = True
if CAVE:
    MAP_DIM = [-8, 8, -8, 8]
else:
    MAP_DIM = [-10, 10, -8, 8]

class PotentialField:
    def __init__(self, clientID) -> None:
        self.goal = [0, -1.1]
        self.tolerance = 0.2
        self.robot = DifferentialRobot(object_name="Pioneer_p3dx", clientID=clientID)
    
    def get_input(self):
        """
        Method that treats the goal input
        """
        log_message(f"Robot current position: {self.robot.get_position()}")
        valid_input = 0

        try:
            if CAVE:
                map_dir = '.\\mapas\\cave.png'
            else:
                map_dir = '.\\mapas\\paredes.png'
            img = plt.imread(map_dir)
            plt.imshow(img,extent=MAP_DIM)
            plt.grid(True)
            plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
            plt.plot(self.robot.get_position()[0], self.robot.get_position()[1], color='r', marker="o",  markersize=10)
            plt.show()
        except FileNotFoundError:
            print('Map not found')

        while not valid_input:
            try:
                self.goal[0] =  input("Input the goal x coordinate ('q' to quit): ")
                if self.goal[0] == 'q':
                    quit()
                self.goal[0] = float(self.goal[0])

                self.goal[1] =  input("Input the goal y coordinate ('q' to quit): ")
                if self.goal[1] == 'q':
                    quit()
                self.goal[1] = float(self.goal[1])

                log_message(f"Goal set to {self.goal}")
                valid_input = 1
            except ValueError:
                log_message("Input must be a number")
    
    def attractive_potencial(self, x, y):

        K = 0.2  # gain

        Ux = - K*(x - self.goal[0])
        Uy = - K*(y - self.goal[1])
        U_a = [Ux, Uy]

        return U_a
    
    def repulsive_potencial(self, closest_dist, closest_angle, theta):
        K = 0.15
        D_safe = 5

        if (closest_dist > D_safe):
            Ux = 0
            Uy = 0
            U_r = [Ux, Uy]

        else:
            grad_x = cos(closest_angle + theta)
            grad_y = sin(closest_angle + theta)

            Ux = K * (1.0/D_safe - 1.0/closest_dist) * (1.0/closest_dist**2) * grad_x 
            Uy = K * (1.0/D_safe - 1.0/closest_dist) * (1.0/closest_dist**2) * grad_y

            U_r = [Ux, Uy]

        return U_r
    
    def distance_to_goal(self):
        x = self.robot.get_position()[0]
        y = self.robot.get_position()[1]
        return sqrt((self.goal[0] - x) ** 2 + (self.goal[1] - y) ** 2)
    
    def plot_info(self, U_att, U_rep, pos=None):
        x = self.robot.get_position()[0]
        y = self.robot.get_position()[1]
        U_total = 5 * (U_att + U_rep)

        plt.cla()
        plt.xlim(MAP_DIM[:2])
        plt.ylim(MAP_DIM[-2:])

        try:
            if CAVE:
                map_dir = '.\\mapas\\cave.png'
            else:
                map_dir = '.\\mapas\\paredes.png'
            img = plt.imread(map_dir)
            plt.imshow(img,extent=MAP_DIM)
        except FileNotFoundError:
            pass

        plt.grid(True)
        plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        plt.plot(self.goal[0], self.goal[1], color='g', marker="o",  markersize=10)        
        plt.plot(self.robot.get_position()[0], self.robot.get_position()[1], color='r', marker="o",  markersize=5)

        plt.quiver(x, y, U_total[0], U_total[1])
        plt.quiver(x, y, U_att[0], U_att[1], color='b') 
        plt.quiver(x, y, U_rep[0], U_rep[1], color='r')

        if pos is not None:
            plt.plot([x[0] for x in pos], [y[1] for y in pos], 'b')

        plt.pause(0.0001)

    def main_service(self):
        self.get_input()
        pos = []
        while self.distance_to_goal() > self.tolerance: 
            x = self.robot.get_position()[0]
            y = self.robot.get_position()[1]
            pos.append((x,y))
            closest_dist, closest_angle = self.robot.closest_obstacle()
            theta = self.robot.get_orientation()[-1]

            attractive = np.array(self.attractive_potencial(x, y))
            repulsive = np.array(self.repulsive_potencial(closest_dist, closest_angle, theta))

            self.plot_info(attractive, repulsive,pos)

            U_total = attractive + repulsive
            # Normalizing the control input
            U_total_norm = U_total/np.linalg.norm(U_total) * 0.25

            # print(f"U_total: {np.round(U_total, 2)} | attractive: {np.round(attractive, 2)} | repulsive: {np.round(repulsive, 2)}\r", end="")

            v, w = self.robot.feedback_linearization(U_total_norm[0], U_total_norm[1])
            self.robot.set_wheel_speeds(v, w)
        
        log_message("Goal reached")        
        # Stopping the robot
        self.robot.set_wheel_speeds(0, 0)
        self.main_service()


if __name__ == '__main__':
    log_message('Starting program')
    
    try:
        # Closing all opened connections
        sim.simxFinish(-1)
        # Connect to CoppeliaSim
        clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
        if clientID != -1:
            log_message('Connected to remote API server')
            service = PotentialField(clientID)
            service.main_service()
        else:
            raise SimulationError
    except SimulationError:
        log_message('Failed connecting to remote API server')
