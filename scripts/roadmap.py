import networkx
import numpy as np
from math import cos, sin, sqrt

from utils import sim
from utils.utils import log_message
from utils.graph_image import GraphImage
from utils.sim_object import SimulationError
from utils.omnidirectional_robot import OmnidirectionallRobot

SQUARE = True


class Roadmap:
    def __init__(self, clientID, img_path: str = None) -> None:
        self.goal = [0, 0]
        self.tolerance = 0.1

        if SQUARE:
            self.cell_size = 0.12
        else:
            self.cell_size = 0.15

        self.map_dims = [20, 20]
        self.robot = OmnidirectionallRobot(object_name="robotino", clientID=clientID)

        if img_path is None:
            if SQUARE:
                # Square Maze
                img_path = '.\mapas\square_maze_new_code_grown.png'
            else:
                # Circular Maze
                img_path = '.\mapas\circular_maze_code_grown.png'
        self.graph = GraphImage(map_dir=img_path, cell_size=self.cell_size, map_dims=self.map_dims)
    
    def get_input(self):
        """
        Method that treats the goal input
        """
        log_message(f"Robot current position: {self.robot.get_position()}")
        log_message(f"Map dimensions: [0, 20, 0 20]")
        valid_input = False

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
                node_goal = self.coord_to_node(self.goal[0], self.goal[1])
                log_message(f"Closest position of {self.goal} in the graph is {self.node_to_coord(node_goal[0], node_goal[1])}")
                valid_input = 1
            except ValueError:
                log_message("Input must be a number")

    def distance_to_target(self, tx, ty):
        x = self.robot.get_position()[0]
        y = self.robot.get_position()[1]
        return sqrt((tx - x) ** 2 + (ty - y) ** 2)

    def control(self, tx, ty):
        k = 0.2
        U = np.zeros(2)
        while self.distance_to_target(tx, ty) > self.tolerance:
            x_robot = self.robot.get_position()[0]
            y_robot = self.robot.get_position()[1]
            U[0] = (tx - x_robot)
            U[1] = (ty - y_robot)
            U = k * U/np.linalg.norm(U)
            self.robot.set_wheel_speeds(vx=U[0], vy=U[1], vtheta=0)

    def node_to_coord(self, row, column):
        x = column * self.cell_size
        y = self.map_dims[1] - row * self.cell_size
        return x, y
    
    def coord_to_node(self, x, y):
        """
        Returns the node that is the closest one to a desired coordinate
        """
        nodes = np.array(self.graph.G.nodes)
        row = int((self.map_dims[1] - y)/self.cell_size)
        column = int(x/self.cell_size)
        diff = nodes - np.array([row, column])
        diff_sum = np.sum(np.abs(diff), axis=1)
        id_min = np.where(diff_sum == np.amin(diff_sum))[0][0]

        return nodes[id_min][0], nodes[id_min][1]

    def main_service(self):
        invalid = True
        self.get_input()
        # Square Maze
        start_coord = self.robot.get_position()[:2]
        start_node = self.coord_to_node(start_coord[0], start_coord[1])
        end_node = self.coord_to_node(self.goal[0], self.goal[1])

        # Circular Maze
        while invalid:
            try:
                path = self.graph.shortest_path(start_node=start_node, end_node=end_node, plot=True)
                invalid = False
            except networkx.exception.NetworkXNoPath:
                log_message("No path exists, try another goal")
                self.get_input()
                end_node = self.coord_to_node(self.goal[0], self.goal[1])

        for point in path:
            x, y = self.node_to_coord(point[0], point[1])
            self.control(x, y)
        log_message('Stopping Robot')
        self.robot.set_wheel_speeds(vx=0, vy=0, vtheta=0)


if __name__ == '__main__':
    log_message('Starting program')    
    try:
        # Closing all opened connections
        sim.simxFinish(-1)
        # Connect to CoppeliaSim
        clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
        if clientID != -1:
            log_message('Connected to remote API server')
            service = Roadmap(clientID)
            service.main_service()
        else:
            raise SimulationError
    except SimulationError:
        log_message('Failed connecting to remote API server')
