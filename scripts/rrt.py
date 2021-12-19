import random
import numpy as np
from math import cos, sin, sqrt, atan2

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from utils import sim
from utils.utils import log_message
from utils.tree_image import TreeImage
from utils.sim_object import SimulationError
from utils.omnidirectional_robot import OmnidirectionallRobot


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None

class RRT:
    def __init__(self, clientID, img_path: str = None) -> None:
        self.goal = [8, 5]
        self.tolerance = 0.1
        self.step_size = 5
        self.start = Node(0, 0)
        self.end = Node(1, 1)

        self.max_iter = 5000
        self.node_list = []

        self.map_dims = [10, 10]
        
        self.robot = OmnidirectionallRobot(object_name="robotino", clientID=clientID)

        if img_path is None:
            # Square Maze
            self.img_path = '.\mapas\circular_maze_code_grown.png'
            self.img_path = '.\mapas\my_map_rotade.png'
            
            # Circular Maze
            # self.img_path = '.\mapas\circular_maze_code_grown.png'
        self.tree = TreeImage(map_dir=self.img_path, map_dims=self.map_dims)
        
    
    def get_input(self):
        """
        Method that treats the goal input
        """
        valid_input = False
        log_message(f"Robot current position: {self.robot.get_position()}")
        log_message(f"Map dimensions: [0, 10, 0 10]")

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
                self.goal = self.coord_to_pixel(self.goal[0], self.goal[1])
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

    @staticmethod  
    def dist(p1,p2): 
        return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def get_random(self):
        img = mpimg.imread(self.img_path)
        try:
            WORLDX, WORLDY = img.shape
        except ValueError:
            WORLDX, WORLDY, _ = img.shape
        x = np.random.uniform(0, WORLDX)
        y = np.random.uniform(0, WORLDY)
        return Node(x, y)
    
    def extend_rrt(self, q_rand):
        q_near = self.find_qnear(q_rand)
        q_new = self.step(q_near, q_rand)
        p1 = (q_near.x, q_near.y)
        p2 = (q_new.x, q_new.y)

        if self.tree.check_free_collision(p1, p2):
            self.tree.plot_tree_in_map(self.node_list, self.end)
            self.node_list.append(q_new)
            return q_new
        else:
            return None
    
    def find_qnear(self, node: Node) -> Node:
        dist_list = [(q.x - node.x)**2 + (q.y - node.y)**2 for q in self.node_list]
        closest_index = dist_list.index(min(dist_list))
        return self.node_list[closest_index]
    
    def step(self, q1: Node, q2: Node) -> Node:
        step_len = self.step_size
        q_new = Node(q1.x, q1.y)
        d = self.dist([q2.x, q2.y], [q_new.x, q_new.y])
        theta = atan2((q2.y-q_new.y),(q2.x-q_new.x))

        if step_len > d:
            step_len = d
        
        q_new.x = step_len*cos(theta) + q1.x
        q_new.y = step_len*sin(theta) + q1.y
        q_new.parent = q1

        return q_new
    
    def plot_tree(self):
        plt.cla()
        img = mpimg.imread(self.img_path)
        try:
            WORLDX, WORLDY = img.shape
        except ValueError:
            WORLDX, WORLDY, _ = img.shape
        plt.xlim(0, WORLDX)
        plt.ylim(0, WORLDY)
        plt.plot(self.end.x, self.end.y, 'g*')
        for n in self.node_list:
            plt.plot(n.x, n.y, 'b*')
        plt.pause(0.0001)
    
    def final_path(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        img = mpimg.imread(self.img_path)
        try:
            WORLDX, WORLDY = img.shape
        except ValueError:
            WORLDX, WORLDY, _ = img.shape

        plt.xlim(0, WORLDX)
        plt.ylim(0, WORLDY)
        plt.plot(self.end.x, self.end.y, 'g*')
        for n in self.node_list:
            plt.plot(n.x, n.y, 'b*')
        
        plt.plot([p[0] for p in path], [p[1] for p in path], 'g')
        plt.show()
        return path
    
    def planning(self):
        self.node_list = [self.start]
        min_d = float('inf')
        count = 0
        for _ in range(self.max_iter):
            if count % 10 == 0:
                q_rand = self.end
            else:
                q_rand = self.get_random()
            count += 1
            self.extend_rrt(q_rand)

            p1 = (self.node_list[-1].x, self.node_list[-1].y)
            p2 = (self.end.x, self.end.y)
            if self.dist(p1, p2) < min_d:
                log_message(self.dist(p1, p2))
                min_d = self.dist(p1, p2)
            if self.dist(p1, p2) <= self.step_size:# or self.tree.check_free_collision(p1, p2):
                return self.final_path(len(self.node_list) - 1)
        return None
    
    def pixel_to_coord(self, x, y):
        img = mpimg.imread(self.img_path)
        try:
            WORLDX, WORLDY = img.shape
        except ValueError:
            WORLDX, WORLDY, _ = img.shape
        scale_x = self.map_dims[0]/WORLDX
        scale_y = self.map_dims[1]/WORLDY
        x_coord = x * scale_x
        y_coord = y * scale_y
        return x_coord, y_coord
    
    def coord_to_pixel(self, x, y):
        img = mpimg.imread(self.img_path)
        try:
            WORLDX, WORLDY = img.shape
        except ValueError:
            WORLDX, WORLDY, _ = img.shape
        scale_x = self.map_dims[0]/WORLDX
        scale_y = self.map_dims[1]/WORLDY
        x_pixel = int(x / scale_x)
        y_pixel = int(y / scale_y)
        return x_pixel, y_pixel

    def main_service(self):        
        start_coord = self.robot.get_position()[:2]
        start_pixel = self.coord_to_pixel(start_coord[0], start_coord[1])
        # goal = [8, 5]
        # goal_pixel = self.coord_to_pixel(goal[0], goal[1])
        self.get_input()

        self.start = Node(start_pixel[0], start_pixel[1])
        self.end = Node(self.goal[0], self.goal[1])
        path = self.planning()
        path.reverse()
        for point in path:
            x, y = self.pixel_to_coord(point[0], point[1])
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
            service = RRT(clientID)
            service.main_service()
        else:
            raise SimulationError
    except SimulationError:
        log_message('Failed connecting to remote API server')
