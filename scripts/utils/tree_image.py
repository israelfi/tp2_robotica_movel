import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from numpy.lib.polynomial import poly

from skimage.color.colorconv import rgb2gray
from shapely.geometry import shape, Point, Polygon, LineString
from shapely import affinity
from skimage import measure
from descartes.patch import PolygonPatch

class TreeImage:
    plot = False

    def __init__(self, map_dir: str, map_dims: list = [20, 20]) -> None:
        # Dimensões do mapa informado em metros (X, Y)
        self.map_dims = np.array(map_dims)

        # Invertendo os valores para visualização (Branco - 0, Preto - 1)
        img = 1 - mpimg.imread(map_dir)
        self.img = img

        # polypic = rgba2rgb(polypic)
        gray = rgb2gray(img)
        # find contours
        contours = measure.find_contours(gray, 0.5)
        if 'square' in map_dir:
            self.poly_map = Polygon(contours[1]).simplify(1.0)
            self.poly_map = affinity.rotate(self.poly_map, -90)
        elif 'my_map' in map_dir:
            poly_map_list = []
            for obs in contours:
                poly_map_list.append(Polygon(obs).simplify(1.0))
            self.poly_map = poly_map_list
        else:
            self.poly_map = Polygon(contours[0]).simplify(1.0)
            self.poly_map = affinity.rotate(self.poly_map, -90)

        if self.plot:
            fig = plt.figure(figsize=(8,5), dpi=100)
            ax = fig.add_subplot(111, aspect='equal')
            try:
                WORLDX, WORLDY = img.shape
            except ValueError:
                WORLDX, WORLDY, _ = img.shape
            if isinstance(self.poly_map, list):
                for obs in self.poly_map:
                    ax.add_patch(PolygonPatch(obs, facecolor='gray'))
            else:
                ax.add_patch(PolygonPatch(self.poly_map, facecolor='gray'))
            ax.set_xlim(0, WORLDX)
            ax.set_ylim(0, WORLDY)
            plt.show()
            # plt.plot(*poly_map.exterior.xy)
   
    
    def check_free_collision(self, p1, p2):
        line = LineString([p1, p2])
        if isinstance(self.poly_map, list):
            for obs in self.poly_map: 
                colision = line.intersects(obs)
                if colision:
                    return False
            return True
        else:
            return not line.intersects(self.poly_map)
    
    def plot_map(self, ax, node_list):
        WORLDX, WORLDY = self.img.shape
        ax.add_patch(PolygonPatch(self.poly_map, facecolor='gray'))
        ax.set_xlim(0, WORLDX)
        ax.set_ylim(0, WORLDY)
        for n in node_list:
            ax.plot(n.x, n.y, 'b*')
        plt.pause(0.1)
    
    def plot_tree_in_map(self, node_list, end):
        plt.cla()
        try:
            WORLDX, WORLDY = self.img.shape
        except ValueError:
            WORLDX, WORLDY, _ = self.img.shape
        plt.xlim(0, WORLDX)
        plt.ylim(0, WORLDY)
        if isinstance(self.poly_map, list):
            for obs in self.poly_map: 
                plt.plot(*obs.exterior.xy, 'k')
        plt.plot(end.x, end.y, 'g*')
        for n in node_list:
            plt.plot(n.x, n.y, 'b*')
        plt.pause(0.0001)