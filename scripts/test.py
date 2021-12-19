import numpy as np
import matplotlib.pyplot as plt
from imageio import imread
# import matplotlib.image as mpimg
from skimage import measure, color
from skimage.color.colorconv import rgb2gray, rgba2rgb
from shapely.geometry import shape, Point, Polygon, LineString
from shapely import affinity

from descartes.patch import PolygonPatch

# read a PNG
polypic = imread('.\mapas\my_map.png')
# convert to greyscale if need be
# polypic = rgba2rgb(polypic)
gray = rgb2gray(polypic)

# find contours
# Not sure why 1.0 works as a level -- maybe experiment with lower values
contours = measure.find_contours(gray, 0.5)


# build polygon, and simplify its vertices if need be
# this assumes a single, contiguous shape
# if you have e.g. multiple shapes, build a MultiPolygon with a list comp

# RESULTING POLYGONS ARE NOT GUARANTEED TO BE SIMPLE OR VALID
# check this yourself using e.g. poly.is_valid
# print(contours)

poly_list = []
for obs in contours:
    poly_list.append(Polygon(obs).simplify(1.0))


# fig = plt.figure(figsize=(8,5), dpi=100)
# ax = fig.add_subplot(111, aspect='equal')
for i in poly_list:
    plt.plot(*i.exterior.xy)


# print(np.array(contours).shape)
# poly = Polygon(contours[0]).simplify(1.0)
# # poly = affinity.rotate(poly, -90)

# poly3 = Polygon(contours[1]).simplify(1.0)
# # poly = Polygon(contours)
# # poly3 = affinity.rotate(poly3, -90)

# print(polypic.shape)
try:
    WORLDX, WORLDY = polypic.shape
except ValueError:
    WORLDX, WORLDY, _ = polypic.shape
# fig = plt.figure(figsize=(8,5), dpi=100)
# ax = fig.add_subplot(111, aspect='equal')
# # obs3 = Polygon([(20, 10), (15, 5), (30, 5), (35, 10)])

# ax.plot(*poly.exterior.xy)
# ax.plot(*poly3.exterior.xy)
# ax.set_xlim(0, WORLDX)
# ax.set_ylim(0, WORLDY)

plt.xlim(0, WORLDX)
plt.ylim(0, WORLDY)

# for i in range(2, len(poly_list)):
#     ax.add_patch(PolygonPatch(poly_list[i], facecolor='gray'))
# ax.add_patch(PolygonPatch(poly_list[1], facecolor='gray'))
    # ax.add_patch(PolygonPatch(poly, facecolor='gray'))
    

# line = LineString([[178, 20], [200, 25]])
# x, y = line.xy
# plt.gca().plot(x, y, color='r')
# print(line.intersects(poly_list[0]))
plt.show()
