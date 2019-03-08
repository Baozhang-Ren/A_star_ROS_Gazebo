import numpy as np
import matplotlib.pyplot as plt
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm
import math

BOUND = np.array([[8, 8], [8, -7], [-7, -7], [-7, 8]])
OBSTACLES = np.array([[[-1, -1], [-6, -2], [-5, 2], [-3, 2], [-4, 0]],
                      [[6, 5], [4, 1], [5, -2], [2, -4], [1, 2]],
                      [[0, -3], [0, -4], [1, -5], [-5, -5], [-5, -4]],
                      [[7, 6], [0, 4], [-5, 6], [0, 6], [4, 7]]])
GOAL = np.array([[[-6, 6], [7, -6]], [[-5, -6], [5, 1]]])


# fig, ax = plt.subplots(figsize=(11, 11))
# plt.plot(np.append(np.array(OBSTACLES[0])[:, 0], np.array(OBSTACLES[0])[0, 0]),
#          np.append(np.array(OBSTACLES[0])[:, 1], np.array(OBSTACLES[0])[0, 1]), label="obstacles1")
# plt.plot(np.append(np.array(OBSTACLES[1])[:, 0], np.array(OBSTACLES[1])[0, 0]),
#          np.append(np.array(OBSTACLES[1])[:, 1], np.array(OBSTACLES[1])[0, 1]), label="obstacles2")
# plt.plot(np.append(np.array(OBSTACLES[2])[:, 0], np.array(OBSTACLES[2])[0, 0]),
#          np.append(np.array(OBSTACLES[2])[:, 1], np.array(OBSTACLES[2])[0, 1]), label="obstacles3")
# x_major_ticks = np.arange(-7, 4.25, 0.5)
# x_minor_ticks = np.arange(-7, 4.25, 0.25)
# y_major_ticks = np.arange(-7, 4.25, 0.5)
# y_minor_ticks = np.arange(-7, 4.25, 0.25)
# ax.set_xticks(x_major_ticks)
# ax.set_xticks(x_minor_ticks, minor=True)
# ax.set_yticks(y_major_ticks)
# ax.set_yticks(y_minor_ticks, minor=True)
# # And a corresponding grid
# ax.grid(which='both')
#
# # Or if you want different settings for the grids:
# ax.grid(which='minor', alpha=0.2)
# ax.grid(which='major', alpha=0.5)
#
# plt.show()






def dot(v,w):
    x,y = v
    X,Y = w
    return x*X + y*Y

def length(v):
    x,y = v
    return math.sqrt(x*x + y*y)

def vector(b,e):
    x,y = b
    X,Y = e
    return (X-x, Y-y)

def unit(v):
    x,y = v
    mag = length(v)
    return (x/mag, y/mag)

def distance(p0,p1):
    return length(vector(p0,p1))

def scale(v,sc):
    x,y= v
    return (x * sc, y * sc)

def add(v,w):
    x,y = v
    X,Y = w
    return (x+X, y+Y)


# Given a line with coordinates 'start' and 'end' and the
# coordinates of a point 'pnt' the proc returns the shortest
# distance from pnt to the line and the coordinates of the
# nearest point on the line.
#
# 1  Convert the line segment to a vector ('line_vec').
# 2  Create a vector connecting start to pnt ('pnt_vec').
# 3  Find the length of the line vector ('line_len').
# 4  Convert line_vec to a unit vector ('line_unitvec').
# 5  Scale pnt_vec by line_len ('pnt_vec_scaled').
# 6  Get the dot product of line_unitvec and pnt_vec_scaled ('t').
# 7  Ensure t is in the range 0 to 1.
# 8  Use t to get the nearest location on the line to the end
#    of vector pnt_vec_scaled ('nearest').
# 9  Calculate the distance from nearest to pnt_vec_scaled.
# 10 Translate nearest back to the start/end line.
# Malcolm Kesson 16 Dec 2012

def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = scale(pnt_vec, 1.0/line_len)
    t = dot(line_unitvec, pnt_vec_scaled)
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return (dist, nearest)

class GridWorld():

    def __init__(self, bound, obstacles, goal):
        self.bound = bound
        self.obstacles = obstacles
        self.goal = goal
        self.cell_length = 0.25  # the length of each cell
        self.x_min = np.min(self.bound[:, 0])  # minimum x coordinate
        self.x_max = np.max(self.bound[:, 0])
        self.y_min = np.min(self.bound[:, 1])
        self.y_max = np.max(self.bound[:, 1])
        self.matrix = self.init_matrix()
        self.height = self.matrix.shape[0]
        self.width = self.matrix.shape[1]


    def init_matrix(self):
        x_length = (int((self.x_max - self.x_min) / 0.25)) + 1  # number of vertices in x
        y_length = (int((self.y_max - self.y_min) / 0.25)) + 1
        matrix = np.zeros((x_length, y_length))
        return matrix

    def make_grid_world(self):
        self.find_fringe()
        #self.find_inside_obstacles()
        self.extend_obstacles()

    def if_vertex_inside_obstacles(self, matrix_index):
        if_inside = False

        x_value = self.x_min + matrix_index[1] * self.cell_length
        y_value = self.y_max - matrix_index[0] * self.cell_length

        # if in obstacle 1
        if y_value > -1 and x_value > -5 and y_value < 2 and y_value < -1.5 * x_value - 2.5:
            if_inside = True

        # if in obstacle 2
        elif y_value < 2 and x_value < 2 and y_value > -1.5 * x_value -1:
            if_inside = True

        # if in obstacle 3
        elif y_value < -(15 / 22) * x_value - (35 / 11) and x_value < 1.2 and y_value > -5 and x_value > -5 \
                and y_value < -2.5:
            if_inside = True

        return if_inside

    def if_vertex_near_obstacles(self, matrix_index):
        if_near = False

        x_value = self.x_min + matrix_index[1] * self.cell_length
        y_value = self.y_max - matrix_index[0] * self.cell_length
        p0 = np.array([x_value, y_value])

        # if near obstacles
        for i in range(len(self.obstacles)):
            for j in range(len(self.obstacles[i])):
                if j < len(self.obstacles[i]) - 1:
                    d,n = pnt2line(p0, np.array(self.obstacles[i][j]), np.array(self.obstacles[i][j+1]))
                else:
                    d,n = pnt2line(p0, np.array(self.obstacles[i][j]), np.array(self.obstacles[i][0]))

                if d < 0.25:
                    if_near = True
                    return if_near
        return if_near

    def find_inside_obstacles(self):
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.if_vertex_inside_obstacles((i, j)) == True:
                    self.matrix[i][j] = 2

    def find_fringe(self):
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.matrix[i][j] != 2:
                    if self.if_vertex_near_obstacles((i, j)) == True:
                        self.matrix[i][j] = 1

    def extend_obstacles(self):
        x_length = (int((self.x_max - self.x_min) / 0.25)) + 1  # number of vertices in width
        y_length = (int((self.y_max - self.y_min) / 0.25)) + 1

        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.matrix[i][j] == 1:
                    self.matrix[i][j] = 3

        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.matrix[i][j] == 3:
                    for m in range(-1, 2):
                        for n in range(-1, 2):
                            if 0 <= i+m < y_length and 0 <= j+n <= x_length:
                                if -1 < (i + m) < self.height and -1< (j+n) < self.width:
                                    if self.matrix[i + m][j + n] != 2 and self.matrix[i + m][j + n] != 3:
                                        self.matrix[i + m][j + n] = 1

    def print_grid_world(self):
        fig, ax = plt.subplots(figsize=(11, 11))
        for i in range(len(self.obstacles)):

            plt.plot(np.append(np.array(self.obstacles[i])[:, 0], np.array(self.obstacles[i])[0, 0]),
                     np.append(np.array(self.obstacles[i])[:, 1], np.array(self.obstacles[i])[0, 1]), label="obstacles1")


        self.rout = np.asarray(self.rout)
        plt.plot(self.rout[:, 0], self.rout[:, 1], 'red')

        # change block cells to black
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.matrix[i][j] == 3:
                    # paint all cells with this vertex to black
                    x_value = self.x_min + j * self.cell_length
                    y_value = self.y_max - i * self.cell_length
                    ax.broken_barh([(x_value, self.cell_length)],
                                   (y_value, self.cell_length), facecolor='gray')
                    ax.broken_barh([(x_value - self.cell_length, self.cell_length)],
                                   (y_value, self.cell_length), facecolor='gray')
                    ax.broken_barh([(x_value - self.cell_length, self.cell_length)],
                                   (y_value - self.cell_length, self.cell_length), facecolor='gray')
                    ax.broken_barh([(x_value, self.cell_length)],
                                   (y_value - self.cell_length, self.cell_length), facecolor='gray')
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                if self.matrix[i][j] == 2:
                    # paint all cells with this vertex to black
                    x_value = self.x_min + j * self.cell_length
                    y_value = self.y_max - i * self.cell_length
                    ax.broken_barh([(x_value, self.cell_length)],
                                   (y_value, self.cell_length), facecolor='black')
                    ax.broken_barh([(x_value - self.cell_length, self.cell_length)],
                                   (y_value, self.cell_length), facecolor='black')
                    ax.broken_barh([(x_value - self.cell_length, self.cell_length)],
                                   (y_value - self.cell_length, self.cell_length), facecolor='black')
                    ax.broken_barh([(x_value, self.cell_length)],
                                   (y_value - self.cell_length, self.cell_length), facecolor='black')

        x_major_ticks = np.arange(self.x_min, self.x_max + self.cell_length, 0.5)
        x_minor_ticks = np.arange(self.x_min, self.x_max + self.cell_length, 0.25)
        y_major_ticks = np.arange(self.y_min, self.y_max + self.cell_length, 0.5)
        y_minor_ticks = np.arange(self.y_min, self.y_max + self.cell_length, 0.25)
        ax.set_xticks(x_major_ticks)
        ax.set_xticks(x_minor_ticks, minor=True)
        ax.set_yticks(y_major_ticks)
        ax.set_yticks(y_minor_ticks, minor=True)

        # And a corresponding grid
        ax.grid(which='both')

        # Or if you want different settings for the grids:
        ax.grid(which='minor', alpha=0.2)
        ax.grid(which='major', alpha=0.5)
        plt.savefig('astar_grid_world/map_2')
        plt.show()


if __name__ == '__main__':
    grad_word = GridWorld(BOUND, OBSTACLES, GOAL)
    grad_word.find_fringe()
    # grad_word.find_inside_obstacles()
    grad_word.extend_obstacles()
    grad_word.print_grid_world()




