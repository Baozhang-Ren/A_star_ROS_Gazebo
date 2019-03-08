import numpy as np
import matplotlib.pyplot as plt
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm
import math

BOUND = np.array([[4, 4], [4, -7], [-7, -7], [-7, 4]])
OBSTACLES = np.array([[[-1, -1], [-5, -1], [-5, 2], [-3, 2]],
                      [[-2, 2], [2, 2], [2, -4]],
                      [[-1, -2.5], [1.2, -4], [1.2, -5], [-5, -5], [-5, -2.5]]])
GOAL = np.array([[[-1, -1.75], [-5, -1.75]], [[-1, -1.75], [-6, -2]]])


class ReduceVisibilityGraph():
    def __init__(self, bound, obstacles, start, goal):
        self.bound = bound
        self.obstacles = obstacles
        self.start = start
        self.goal = goal

        self.connect_list = []    # the pairs of vertices which should connected

    def cross(self, p1, p2, p3):  # using for check intersect
        return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])

    def if_intersect(self, segment_1, segment_2):  # if two segment intersect
        p1 = segment_1[0]
        p2 = segment_1[1]
        p3 = segment_2[0]
        p4 = segment_2[1]
        return self.cross(p1, p3, p4) != self.cross(p2, p3, p4) and self.cross(p1, p2, p3) != self.cross(p1, p2, p4)

    def if_unblock(self, segment):  # if a segment not blocked(intersect with any obstacles)
        for obstacles_num in range(len(self.obstacles)):
            for i in range(len(self.obstacles[obstacles_num])):
                if i < len(self.obstacles[obstacles_num]) - 1:
                    if self.if_intersect(segment, (self.obstacles[obstacles_num][i], self.obstacles[obstacles_num][i + 1])) \
                            and self.if_intersect(segment, (self.obstacles[obstacles_num][i + 1], self.obstacles[obstacles_num][i])):
                        return False
                else:
                    if self.if_intersect(segment, (self.obstacles[obstacles_num][i], self.obstacles[obstacles_num][0])) \
                            and self.if_intersect(segment, (self.obstacles[obstacles_num][0], self.obstacles[obstacles_num][i])):
                        return False
        return True

    def find_connect_list(self):
        # start to goal
        segment = (self.start, self.goal)
        self.connect_list.append(segment)

        # start to obstacles
        for i in range(len(self.obstacles)):
            for vertex in range(len(self.obstacles[i])):
                segment = (self.start, self.obstacles[i][vertex])
                if self.if_unblock(segment):
                    self.connect_list.append(segment)

        # goal to obstacle;
        for i in range(len(self.obstacles)):
            for vertex in range(len(self.obstacles[i])):
                segment = (self.goal, self.obstacles[i][vertex])
                if self.if_unblock(segment):
                    self.connect_list.append(segment)

        # between obstacles:
        for i in range(len(self.obstacles)):
            for j in range(i+1, len(self.obstacles)):
                for v1 in range(len(self.obstacles[i])):
                    for v2 in range(len(self.obstacles[j])):
                        segment = (self.obstacles[i][v1], self.obstacles[j][v2])
                        if self.if_unblock(segment):
                            self.connect_list.append(segment)

        # obstacles' edge
        for i in range(len(self.obstacles)):
            for j in range(len(self.obstacles[i])):
                if j < len(self.obstacles[i]) - 1:
                    segment = (self.obstacles[i][j], self.obstacles[i][j+1])
                else:
                    segment = (self.obstacles[i][j], self.obstacles[i][0])
                self.connect_list.append(segment)

    #def reduce_connect_list(self):



    def draw_map(self):
        self.connect_list = np.asarray(self.connect_list)
        fig, ax = plt.subplots(figsize=(11, 11))

        # draw bound
        plt.plot(np.append(np.array(self.bound)[:, 0], np.array(self.bound)[0, 0]),
                 np.append(np.array(self.bound)[:, 1], np.array(self.bound)[0, 1]), label="obstacles1")

        # draw obstacles
        plt.plot(np.append(np.array(self.obstacles[0])[:, 0], np.array(self.obstacles[0])[0, 0]),
                 np.append(np.array(self.obstacles[0])[:, 1], np.array(self.obstacles[0])[0, 1]), 'red', label="obstacles1")
        plt.plot(np.append(np.array(self.obstacles[1])[:, 0], np.array(self.obstacles[1])[0, 0]),
                 np.append(np.array(self.obstacles[1])[:, 1], np.array(self.obstacles[1])[0, 1]), 'red', label="obstacles2")
        plt.plot(np.append(np.array(self.obstacles[2])[:, 0], np.array(self.obstacles[2])[0, 0]),
                 np.append(np.array(self.obstacles[2])[:, 1], np.array(self.obstacles[2])[0, 1]), 'red', label="obstacles3")

        for segment in range(len(self.connect_list)):
            plt.plot(self.connect_list[segment][:, 0], self.connect_list[segment][:, 1], 'b--')



        # print(self.start)
        # print(self.obstacles[0][0])
        # plt.plot(np.array([self.start[0], self.goal[0]]), np.array([self.start[1], self.goal[1]]))
        # plt.plot(self.goal[0], self.goal[1])

        plt.show()


if __name__ == '__main__':
    grad_word = ReduceVisibilityGraph(BOUND, OBSTACLES, GOAL[0][0], GOAL[0][1])

    grad_word.find_connect_list()
    grad_word.draw_map()
    print(grad_word.connect_list)

