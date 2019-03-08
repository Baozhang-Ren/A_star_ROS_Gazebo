import numpy as np
import matplotlib.pyplot as plt
import queue as q
import math

MATRIX = np.zeros((3, 5))
MATRIX[0, 1:3] = 1
MATRIX[1, 1:5] = 1
MATRIX[2, 3:5] = 1


class FDA_STAR():

    def __init__(self, matrix, start, goal):
        self.matrix = matrix
        self.start = start
        self.goal = goal
        self.height = self.matrix.shape[0]
        self.width = self.matrix.shape[1]


        # data structure used in a_star
        self.g_matrix = np.zeros((self.height, self.width)) + 1000000  # save g value for each vertex to infinity
        self.parent_matrix = [[[] for i in range(self.width)] for i in range(self.height)]
        self.fringe = q.PriorityQueue()

    def h_value(self, index):
        x_dis = abs(index[1] - self.goal[1])
        y_dis = abs(index[0] - self.goal[0])
        h_value = math.sqrt(x_dis ** 2 + y_dis ** 2)
        return h_value

    def c(self, s_index, s_prime_index):  # straight line distance
        distance = math.sqrt((s_index[1] - s_prime_index[1]) ** 2 + (s_index[0] - s_prime_index[0]) ** 2)
        return distance

    def if_infringe(self, index):
        temp = []
        while not self.fringe.empty():
            next_vertex_info = self.fringe.get()
            temp.append(next_vertex_info)
            if index == next_vertex_info[2]:
                for vertex_info in temp:
                    self.fringe.put(vertex_info)
                return True
        for vertex_info in temp:
            self.fringe.put(vertex_info)
        return False

    def successor(self, index):
        succ_list = []

        if self.matrix[index[0], index[1]] == 0:
            if index[0] - 1 >= 0 and index[1] - 1 >= 0:
                succ_list.append((index[0] - 1, index[1] - 1))
            if index[0] - 1 >= 0:
                succ_list.append((index[0] - 1, index[1]))
            if index[0] - 1 >= 0 and index[1] + 1 < self.width:
                succ_list.append((index[0] - 1, index[1] + 1))
            if index[1] - 1 >= 0:
                succ_list.append((index[0], index[1] - 1))
            if index[1] + 1 < self.width:
                succ_list.append((index[0], index[1] + 1))
            if index[0] + 1 < self.height and index[1] - 1 >= 0:
                succ_list.append((index[0] + 1, index[1] - 1))
            if index[0] + 1 < self.height:
                succ_list.append((index[0] + 1, index[1]))
            if index[0] + 1 < self.height and index[1] + 1 < self.width:
                succ_list.append((index[0] + 1, index[1] + 1))

        if self.matrix[index[0], index[1]] == 1:
            if index[0] - 1 >= 0 and index[1] - 1 >= 0 \
                    and self.matrix[index[0] - 1, index[1] - 1] != 2 and self.matrix[index[0] - 1, index[1] - 1] != 3:
                succ_list.append((index[0] - 1, index[1] - 1))
            if index[0] - 1 >= 0 \
                    and self.matrix[index[0] - 1, index[1]] != 2 and self.matrix[index[0] - 1, index[1]] != 3:
                succ_list.append((index[0] - 1, index[1]))
            if index[0] - 1 >= 0 and index[1] + 1 < self.width \
                    and self.matrix[index[0] - 1, index[1] + 1] != 2 and self.matrix[index[0] - 1, index[1] + 1] != 3:
                succ_list.append((index[0] - 1, index[1] + 1))
            if index[1] - 1 >= 0 \
                    and self.matrix[index[0], index[1] - 1] != 2 and self.matrix[index[0], index[1] - 1] != 3:
                succ_list.append((index[0], index[1] - 1))
            if index[1] + 1 < self.width \
                    and self.matrix[index[0], index[1] + 1] != 2 and self.matrix[index[0], index[1] + 1] != 3:
                succ_list.append((index[0], index[1] + 1))
            if index[0] + 1 < self.height and index[1] - 1 >= 0 \
                    and self.matrix[index[0] + 1, index[1] - 1] != 2 and self.matrix[index[0] + 1, index[1] - 1] != 3:
                succ_list.append((index[0] + 1, index[1] - 1))
            if index[0] + 1 < self.height \
                    and self.matrix[index[0] + 1, index[1]] != 2 and self.matrix[index[0] + 1, index[1]] != 3:
                succ_list.append((index[0] + 1, index[1]))
            if index[0] + 1 < self.height and index[1] + 1 < self.width \
                    and self.matrix[index[0] + 1, index[1] + 1] != 2 and self.matrix[index[0] + 1, index[1] + 1] != 3:
                succ_list.append((index[0] + 1, index[1] + 1))

        return succ_list

    def find_rout(self):
        self.g_matrix[self.start[0], self.start[1]] = 0  # g(S_start) = 0
        self.parent_matrix[self.start[0]][self.start[1]] = (self.start[0], self.start[1])  # parent(S_start) = S_start
        self.fringe = q.PriorityQueue() # fringe = []
        self.fringe.put((self.g_matrix[self.start[0], self.start[1]] + self.h_value((self.start[0], self.start[1])),
                        - self.g_matrix[self.start[0], self.start[1]],
                        (self.start[0], self.start[1])))  # fringe.Insert(start, g(Sstart) + h(Sstart)
        close_list = []  # closed = []

        while not self.fringe.empty():
            s = self.fringe.get()[2]  # s := fringe.pop()
            if s == self.goal:
                return 'path found'
            close_list.append(s)
            for s_prime in self.successor(s):
                if s_prime not in close_list:
                    if not self.if_infringe(s_prime):
                        self.g_matrix[s_prime[0], s_prime[1]] = 1000000
                        self.parent_matrix[s_prime[0]][s_prime[1]] = []
                    self.update_verex(s, s_prime)
        return 'no path found'

    def update_verex(self, s_index, s_prime_index):
        if self.line_of_sight(self.parent_matrix[s_index[0]][s_index[1]], s_prime_index):
            if (self.g_matrix[self.parent_matrix[s_index[0]][s_index[1]][0], self.parent_matrix[s_index[0]][s_index[1]][1]] + self.c(self.parent_matrix[s_index[0]][s_index[1]], s_prime_index)) < self.g_matrix[s_prime_index[0], s_prime_index[1]]:
                self.g_matrix[s_prime_index[0], s_prime_index[1]] = self.g_matrix[self.parent_matrix[s_index[0]][s_index[1]][0], self.parent_matrix[s_index[0]][s_index[1]][1]] + self.c(
                    self.parent_matrix[s_index[0]][s_index[1]], s_prime_index)
                self.parent_matrix[s_prime_index[0]][s_prime_index[1]] = self.parent_matrix[s_index[0]][s_index[1]]
                if self.if_infringe(s_prime_index):  # remove s_prime from fringe
                    temp = []
                    next_vertex_info = self.fringe.get()
                    temp.append(next_vertex_info)
                    while next_vertex_info[2] != s_prime_index:
                        next_vertex_info = self.fringe.get()
                        temp.append(next_vertex_info)
                    for vertex_info in temp:
                        self.fringe.put(vertex_info)
                self.fringe.put((self.g_matrix[s_prime_index[0], s_prime_index[1]] + self.h_value(
                    (s_prime_index[0], s_prime_index[1])),
                                 - self.g_matrix[s_prime_index[0], s_prime_index[1]],
                                 s_prime_index))
        else:
            if (self.g_matrix[s_index[0], s_index[1]] + self.c(s_index, s_prime_index)) < self.g_matrix[s_prime_index[0], s_prime_index[1]]:
                self.g_matrix[s_prime_index[0], s_prime_index[1]] = self.g_matrix[s_index[0], s_index[1]] + self.c(s_index, s_prime_index)
                self.parent_matrix[s_prime_index[0]][s_prime_index[1]] = s_index
                if self.if_infringe(s_prime_index):  # remove s_prime from fringe
                    temp = []
                    next_vertex_info = self.fringe.get()
                    temp.append(next_vertex_info)
                    while next_vertex_info[2] != s_prime_index:
                        next_vertex_info = self.fringe.get()
                        temp.append(next_vertex_info)
                    for vertex_info in temp:
                        self.fringe.put(vertex_info)
                self.fringe.put((self.g_matrix[s_prime_index[0], s_prime_index[1]] + self.h_value((s_prime_index[0], s_prime_index[1])),
                                - self.g_matrix[s_prime_index[0], s_prime_index[1]],
                                s_prime_index))

    def is_block(self, index):
        if self.matrix[index[0], index[1]] == 2 or self.matrix[index[0], index[1]] == 3:
            return True
        elif self.matrix[index[0] + 1, index[1]] == 2 or self.matrix[index[0] + 1, index[1]] == 3:
            return True
        elif self.matrix[index[0], index[1] + 1] == 2 or self.matrix[index[0], index[1] + 1] == 3:
            return True
        elif self.matrix[index[0] + 1, index[1] + 1] == 2 or self.matrix[index[0] + 1, index[1] + 1] == 3:
            return True
        else:
            return False

    def line_of_sight(self, s_index, s_prime_index):
        #  in this func, x, y is the indices in height, width
        x_0 = s_index[0]
        y_0 = s_index[1]
        x_1 = s_prime_index[0]
        y_1 = s_prime_index[1]
        f = 0
        d_y = y_1 - y_0
        d_x = x_1 - x_0
        if d_y < 0:
            d_y = - d_y
            s_y = -1
            s_y_index = -1  # (sy-1)/2
        else:
            s_y = 1
            s_y_index = 0

        if d_x < 0:
            d_x = - d_x
            s_x = -1
            s_x_index = -1
        else:
            s_x = 1
            s_x_index = 0

        if d_x > d_y:
            while x_0 != x_1:

                f = f + d_y
                if f > d_x:
                    if self.is_block([x_0 + s_x_index, y_0 + s_y_index]):
                        return False
                    y_0 = y_0 + s_y
                    f = f - d_x
                if f != 0 and self.is_block([x_0 + s_x_index, y_0 + s_y_index]):
                    return False
                if d_y == 0 and (self.matrix[x_0, y_0] == 2 or self.matrix[x_0, y_0]) == 3:
                    return False
                x_0 = x_0 + s_x

        else:
            while y_0 != y_1:
                f = f + d_x
                if f > d_y:
                    if self.is_block([x_0 + s_x_index, y_0 + s_y_index]):
                        return False
                    x_0 = x_0 + s_x
                    f = f - d_y
                if f != 0 and self.is_block([x_0 + s_x_index, y_0 + s_y_index]):
                    return False
                if d_x == 0 and (self.matrix[x_0, y_0] == 2 or self.matrix[x_0, y_0] == 3):
                    return False
                y_0 = y_0 + s_y
        return True

    def print_rout(self):

        rout = []
        next_vertex = self.goal
        rout.append(next_vertex)
        while next_vertex != self.start:
            next_vertex = self.parent_matrix[next_vertex[0]][next_vertex[1]]
            rout.insert(0, next_vertex)
        print(rout)
        return rout


if __name__ == '__main__':
    fda_star = FDA_STAR(MATRIX)
    fda_star.find_rout()
    print(fda_star.g_matrix[1, 0])
    # print(a_star.h_value((1, 2)))
    fda_star.print_rout()


