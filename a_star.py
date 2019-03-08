import numpy as np
import matplotlib.pyplot as plt
import queue as q
import math

MATRIX = np.zeros((3, 5))
MATRIX[0, 1:3] = 1
MATRIX[1, 1:5] = 1
MATRIX[2, 3:5] = 1



class A_STAR():

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
        # self.succ_matrix = [
        #     [[(1, 0), (0, 1)], [], [], [], []],
        #     [],
        #     [],
        # ]

    def h_value(self, index):
        x_dis = abs(index[1] - self.goal[1])
        y_dis = abs(index[0] - self.goal[0])
        h_value = math.sqrt(2) * min(x_dis, y_dis) + max(x_dis, y_dis) - min(x_dis, y_dis)
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
                    and self.matrix[index[0] - 1, index[1] - 1] != 2 and self.matrix[index[0] - 1, index[1] - 1] != 3 \
                    and self.matrix[index[0] - 1, index[1]] != 2 and self.matrix[index[0] - 1, index[1]] != 3 \
                    and self.matrix[index[0], index[1] - 1] != 2 and self.matrix[index[0], index[1] - 1] != 3:
                succ_list.append((index[0] - 1, index[1] - 1))
            if index[0] - 1 >= 0 \
                    and self.matrix[index[0] - 1, index[1]] != 2 and self.matrix[index[0] - 1, index[1]] != 3:
                succ_list.append((index[0] - 1, index[1]))
            if index[0] - 1 >= 0 and index[1] + 1 < self.width \
                    and self.matrix[index[0] - 1, index[1] + 1] != 2 and self.matrix[index[0] - 1, index[1] + 1] != 3 \
                    and self.matrix[index[0] - 1, index[1]] != 2 and self.matrix[index[0] - 1, index[1]] != 3 \
                    and self.matrix[index[0], index[1] + 1] != 2 and self.matrix[index[0], index[1] + 1] != 3:
                succ_list.append((index[0] - 1, index[1] + 1))
            if index[1] - 1 >= 0 \
                    and self.matrix[index[0], index[1] - 1] != 2 and self.matrix[index[0], index[1] - 1] != 3:
                succ_list.append((index[0], index[1] - 1))
            if index[1] + 1 < self.width \
                    and self.matrix[index[0], index[1] + 1] != 2 and self.matrix[index[0], index[1] + 1] != 3:
                succ_list.append((index[0], index[1] + 1))
            if index[0] + 1 < self.height and index[1] - 1 >= 0 \
                    and self.matrix[index[0] + 1, index[1] - 1] != 2 and self.matrix[index[0] + 1, index[1] - 1] != 3 \
                    and self.matrix[index[0] + 1, index[1]] != 2 and self.matrix[index[0] + 1, index[1]] != 3 \
                    and self.matrix[index[0], index[1] - 1] != 2 and self.matrix[index[0], index[1] - 1] != 3:
                succ_list.append((index[0] + 1, index[1] - 1))
            if index[0] + 1 < self.height \
                    and self.matrix[index[0] + 1, index[1]] != 2 and self.matrix[index[0] + 1, index[1]] != 3:
                succ_list.append((index[0] + 1, index[1]))
            if index[0] + 1 < self.height and index[1] + 1 < self.width \
                    and self.matrix[index[0] + 1, index[1] + 1] != 2 and self.matrix[index[0] + 1, index[1] + 1] != 3 \
                    and self.matrix[index[0] + 1, index[1]] != 2 and self.matrix[index[0] + 1, index[1]] != 3 \
                    and self.matrix[index[0], index[1] + 1] != 2 and self.matrix[index[0], index[1] + 1] != 3:
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

    def print_rout(self):

        rout = []
        next_vertex = self.goal
        rout.append(next_vertex)
        while next_vertex != self.start:
            next_vertex = self.parent_matrix[next_vertex[0]][next_vertex[1]]
            rout.insert(0, next_vertex)
        print(rout)
        return rout

    # def print_grid_world(self):
    #     fig, ax = plt.subplots(figsize=(11, 11))
    #     plt.plot(np.append(np.array(self.obstacles[0])[:, 0], np.array(self.obstacles[0])[0, 0]),
    #              np.append(np.array(self.obstacles[0])[:, 1], np.array(self.obstacles[0])[0, 1]), label="obstacles1")
    #     plt.plot(np.append(np.array(self.obstacles[1])[:, 0], np.array(self.obstacles[1])[0, 0]),
    #              np.append(np.array(self.obstacles[1])[:, 1], np.array(self.obstacles[1])[0, 1]), label="obstacles2")
    #     plt.plot(np.append(np.array(self.obstacles[2])[:, 0], np.array(self.obstacles[2])[0, 0]),
    #              np.append(np.array(self.obstacles[2])[:, 1], np.array(self.obstacles[2])[0, 1]), label="obstacles3")
    #
    #     # change block cells to black
    #     for i in range(len(self.matrix)):
    #         for j in range(len(self.matrix[0])):
    #             if self.matrix[i][j] == 3:
    #                 # paint all cells with this vertex to black
    #                 x_value = self.x_min + j * self.cell_length
    #                 y_value = self.y_max - i * self.cell_length
    #                 ax.broken_barh([(x_value, self.cell_length)],
    #                                (y_value, self.cell_length), facecolor='gray')
    #                 ax.broken_barh([(x_value - self.cell_length, self.cell_length)],
    #                                (y_value, self.cell_length), facecolor='gray')
    #                 ax.broken_barh([(x_value - self.cell_length, self.cell_length)],
    #                                (y_value - self.cell_length, self.cell_length), facecolor='gray')
    #                 ax.broken_barh([(x_value, self.cell_length)],
    #                                (y_value - self.cell_length, self.cell_length), facecolor='gray')
    #     for i in range(len(self.matrix)):
    #         for j in range(len(self.matrix[0])):
    #             if self.matrix[i][j] == 2:
    #                 # paint all cells with this vertex to black
    #                 x_value = self.x_min + j * self.cell_length
    #                 y_value = self.y_max - i * self.cell_length
    #                 ax.broken_barh([(x_value, self.cell_length)],
    #                                (y_value, self.cell_length), facecolor='black')
    #                 ax.broken_barh([(x_value - self.cell_length, self.cell_length)],
    #                                (y_value, self.cell_length), facecolor='black')
    #                 ax.broken_barh([(x_value - self.cell_length, self.cell_length)],
    #                                (y_value - self.cell_length, self.cell_length), facecolor='black')
    #                 ax.broken_barh([(x_value, self.cell_length)],
    #                                (y_value - self.cell_length, self.cell_length), facecolor='black')
    #
    #
    #     x_major_ticks = np.arange(self.x_min, self.x_max + self.cell_length, 0.5)
    #     x_minor_ticks = np.arange(self.x_min, self.x_max + self.cell_length, 0.25)
    #     y_major_ticks = np.arange(self.y_min, self.y_max + self.cell_length, 0.5)
    #     y_minor_ticks = np.arange(self.y_min, self.y_max + self.cell_length, 0.25)
    #     ax.set_xticks(x_major_ticks)
    #     ax.set_xticks(x_minor_ticks, minor=True)
    #     ax.set_yticks(y_major_ticks)
    #     ax.set_yticks(y_minor_ticks, minor=True)
    #
    #     # And a corresponding grid
    #     ax.grid(which='both')
    #
    #     # Or if you want different settings for the grids:
    #     ax.grid(which='minor', alpha=0.2)
    #     ax.grid(which='major', alpha=0.5)
    #
    #     plt.show()



if __name__ == '__main__':
    a_star = A_STAR(MATRIX)
    a_star.find_rout()
    print(a_star.g_matrix[1, 0])
    # print(a_star.h_value((1, 2)))
    a_star.print_rout()


