#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv
from q_matrix import QMatrix_init
import time

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        self.q_matrix = np.zeros(shape=(64, 9))

        
    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        with open(os.path.dirname(__file__) + '/../output/q_matrix.csv', mode='w') as q_file:
            q_writer = csv.writer(q_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for row in self.q_matrix:
                q_writer.writerow(row)

    def run(self):
        q_matrix_node = QMatrix_init()

        # initialize the q_matrix and convergence_matrix with invalid options
        for i in range(len(q_matrix_node.q_matrix)):
            for j in range(len(q_matrix_node.q_matrix[0])):
                q_matrix_node.q_matrix[i][j] = -1
                q_matrix_node.convergence_matrix[i][j] = -1

        for i in range(len(q_matrix_node.q_matrix)):
            for j in range(len(q_matrix_node.q_matrix)):
                if self.action_matrix[i][j] != -1:
                    q_matrix_node.q_matrix[i][(self.action_matrix[i][j]).astype(np.int64)] = 0
                    q_matrix_node.convergence_matrix[i][(self.action_matrix[i][j]).astype(np.int64)] = 0

        print(q_matrix_node.q_matrix)

        time.sleep(1)

        # go until q_matrix is converged and then write it to a file
        q_matrix_node.init_matrices(self.actions, self.states, self.action_matrix)
        q_matrix_node.converge()

        while not q_matrix_node.is_converged:
            rospy.sleep(1)
        
        self.q_matrix = q_matrix_node.q_matrix
        
        self.save_q_matrix()

if __name__ == "__main__":
    node = QLearning()
    node.run()
