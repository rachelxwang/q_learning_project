#!/usr/bin/env python3

import rospy
import numpy as np
import random
from q_learning_project.msg import QLearningReward, QMatrix, QMatrixRow, RobotMoveDBToBlock

class QMatrix_init(object):
    def __init__(self):
        # need to maintain whether the matrix is converged
        # also need to keep track of states and actions
        self.is_converged = False
        self.cur_state = 0
        self.prev_state = 0
        self.prev_action = 0

        # create empty q_matrix and convergence_matrix
        self.q_matrix = np.zeros(shape=(64, 9))
        self.convergence_matrix = np.zeros(shape=(64, 9))

        # create publishers
        self.q_matrix_pub = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
        self.action_pub = rospy.Publisher('/q_learning/robot_action', RobotMoveDBToBlock, queue_size=10)

        # create subscriber
        rospy.Subscriber('/q_learning/reward', QLearningReward, self.handle_reward)

    def init_matrices(self, actions, states, action_matrix):
        self.actions = actions
        self.states = states
        self.action_matrix = action_matrix

    # method to select and publish an action for convergence
    def converge(self):
        possible_actions = {}
        index = []
        action = []

        # identify possible actions from current state
        for i in range(len(self.action_matrix[self.cur_state])):
            if self.action_matrix[self.cur_state][i] != -1:
                index.append(i)
                action.append(self.action_matrix[self.cur_state][i])

        for i in range(len(index)):
            possible_actions[index[i]] = action[i]

        # if no possible actions return to 0- this might need to be changed
        if len(possible_actions) == 0:
            self.cur_state = 0
            self.converge()
            return

        # choose random action
        chosen_action = random.choice(list(possible_actions.items()))

        # create action and publish
        action = RobotMoveDBToBlock()
        action.robot_db = self.actions[chosen_action[1].astype(np.int64)]["dumbbell"]
        action.block_id = self.actions[chosen_action[1].astype(np.int64)]["block"]

        self.action_pub.publish(action)

        # update states and action
        self.prev_state = self.cur_state
        self.cur_state = chosen_action[0]
        self.prev_action = chosen_action[1].astype(np.int64)

    # method that updates the q_matrix dependent on reward received
    def handle_reward(self, data):
        print(data)
        q_msg = QMatrix()
        q_rows = []
        sum = 0
        new_val = 0

        # calculate new value for q_matrix
        if (max(self.q_matrix[self.cur_state])) != -1:
            new_val = int(data.reward + 0.8 * max(self.q_matrix[self.cur_state]))
        else:
            new_val = int(data.reward)

        # test whether the new value updates the q_matrix
        # this is used to test for convergence
        if (self.q_matrix[self.prev_state][self.prev_action] != new_val):
            self.q_matrix[self.prev_state][self.prev_action] = new_val
            self.convergence_matrix[self.prev_state][self.prev_action] = 0
        else:
            self.convergence_matrix[self.prev_state][self.prev_action] = 1
            # if we aren't updating the q_matrix test if we have converged
            for row in self.convergence_matrix:
                for item in row:
                    if item == 0:
                        pass
                    else:
                        sum += 1

        # print(self.q_matrix)
        if (sum == 576):
            self.is_converged = True
            return
        else:
            self.is_converged = False

        

        # create q_matrix message to publish
        for i in range(len(self.q_matrix)):
            q_rows.append(QMatrixRow())
            q_rows[i].q_matrix_row = self.q_matrix[i].astype(int).tolist()

        q_msg.q_matrix = q_rows

        self.q_matrix_pub.publish(q_msg)

        self.converge()
