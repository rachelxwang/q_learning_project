#!/usr/bin/env python3

import rospy
import numpy as np
from q_matrix_learning_project.msgs import QLearningReward, QMatrix, RobotMoveDBTtoBlock

class QMatrix(object):
    def __init__(self):
        rospy.init_node('q_matrix')

        self.is_converged = False
        self.t = 0
        self.cur_state = 0
        self.prev_state = 0
        self.prev_action = 0

        # create empty q_matrix
        self.q_matrix = np.zeros(shape=(64, 9))

        # set reward to -1 if invalid state/action pair, otherwise 0
        for i in range(len(self.q_matrix)):
            for j in range(len(self.q_matrix[0])):
                if self.action_matrix[i][j] == -1:
                    self.q_matrix[i][j] = -1

        # self.q_reward_pub = rospy.Publisher('/q_learning/reward', QLearningReward, queue_size=10)
        self.q_matrix_pub = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
        self.action_pub = rospy.Publisher('/q_learning/robot_action', RobotMoveDBTtoBlock, queue_size=10)

        rospy.Subscriber('/q_learning/reward', QLearningReward, self.handle_reward))

    def converge(self, actions, states, action_matrix):
        while !self.is_converged:
            # select action at random
            action_num = np.random.randint(0, 9)
            # publish action if valid
            action = RobotMoveDBTtoBlock()

            for i range(len(action_matrix[self.cur_state])):
                if action_matrix[self.cur_state][i] == action_num:
                    action.robot_db = actions[action_num]["dumbbell"]
                    action.block_id = actions[action_num]["block"]
                    self.action_pub.publish(action)
                    self.prev_state = self.cur_state
                    self.cur_state = i
                    self.prev_action = action_num
           
            self.t = self.t + 1

    def handle_reward(self, data):
        q_msg = QMatrix()

        self.q_matrix[self.prev_state][self.prev_action] = data.reward + 0.8*(max(q_matrix[self.cur_state]))

        q_msg.q_matrx = self.q_matrix

        self.q_matrix_pub.publish(q_msg)
