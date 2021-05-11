#!/usr/bin/env python3

import rospy
import numpy as np
import random
from q_learning_project.msg import QLearningReward, QMatrix, RobotMoveDBToBlock

class QMatrix_init(object):
    def __init__(self):
        self.is_converged = False
        self.t = 0
        self.cur_state = 0
        self.prev_state = 0
        self.prev_action = 0

        # create empty q_matrix
        self.q_matrix = np.zeros(shape=(64, 9))
        self.convergence_matrix = np.zeros(shape=(64, 9))

        # self.q_reward_pub = rospy.Publisher('/q_learning/reward', QLearningReward, queue_size=10)
        self.q_matrix_pub = rospy.Publisher('/q_learning/q_matrix', QMatrix, queue_size=10)
        self.action_pub = rospy.Publisher('/q_learning/robot_action', RobotMoveDBToBlock, queue_size=10)

        rospy.Subscriber('/q_learning/reward', QLearningReward, self.handle_reward)

    def converge(self, actions, states, action_matrix):
        # select action at random
        possible_actions = {}
        index = []
        action = []

        for i in range(len(action_matrix[self.cur_state])):
            if action_matrix[self.cur_state][i] != -1:
                index.append(i)
                action.append(action_matrix[self.cur_state][i])

        for i in range(len(index)):
            possible_actions[index[i]] = action[i]

        if len(possible_actions) == 0:
            self.cur_state = 0
            return

        chosen_action = random.choice(list(possible_actions.items()))

        # publish action if valid
        action = RobotMoveDBToBlock()

        action.robot_db = actions[chosen_action[1].astype(np.int64)]["dumbbell"]
        action.block_id = actions[chosen_action[1].astype(np.int64)]["block"]
        self.action_pub.publish(action)
        self.prev_state = self.cur_state
        self.cur_state = chosen_action[0]
        self.prev_action = chosen_action[1].astype(np.int64)

    def handle_reward(self, data):
        print("reward")
        q_msg = QMatrix()
        q_rows = []
        sum = 0

        new_val = data.reward + 0.8 * max(self.q_matrix[self.cur_state])

        if (self.q_matrix[self.prev_state][self.prev_action] != new_val):
            self.q_matrix[self.prev_state][self.prev_action] = new_val
            self.convergence_matrix[self.prev_state][self.prev_action] = 0
        else:
            self.convergence_matrix[self.prev_state][self.prev_action] = 1
            for row in self.convergence_matrix:
                for item in row:
                    if item == 0:
                        continue
                    else:
                        sum += 1
        print(sum)
        if (sum == 576):
            self.is_converged = True
        else:
            self.is_converged = False

        for i in range(len(self.q_matrix)):
            q_rows.append(self.q_matrix[i])

        q_msg.q_matrix = q_rows

        self.q_matrix_pub.publish(q_msg)
