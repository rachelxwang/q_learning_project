#!/usr/bin/env python3

import rospy
import numpy as np
import os
from block_recognizer import BlockRecognizer
from dumbbell_recognizer import DumbbellRecognizer

path_prefix = os.path.dirname(__file__) + "/action_states/"

class Controller(object):
    def __init__(self):
        rospy.init_node("controller")

        self.block_recognizer = BlockRecognizer()
        self.dumbbell_recognizer = DumbbellRecognizer()

        self.cur_state = 0

        self.q_matrix = np.genfromtxt(os.path.dirname(__file__) + '/../output/q_matrix.csv', delimiter=',')
        
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))

        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

    def perform_action(self):
        action = np.where(self.q_matrix[self.cur_state] == (max(self.q_matrix[self.cur_state])))

        self.dumbbell_recognizer.run(self.actions[action[0][0]]["dumbbell"])
        self.block_recognizer.run(str(self.actions[action[0][0]]["block"]))

        self.cur_state = np.where(self.action_matrix[self.cur_state] == action[0][0])[0][0]

    def run(self):
        for i in range(3):
            self.perform_action()


if __name__ == "__main__":
    node = Controller()
    node.run()