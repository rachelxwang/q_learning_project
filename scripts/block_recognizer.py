#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import keras_ocr, moveit_commander
import dumbbell_recognizer

class BlockRecognizer(object):

    def __init__(self):

        self.initialized = False

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        # subscribe to the robot's scan topic
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # set up publisher and Twist to publish to /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # set up interface to openmanipulator
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # keeps position of each block
        self.blocks = { "1": None, "2": None, "3": None }

        # goal block
        self.goal = None

        # variables for control flow since much of the different logic occurs
        # in call back functions for scan and rgb camera topics
        self.in_range = False
        self.ready_for_image_rec = False
        self.done_with_image_rec = False
        self.done_processing_scan = False
        self.done = False

        # set to True for extra information while debugging
        self.DEBUGGING = False

        self.initialized = True


    # method to call image recognition; note the '1' sometimes is mistaken by the
    # keras_ocr algorithm to be an 'l' or a 't', so these values are also
    # considered to be '1'
    def image_callback(self, data):

        if (not self.initialized):
            return

        if self.ready_for_image_rec and not self.done_with_image_rec:
            # converts the incoming ROS message to cv2 format
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            pipeline = keras_ocr.pipeline.Pipeline()
            print("\nAttempting image recognition...")
            prediction_groups = pipeline.recognize([img])
            print("\nImage recognition found:\n", prediction_groups[0])

            # set block numbers to position based on how many pixels horizonally
            # the recognized character is in the whole image
            for e in prediction_groups[0]:

                if e[1][0][0] < 100:
                    if e[0] == 'l' or e[0] == 't':
                        self.blocks['1'] = "left"
                    else:
                        self.blocks[e[0]] = "left"

                elif e[1][0][0] < 500:
                    if e[0] == 'l' or e[0] == 't':
                        self.blocks['1'] = "middle"
                    else:
                        self.blocks[e[0]] = "middle"

                else:
                    if e[0] == 'l' or e[0] == 't':
                        self.blocks['1'] = "right"
                    else:
                        self.blocks[e[0]] = "right"

            print("\nDetermined blocks to be in this order:", self.blocks)

            if all(b != None for b in self.blocks.values()):
                # done with image recognition, we now know where each block is
                self.done_with_image_rec = True
            else:
                print("try again")
                # try again, causes robot to rotate until finding three blocks
                # again in process_scan and then retry image recognition in
                # image_callback
                self.ready_for_image_rec = False


    # method to put the robot in position for image recognition
    def process_scan(self, data):

        if (not self.initialized):
            return

        # rotate the robot in place until it "sees" all three blocks with the
        # LiDAR scan so that all three numbers can be seen by RBG camera at once
        if not self.ready_for_image_rec and self.in_range:
            if (data.ranges[0] == float("inf")) or (data.ranges[30] == float("inf")) or (data.ranges[330] == float("inf")):
                self.twist.angular.z = 0.2
            else:
                self.ready_for_image_rec = True
                self.twist.angular.z = 0.0

            self.cmd_vel_pub.publish(self.twist)

        # once image processing is over and positions of blocks have been
        # determined, call method to drive to desired block
        if self.done_with_image_rec and not self.done_processing_scan:
            self.drive_to_block()
            self.done_processing_scan = True


    # method to drive to whichever block is specified by the goal
    def drive_to_block(self):

        # if the goal is the left block, rotate to face left block, then
        # drive forwards
        if self.blocks[self.goal] == "left":
            self.twist.angular.z = math.radians(20)
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(2)
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            self.twist.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(16)
            self.twist.linear.x = 0
            self.cmd_vel_pub.publish(self.twist)

        # if the goal is the middle block, simply drive forwards
        if self.blocks[self.goal] == "middle":
            self.twist.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(11)
            self.twist.linear.x = 0
            self.cmd_vel_pub.publish(self.twist)

        # if the goal is the right block, rotate to face right block, then
        # drive forwards
        if self.blocks[self.goal] == "right":
            self.twist.angular.z = math.radians(-20)
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(2)
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            self.twist.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(16)
            self.twist.linear.x = 0
            self.cmd_vel_pub.publish(self.twist)

        self.drop_dumbbell()


    def drop_dumbbell(self):

        # drop dumbbell while backing up so as to not knock the dumbbell over
        self.twist.linear.x = -0.1
        self.cmd_vel_pub.publish(self.twist)

        # move the arm
        arm_joint_goal = [0.0, math.radians(45.0), math.radians(-30.0), math.radians(-5.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop() # prevent any residual movement

        # continue backing up so as to not pick the dumbbell back up
        rospy.sleep(14)

        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)

        # reset arm so the red dot from arm doesn't disturb dumbbell recognition
        self.move_group_arm.go([0.0,0.0,0.0,0.0], wait=True)

        self.done = True


    # move so that all three blocks are within the 3.5m range of the robot's scan;
    # assumes that robot's starting position is where dumbbell_recognizer leaves
    # it after picking up the dumbbell
    def move_blocks_into_range(self):

        self.twist.linear.x = -0.11
        self.cmd_vel_pub.publish(self.twist)

        rospy.sleep(2)

        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)

        self.in_range = True


    # number argument is a string, e.g. to go to block 2, call run("2")
    # call this method to make robot detect and drive to a block
    def run(self, number):

        # ensure initilization
        rospy.sleep(1)

        # reset control flow and set block goal
        self.goal = number
        self.in_range = False
        self.ready_for_image_rec = False
        self.done_with_image_rec = False
        self.done_processing_scan = False
        self.done = False

        self.move_blocks_into_range()

        while not self.done:
            rospy.sleep(1)

# this is for running the node manually for debugging and testing
if __name__ == '__main__':
    node = BlockRecognizer()
    node.run("3")
