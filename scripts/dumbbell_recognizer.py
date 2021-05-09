#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import moveit_commander

class DumbbellRecognizer(object):

    def __init__(self, color):

        self.initialized = False

        rospy.init_node('dumbbell_recognizer')

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
        self.move_group_arm.go([0.0,0.0,0.0,0.0], wait=True)

        # the color of the dumbbell the turtlebot wants to identify and approach
        self.color_goal = color

        # variable to determine whether the robot should be moving
        # used to keep the robot motionless while moving the arm
        self.stop = True

        # set to True for extra information while debugging
        self.DEBUGGING = False

        self.initialized = True


    # this may or may not be useful
    def change_color(self, color):

        self.color_goal = color


    # helper method to get the color mask
    def get_mask(self, hsv):

        if (self.color_goal == 'green'):
            return cv2.inRange(hsv, (40, 40, 40), (70, 255, 255))

        if (self.color_goal == 'blue'):
            return cv2.inRange(hsv, (100, 150, 0), (140, 255, 255))

        if (self.color_goal == 'red'):
            mask1 = cv2.inRange(hsv, (0,50,20), (5,255,255))
            mask2 = cv2.inRange(hsv, (175,50,20), (180,255,255))
            return cv2.bitwise_or(mask1, mask2)


    # method for identifying dumbbells and angular movement
    def image_callback(self, data):

        if (not self.initialized):
            return

        # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = self.get_mask(hsv)

        # using moments() function, determine the center of the colored pixels
        M = cv2.moments(mask)

        if M['m00'] > 0:
            # determine the center of the colored pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            err = (image.shape[1] / 2) - cx
            k_p = 0.0018

            if not self.stop:
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)

            if self.DEBUGGING:
                # visualize a white circle in our debugging window to indicate
                # the center point of the colored pixels
                cv2.circle(image, (cx, cy), 10, (255,255,255), -1)
                cv2.imshow('window', image)
                cv2.waitKey(0)


    # method for linear movement towards the dumbell and stopping 0.25m from it
    def process_scan(self, data):

        if (not self.initialized):
            return

        if data.ranges[0] >= 0.25 and not self.stop:
            # Go forward if not close enough to dumbell.
            self.twist.linear.x = 0.1
        else:
            # Close enough to dumbbell, stop.
            self.stop = True
            self.twist.linear.x = 0

        # Publish msg to cmd_vel.
        self.cmd_vel_pub.publish(self.twist)


    # method to extend arm so that the robot can simply drive towards the
    # dumbbell to position the dumbbell in between the grabber
    def extend_arm(self):

        if (not self.initialized):
            return

        # Move the arm
        arm_joint_goal = [0.0, math.radians(45.0), math.radians(-30.0), math.radians(-5.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop() # prevent any residual movement

        # Move the gripper
        gripper_joint_goal = [0.01,0.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop() # prevent any residual movement

        self.stop = False


    # method to lift arm once the dumbbell is in position between the grabber
    def lift_arm(self):

        if (not self.initialized):
            return

        arm_joint_goal = [0.0, math.radians(0.0), math.radians(-10.0), math.radians(-35.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop() # prevent any residual movement


    def run(self):
        # sleep to ensure initilization happens before moving the arm
        rospy.sleep(1)

        # extend arm to the position for grabbing dumbbell
        self.extend_arm()

        # run until robot has finished identifying and moving to dumbbell
        while not self.stop:
            rospy.sleep(1)

        # lift the arm to pick up the dumbbell
        self.lift_arm()


# this is for running the node manually for debugging and testing
# TODO: deleted later once controller is implemented
if __name__ == '__main__':
    node = DumbbellRecognizer('red')
    node.run()
