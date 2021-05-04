#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

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

        # the color of the dumbbell the turtlebot wants to identify and approach
        self.color_goal = color

        # a variable to know when it has gotten close enough to the dumbbell
        self.stop = False

        # set to True for extra information while debugging
        self.DEBUGGING = False

        self.initialized = True


    def change_color(self, color):

        self.color_goal = color


    def get_mask(self, hsv):

        if (self.color_goal == 'green'):
            return cv2.inRange(hsv, (40, 40, 40), (70, 255, 255))

        if (self.color_goal == 'blue'):
            return cv2.inRange(hsv, (100, 150, 0), (140, 255, 255))

        if (self.color_goal == 'red'):
            mask1 = cv2.inRange(hsv, (0,50,20), (5,255,255))
            mask2 = cv2.inRange(hsv, (175,50,20), (180,255,255))
            return cv2.bitwise_or(mask1, mask2)


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
            k_p = 0.005

            if not self.stop:
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)

            if self.DEBUGGING:
                # visualize a white circle in our debugging window to indicate
                # the center point of the colored pixels
                cv2.circle(image, (cx, cy), 10, (255,255,255), -1)
                cv2.imshow('window', image)
                cv2.waitKey(0)


    def process_scan(self, data):

        if (not self.initialized):
            return

        if data.ranges[0] >= 0.35 and not self.stop:
            # Go forward if not close enough to wall.
            self.twist.linear.x = 0.1
        else:
            # Close enough to wall, stop.
            self.stop = True
            self.twist.linear.x = 0

        # Publish msg to cmd_vel.
        self.cmd_vel_pub.publish(self.twist)


    def run(self):
        # run for 10 seconds to give robot time to identify and move to dumbbell
        rospy.sleep(10)


if __name__ == '__main__':
    node = DumbbellRecognizer('green')
    node.run()
