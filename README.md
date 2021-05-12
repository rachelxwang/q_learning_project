# Q-Learning Project

## Implementation Plan

**Team Members**: Alec Blagg and Rachel Wang

* **Q-learning algorithm**
  * **Executing the Q-learning algorithm:** We will create a new python ROS node in which there will be a method where we will translate the Q-learning algorithm pseudo code from the website into real code. We will implement and use the `save_q_matrix` function to prevent retraining. To test this we will use the two given nodes mentioned in the “Iterating through the Q-Learning Algorithm” that enable quick iteration of the Q-learning algorithm so that we can do a sanity check on the results.

  * **Determining when the Q-matrix has converged:** We will create a secondary matrix of the same size and shape as the Q-matrix which will store integers representing the number of iterations since that index was last modified. This is based on the system mentioned in lecture when discussing the problem of convergence. To test this we will look at implementing varying thresholds for how many iterations need to have occurred to be considered converged.

  * **Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward:** Considering the robot’s current state, we will identify the index in the matrix that has the highest value for this state (i.e. the column with the highest value for the current state’s row), and take the corresponding action associated with this index/value. To test this we can use the fact that we will be using a Q-matrix stored in a file, meaning we can observe what actions have the highest value in certain states, which we can place the robot in to observe its behavior and ensure it matches with what is expected.

* **Robot perception**

    * **Determining the identities and locations of the three colored dumbbells:** To determine the identities and locations of the three dumbbells we will use a combination of the `/scan` and `/camera/rgb/image_raw` ROS topics, as well as some of the techniques used in the line follower assignment. To test this we will try to direct the robot to move to a specific colored dumbbell and visually confirm that its behavior is as expected.

    * **Determining the identities and locations of the three numbered blocks:** For digit recognition we will use the keras_ocr library as mentioned in the project documentation. This will be combined with the /scan data which can provide information on distance and angles. This will be tested in a similar way as the dumbbell recognition.

* **Robot manipulation & movement**

    * **Picking up and putting down the dumbbells with the OpenMANIPULATOR arm:** We will use the ideas discussed in lecture 8 about the manipulator, and more specifically the GUIs discussed to identify the positions necessary to enable picking up the dumbbells. After this we can use the Moveit library to programmatically perform this action. To test this we will manually the robot in front of the dumbbells and attempt to have it pick them up.

    * **Navigating to the appropriate locations to pick up and put down the dumbbells:**
    Using the information gathered from the previous perception step(s), we will be able to locate both the dumbbells as well as the blocks. To drop the dumbbell off we can use the code previously used to stop the robot in front of a wall to get it to stop in front of the block, followed by implementing a program to put the dumbbell down. To pick it up we can use the ideas from the line follower to keep the dumbbell lined up with the grabber until the robot gets close enough to pick it up. This will also be tested visually.    

**Timeline**
* Monday 5/3: Finish project components for Q-learning algorithm
* Friday 5/7: Finish project components for robot perception
* Monday 5/10: Finish project components for robot manipulation & movement
* Tuesday 5/11: Complete writeup, Partner Contributions Survey, record gif and rosbag
* Wednesday 5/12: Project Due 11am CT


## Writeup

gif TBD

### Objectives Description

TBD

### High-level Description

TBD

### Q-Learning Algorithm Description

TBD

### Robot Perception Description

  * **Identifying the locations and identities of each of the colored dumbbells**

  Located within `image_callback()` within the `DumbbellRecognizer` class. We used the strategies from the in class line follower exercise to distinguish the colored dumbbells (cv2 mask, etc). If the robot does not immediately detect any dumbbell of the specified color, it will rotate in place until it does. We used [this](https://stackoverflow.com/questions/51229126/how-to-find-the-red-color-regions-using-opencv) Stack Overflow question to help set the HSV thresholds for each color.

  * **Identifying the locations and identities of each of the numbered blocks**

  Located primarily within `image_callback()` within the `BlockRecognizer` class. We used keras_ocr library as recommended by the project writeup. The robot will first put itself into a location and orientation so that it can "see" all three blocks at once (in `move_blocks_into_range()` and parts of `process_scan()`). Then it will do keras_ocr image processing once, and since it can see all three blocks at once, it records which numbered block is in which position.

### Robot Manipulation and Movement

  * **Moving to the right spot in order to pick up a dumbbell**

  Located within `image_callback()` and `process_scan()` within the `DumbbellRecognizer` class. We used strategies from the in class line follower and stop at wall exercises to move to the right spot to pick up a dumbbell. The angular movement is determined by using PID control to move the center of the colored pixels to the center of the RGB camera image in `image_callback()`. The linear movement is essentially the same as the stop at wall exercise in `process_scan()`.

  * **Picking up the dumbbell**

  Located within `extend_arm()` and `lift_arm()` within the `DumbbellRecognizer` class. After identifying the dumbbell but before approaching it, the robot will extend its arm and open the gripper so that the gripper is at a height to grab the middle part of the dumbbell. Then after driving forward and positioning the middle part of the dumbbell within the open gripper, the robot will lift its arm so that the gripper catches onto the larger top part of the dumbbell and picks up the dumbbell.

  * **Moving to the desired destination (numbered block) with the dumbbell**

  Located within `drive_to_block()` within the `BlockRecognizer` class. Since the robot knows the positions of all three blocks from the image processing, it can determine whether it needs to go to the left, middle, or right block. If it needs to go to the left or right block, then it will rotate approximately 40 degrees counterclockwise or clockwise respectively and then drive forwards about 2.8m to be roughly in front of the block. If it needs to go to the middle block, then it will simply drive forwards about 2.2m to be in front of the block. Since all three dumbbells are in roughly the same place relative to the blocks, it does not matter which dumbbell the robot has just picked up. Note: Pouya said not to "hard code" block locations on Slack, but Sarah said during our meeting with her that this strategy of knowing the distance the robot needs to drive is ok.

  * **Putting the dumbbell back down at the desired destination**

  Located within `drop_dumbbell()` within the `BlockRecognizer` class. The robot will simultaneously move backwards and return the arm to the extended position from before (picking up dumbbell). This will cause the robot to put the dumbbell down.

### Challenges

One challenge we faced was that there was a lot of noise in the environment (e.g. slight disturbances in the movement towards the dumbbell would make the robot fail to position the dumbbell between the gripper and would make it knock the dumbbell over). We overcame this challenge by decreasing all linear and angular velocities. While this made our robot's movements extremely slow, it minimized the effects of noise and helped us distinguish when bugs were occurring due to noise from implementation issues. Another challenge that we faced was that debugging was really slow and frustrating due to needing to restart the Gazebo and the fact that our robot moved quite slowly. This isn't a challenge that can be overcome per se, but we did learn to be a lot more patient while working.

### Future Work

TBD

### Takeaways

  * One takeaway was realizing the differences between ROS nodes and OOP classes. Initially I thought that the two were roughly equivalent, but later realized that nodes could not access functions of other nodes like classes typically can. Sarah had said that our only options were to either combine all perception and movement into one node, i.e. one class, or to create custom ROS messages to communicate between nodes. We found a workaround that was not either of these solutions. We found that one ROS node can access the methods of another class that is not a ROS node. For example, the `DumbbellRecognizer` class cannot be run on its own as it is not a ROS node but needs to publish msgs, but the `Controller` class can create an instance of `DumbbellRecognizer` and since `Controller` is a ROS node, then it can run the `DumbbellRecognizer` methods (which include publishing to `/cmd_vel` for example). This allowed us to separate code into smaller logical pieces and avoid having huge classes with many functions. This allows for easier testing of specific parts and for better readability.

  * Another takeaway TBD
