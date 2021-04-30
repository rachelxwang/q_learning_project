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
Monday 5/10: Finish project components for robot manipulation & movement
* Tuesday 5/11: Complete writeup, Partner Contributions Survey, record gif and rosbag
* Wednesday 5/12: Project Due 11am CT


## Writeup

TBD
