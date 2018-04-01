## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/reference_frames.png
[image5]: ./misc_images/links_joints.png
[image6]: ./misc_images/urdf_ref_frames.png
[image7]: ./misc_images/SphericalWrist.jpg
[image8]: ./misc_images/ik_1.png
[image9]: ./misc_images/ik_2.png
[image10]: ./misc_images/ik_3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

This is the schematic showing the links, joints, and common normals only. Rather than try to draw this
by hand, I found it more efficient to simply screen-shot this from the lesson material (it's hard enough to "follow"
even with such a 'clean' drawing; doing it by hand made no sense since I'd more or less just end up copying this
file).

NOTE: all DH parameter tables use JJ Craig (2005) conventions 
(Craig, JJ. (2005). Introduction to Robotics: Mechanics and Control, 3rd Ed (Pearson Education, Inc., NJ))

Figure 1 - Robot Schematic
![alt_text][image5]

Figure 2 - Design of links, joints, axes, and origins based on assignment using Craig conventions WITHOUT respect to URDF
![alt_text][image4]

Figure 3 - Definition of axes & origins AS DEFINED IN URDF (note variance from diagram above; URDF origins shown as black triangles)
![alt text][image6]


Next step is to derive the DH parameter table as described in the lesson. The completed table is below:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | theta(1)
2 | -90 degrees | 0.35 | 0 | theta(2)-90
3 | 0 | -1.25 | 1.5 | theta(3)
4 |  -90 degrees | -0.054 | 0 | theta(4)
5 | 90 degrees | 0 | 0 | theta(5)
6 | -90 degrees | 0 | 0 | theta(6)
7 | 0 | 0 | 0.3 | theta(7)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
Inverse Kinematics goal is the opposite of Forward Kinematics. In this case, the position and orientation of the end-effector is known and the goal is to calculate
the joint angles of the arm of the robot. In this case, a closed-form solution is used for performance. In order to use the closed-form solution there are two key requirements
of the robot:
1.  3 neighboring joint axes must intersect at a single point (forming a spherical wrist)
2.  3 neighboring joint axes are parallel

The Kuka Arm satisfies the above conditions and thus closed-form is possible.

![image8]


![image9]


![image10]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code follows the process described above. Please see the code for detailed discussion.


