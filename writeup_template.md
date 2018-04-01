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


Next step is to begin deriving the DH parameter table. To begin this process, I first resolved the easiest entries per the following "rules"
around special cases involving the lines from reference frame Z(i-1) to Z(i)
a.  Collinear lines will yield alpha = 0 and a = 0
b.  Parallel lines will yield alpha = 0 and a <> 0
c.  Intersecting lines will yield alpha <>0 and a = 0
d.  If the common normal intersects Z-hat(i) at the origin of frame i, then:
    If the joint is a revolute, d(i) = 0
    If the joint is prismatic, then theta(i) = 0
    

The initial DH table after applying the above is as follows (based on Figure 2 above):

i | alpha(i-1) | a(i-1) | d(i) | theta(i) | note
--- | --- | --- | --- | --- | ---
1 | 0 | 0 | ? | ? | case a above
2 | ? | ? | 0 | ? | case b above
3 | 0 | 0 | 0 | ? | case a and d (revolute) above
4 | 0 | ? | ? | ? | case c above
5 | 0 | ? | ? | ? | case c above
6 | 0 | ? | ? | ? | case c above
7 (G) | 0 | 0 | ? | 0 | case a and d (fixed, which in this case acts like a prismatic set to 0) above

Next, we will examine the origins and axes in the URDF file (Figure 3) as compared to the origins and axes assigned using Craig conventions
(Figure 2).

i | alpha(i-1) | a(i-1) | d(i) | theta(i) | note
--- | --- | --- | --- | --- | ---
1 | 0 | 0 | ? | ? | case a above
2 | ? | ? | 0 | ? | case b above
3 | 0 | 0 | 0 | ? | case a and d (revolute) above
4 | 0 | ? | ? | ? | case c above
5 | 0 | ? | ? | ? | case c above
6 | 0 | ? | ? | ? | case c above
7 (G) | 0 | 0 | ? | 0 | case a and d (fixed, which in this case acts like a prismatic set to 0) above

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

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


