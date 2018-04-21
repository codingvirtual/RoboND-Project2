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
[image11]: ./misc_images/theta_angles_diagram.png

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

With respect to the 2 diagrams above, a key thing to understand is that the first diagram
represents our choice as engineers of how to define the various reference frames needed
to do the related math and the 2nd diagram depicts the way the reference frames are built in the URDF file. Part
of the challenge in building the DH table is to essentially blend the two together. You'll need to understand
how the frame origins in the URDF file differ from those of your theoretical design so that you can properly
populate the modified DH table.

Next step is to derive the modified DH parameter table as described in the lesson. The completed table is below:

Joint (i) | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | -pi/2 | 0.35 | 0 | q2 - pi/2
3 | 0 | 1.25 | 0 | q3
4 |  -pi/2 | -.054 | 1.5 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
7 | 0 | 0 | 0.303 | 0

To derive the table above, you must step through each joint to build the table row by row. Details follow.

Joint 1
First, the link from the base to my reference origin for joint 1 is examined.
Since the base and the joint have coincident Z axes, alpha will be zero since there is no twist angle between those
two Z axes and a will also be zero since they are coincident.
In the URDF file, there are two lines that define the d value we are looking for. Recall that d represents the
displacement of the point along the Z axis. In this case, the URDF file defines the distance from the base to the
center of Joint 1 as .33 and the distance from the center of Joint 1 to the reference frame origin as .42. You have
to add those two values together (which equals .75) to find the value of d as it relates to my choice of reference
frame origin (see Figure 2 - the reference origin is above the joint, not in the middle of it).

To see this, review Figure 3 where you'll note the black triangle in the middle of Joint 1. That's the reference frame
that the URDF file is using. As mentioned above, I chose the origin to be "above" that joint along the common normal between
Joint 1 and Joint 2, which is to say along X1. So, to figure out how "high" that is, I had to add those two values
together from the URDF file.

Finally, since Joint 1 is a revolute joint, theta(1) remains in the table representing the variable joint revolution angle
and is represented by q1.

Joint 2
Next, the link from Joint 1 to Joint 2 is considered.

First, the Z axes for Joint 2 is rotated relative to joint 1. In the right-hand sense of this rotation, that would
be a clockwise-rotation which will be represented as a negative value. Z2 is rotated exactly 90 degrees from Z1,
so this yields alpha of -90 degrees (mathematically, -pi/2). Next we must consider the value of a. The URDF file 
specifies that this distance is .35 (the x value in the URDF file for this joint), so a = .35. Since the origin lies 
along x1 and is not displaced along Z1, the displacement is of course 0. Finally, this joint is again a revolute joint, 
so theta will be variable depending on the joint's rotation. Since there is a rotation of x2 relative to x1, we need
to compensate for this on this row of the table. Hence, we take the joint's rotation angle and subtract -pi/2 and
represent this as q2.

Joint 3
Now consider the link between Joint 2 & Joint 3. First, the Z axes are parallel (Z2 & Z3). This is going to yield a twist angle
of zero (alpha). The URDF file defines that X3 for this Joint is 1.25m displaced along X2. This is copied to the table
for a. Since this joint's origin lies along X2, there is no displacement along x2 and hence d = 0. Finally, since
this joint is again a revolute, theta is variable and defined as q3.

Joint 4
Note that for Joints 4 & 6, we want to consider 4, 5, and 6 as a spherical wrist. As a result, in the "theoretical"
joint definitions (Figure 2), you'll note that the origins for all three of these joints are the same point, which 
happens to be the center of Joint 5. In contrast, you'll note in Figure 3 that the URDF defines the origins of these
joints differently. Here we will need to take that into account in our DH table parameters.

To determine the origin for 4 with respect to Diagram 2, we will need to consider the URDF values for both Joints 4 & 5.
Since Joint 4 is a revolute, it's Z axis must point along the axis of rotation. Thus, Z4 points to the right in the
diagram (as does Z6). Z4 is rotated with respect to Z3 (by 90 degrees clockwise), so by the right-hand rule, we
set alpha to -pi/2.
To find the horizontal translation for this origin, we will need to add the X values for both
Joints 4 & 5. This is because the URDF file defines the origin of Joint 4 as the center of Joint 4 where I defined
it as the center of Joint 5 for the ease of the math. When you add the .96 and .54 from the URDF files, you get
the 1.5 value you see in the table for a. Similarly, since this joint is slightly lower than Joint 3, you'll note
the URDF file shows the Y value as -.054, indicating this joint origin falls slightly below the previous joint. Finally,
as this joint is a revolute, theta will be variable and is defined as q4.

Joint 5
Since Joint 5 is a revolute and is rotated by counter-clockwise 90 degrees relative to Joint 4, alpha is thus positive 
90 degrees (pi/2) by the right-hand rule. Since I chose the origin of Joint 4 to be the same as for Joint 5, this
means that both a and d will be zero. Joint 5 is again a revolute, so theta is defined as a variable, q5.

Joint 6
As before, Joint 6 is a revolute and is rotated clockwise 90 degrees relative to Joint 5, thus alpha will be negative
90 degrees (-pi/2). As with Joint 5, this joint's origin is the same point as Joint 5 so by definition, a, and d will
again be zero. As a revolute, the joint's rotation will be variable and represented by q6.

"Joint" 7 (the End-Effector's "sweet spot")
For the sake of easier calculations, it's relevant to define one more origin that represents the origin of where we
would want an object to be in order to grab it. For this table, I define that as Joint 7. Joint 7 should be viewed
as a fixed joint (no variables). Due to it being a fixed joint, there is no twist angle relative to Joint 6 and no
translation, hence both alpha and a are zero. I define d as .303 to reference the location where an object to be
gripped should be located. As such, you'll find .3 in the table. Last, since this is a fixed joint, theta is fixed
at zero.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
The general form of the transformation matrix will look like this:

```
Matrix([
[            cos(q),            -sin(q),            0,              a],
[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
[sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
[                 0,                  0,            0,              1]
])
```

###### T0->1
```
⎡cos(q₁)  -sin(q₁)  0   0  ⎤
⎢sin(q₁)  cos(q₁)   0   0  ⎥
⎢   0        0      1  0.75⎥
⎣   0        0      0   1  ⎦
```
###### T1->2
```
⎡cos(q₂ - 0.5⋅π)   -sin(q₂ - 0.5⋅π)  0  0.35⎤
⎢       0                 0          1   0  ⎥
⎢-sin(q₂ - 0.5⋅π)  -cos(q₂ - 0.5⋅π)  0   0  ⎥
⎣       0                 0          0   1  ⎦
```
###### T2->3
```
⎡cos(q₃)  -sin(q₃)  0  1.25⎤
⎢sin(q₃)  cos(q₃)   0   0  ⎥
⎢   0        0      1   0  ⎥
⎣   0        0      0   1  ⎦
```
###### T3->4
```
⎡cos(q₄)   -sin(q₄)  0  -0.054⎤
⎢   0         0      1   1.5  ⎥
⎢-sin(q₄)  -cos(q₄)  0    0   ⎥
⎣   0         0      0    1   ⎦
```
###### T4->5
```
⎡cos(q₅)  -sin(q₅)  0   0⎤
⎢   0        0      -1  0⎥
⎢sin(q₅)  cos(q₅)   0   0⎥
⎣   0        0      0   1⎦
```
###### T5->6
```
⎡cos(q₆)   -sin(q₆)  0  0⎤
⎢   0         0      1  0⎥
⎢-sin(q₆)  -cos(q₆)  0  0⎥
⎣   0         0      0  1⎦
```
###### T6->G or EE (no rotation)
```
⎡1  0  0    0  ⎤
⎢0  1  0    0  ⎥
⎢0  0  1  0.303⎥
⎣0  0  0    1  ⎦
```

##### Homogeneous Transformation matrix from base_link to Gripper


```
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
T_total = T0_G * R_corr
```
Note: R_corr is a 180 degree rotation around the Z axis followed by a -90 degree rotation around the Y axis

 
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
Inverse Kinematics goal is the opposite of Forward Kinematics. In this case, the position and orientation of the end-effector is known and the goal is to calculate
the joint angles of the arm of the robot. In this case, a closed-form solution is used for performance. In order to use the closed-form solution there are two key requirements
of the robot:
1.  3 neighboring joint axes must intersect at a single point (forming a spherical wrist)
2.  3 neighboring joint axes are parallel

The Kuka Arm satisfies the above conditions and thus closed-form is possible.

The spherical wrist involves joints 4,5 and 6. The position of the wrist center is a by-product of the positions of
the first three joints.

We can derive the position of the wrist center by using the complete transformation matrix. That matrix is:

```
⎡lx  mx  nx  px⎤
⎢ly  my  ny  py⎥
⎢lz  mz  nz  pz⎥
⎣0   0   0   1 ⎦
```
where l,m and n are orthonormal vectors representing the end-effector orientation along X,Y,Z axes of local coordinates.

```
wcx = px - (d6 + l) * nx
wcy = py - (d6 + l) * ny
wcz = pz - (d6 + l) * nz
```
where l is the end effort length and d6 = 0 (link 6 length).

To calculate nx, ny and nz, rotation matrices are created with error correction.

```
ROT_x = Matrix([[1, 0,0],
                [0, cos(r), -sin(r)],
                [0, sin(r), cos(r)]
                ])  # ROLL
ROT_y = Matrix([[cos(p), 0, sin(p)],
                [0, 1, 0],
                [-sin(p), 0, cos(p)]
                ])  # PITCH
ROT_z = Matrix([[cos(y), -sin(y), 0],
                [sin(y), cos(y), 0],
                [0, 0, 1]
                ])  # YAW

Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
ROT_EE = ROT_z * ROT_y * ROT_x * Rot_Error
Rrpy = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

nx = Rrpy[0,2]
ny = Rrpy[1,2]
nz = Rrpy[2,2]
```
Roll, pitch and yaw are extracted from the end effector pose and thus Rrpy = Homogeneous RPY rotation between base and gripper.

`theta1 = atan2(wcy, wcx)` and is clipped to +- 185 degrees per the URDF file.

![alt text][image11]

where

```
side_a = 1.501 #(d4)
side_b = sqrt(pow((sqrt(wcx * wcx + wcy * wcy) - 0.35), 2) + pow((wcz - 0.75), 2))
side_c = 1.25 #(a2)
```

```
Theta2 = pi/2 - angle_a - atan2(wcz - 0.75, sqrt(wcx * wcx + wcy * wcy) - 0.35)
Theta3 = pi/2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054 m`
```

Per the URDF file, Theta 2 is clipped to -45 and +85 degrees and Theta 3 is clipped to -210 and +155-90 degrees.

##### Inverse Orientation

Given that

```
R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
and
R0_6 = Rrpy
```
we can precalculate rotations R0_3 (with theta 1 to 3 substituted) by extracting the rotation matrix from the transformation matrices

```
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
```

leading to when we divide both sides by `R0_3` to

```
R3_6 = inv(R0_3) * Rrpy
```

Theta 4,5,6 are derived from R3_6 using Euler Angles from Rotation Matrix.

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,2], R3_6[1,0])
```
Theta 4 & 6 is clipped +-350 degrees per the URDF file as is Theta 5 to +-125 degrees.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code follows the process described above. Please see the code for detailed discussion.


