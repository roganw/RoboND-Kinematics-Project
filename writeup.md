## Project: Kinematics Pick & Place


[//]: # (Image References)

[image1]: ./result/viz.png
[image2]: ./result/dh.png
[image3]: ./result/fk_dh.png
[image4]: ./result/position_frame.png
[image5]: ./result/position_frame2.png
[image6]: ./result/orientation_frame.png
[image7]: ./result/wc.png
[image8]: ./result/euler.png
[image9]: ./result/debug.png
[image10]: ./result/pick_result.png
[image11]: ./result/homogeneous_transformation.png
[image12]: ./result/composed_transform.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The postions of links/joints can be obtained from the RViz view.  
![alt text][image1]  
DH convention described in John J Craig's book.
![alt text][image2]  
The definition of DH parameters for KR210:
![alt text][image3]  
Referred from previous materials, we can get the DH table like this:
```python
dh_table = {
    alpha0: 0,      a0: 0,      d1: 0.75,   q1: q1,
    alpha1: -pi/2., a1: 0.35,   d2: 0,      q2: q2-pi/2,
    alpha2: 0,      a2: 1.25,   d3: 0,      q3: q3,
    alpha3: -pi/2., a3: -0.054, d4: 1.50,   q4: q4,
    alpha4: pi/2.,  a4: 0,      d5: 0,      q5: q5,
    alpha5: -pi/2., a5: 0,      d6: 0,      q6: q6,
    alpha6: 0,      a6: 0,      d7: 0.303,  q7: 0,
}
```

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
Homogeneous transformation between neighboring links is:
![alt text][image11]  

For each link is actually composed of four individual transforms, 2 rotations and 2 translations, performed in the order shown as:
![alt text][image12]  

To abstract the same function, I defined a common method named `generate_homegeneous_transform`. Then I could create individual transformation matrices by one statement.
```python
def generate_homegeneous_transform(alpha, a, d, q):
    # Define Modified DH Transformation matrix
    h_t = Matrix([
        [cos(q),            -sin(q),            0,              a],
        [sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)*d],
        [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),     cos(alpha)*d],
        [0,                 0,                  0,              1]
    ])
    return h_t
```

Extract rotation matrices from the transformation matrices  
```python
T0_1 = generate_homegeneous_transform(alpha0, a0, d1, q1).subs(dh_table)
T1_2 = generate_homegeneous_transform(alpha1, a1, d2, q2).subs(dh_table)
T2_3 = generate_homegeneous_transform(alpha2, a2, d3, q3).subs(dh_table)
T3_4 = generate_homegeneous_transform(alpha3, a3, d4, q4).subs(dh_table)
T4_5 = generate_homegeneous_transform(alpha4, a4, d5, q5).subs(dh_table)
T5_6 = generate_homegeneous_transform(alpha5, a5, d6, q6).subs(dh_table)
T6_G = generate_homegeneous_transform(alpha6, a6, d7, q7).subs(dh_table)
```

Homogeneous transform matrix from base_link to gripper_link:
```python
T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```

Generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
```python
# EE Matrices
# EE rotation matrix
r, p, y = symbols('r p y')
R_x = rot_x(r)
R_y = rot_y(p)
R_z = rot_z(y)

R_EE = R_z * R_y * R_x
R_Error = R_EE.subs({'r': 0, 'p': -pi/2, 'y': pi})
# R_Error = R_z.subs(y, pi) * R_y.subs(p, -pi/2)

R_EE = R_EE * R_Error
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Compensate for rotation discrepancy between DH parameters and Gazebo
```python
R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
```

![alt text][image6]
![alt text][image7]
Calculate WC(wrist center) position matrix to base frame
```python
# Position Matrix of end-effector to base framea
EE_0 = Matrix([[px], [py], [pz]])
# Calculate WC(wrist center) position matrix to base frame, reference from 'Inverse Kinematics' section, d7 = 0.303
WC_0 = EE_0 - dh_table[d7] * R_EE[:, 2]
```
![alt text][image4]
![alt text][image5]
Calculate joint angles using Geometric IK method
```python
# Calculate joint angles using Geometric IK method, reference from 'Inverse Kinematics Example' section
theta1 = atan2(WC_0[1], WC_0[0])

# Using trigonometry, specifically the Cosine Laws, you can calculate theta 2 and theta 3.
side_a = dh_table[d4]
# Reference from 'Inverse Kinematics Example'
temp_r = sqrt(WC_0[0]**2 + WC_0[1]**2) - dh_table[a1]
temp_s = WC_0[2] - dh_table[d1]
# Inverse Kinematics with Kuka KR210
side_b = sqrt(temp_r**2 + temp_s**2)
side_c = dh_table[a2]
# interior angle of triangle
angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2 * side_a * side_c))

# calculate theta2
theta2 = pi/2 - angle_a - atan2(temp_s, temp_r)
theta2 = theta2.evalf()

# theta3 = pi/2 - angle_b - atan2(abs(dh_table[a3]), dh_table[d4])
theta3 = pi/2 - angle_b - 0.036
theta3 = theta3.evalf()
```

Calculate Euler Angles from the Rotation Matrix
![alt text][image8]
```python
# Euler Angles from a Rotation Matrix
R0_3 = T0_1[:3, :3] * T1_2[:3, :3] * T2_3[:3, :3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = R0_3.inv("LU") * R_EE

theta4 = atan2(R3_6[2,2], -R3_6[0,2]).evalf()
theta5 = atan2(sqrt(R3_6[1,0]**2 + R3_6[1,1]**2), R3_6[1,2]).evalf()
theta6 = atan2(-R3_6[1,1], R3_6[1,0]).evalf()
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
In addition to the method `generate_homegeneous_transform` introduced before, three rotation matrices in 3D was defined in front of `IK_server.py`:
```python
def rot_x(q):
    R_x = Matrix([
        [1,     0,          0],
        [0,     cos(q),     -sin(q)],
        [0,     sin(q),     cos(q)]
    ])
    return R_x
    
def rot_y(q):              
    R_y = Matrix([
        [cos(q),    0,      sin(q)],
        [0,         1,      0],
        [-sin(q),   0,      cos(q)]
    ])
    return R_y

def rot_z(q):    
    R_z = Matrix([
        [cos(q),    -sin(q),    0],
        [sin(q),    cos(q),     0],
        [0,         0,          1]
    ])
    return R_z
```


To evaluate the dh parameters and calculation method, I test them in `IK_debug.py`. And finally got these result:
![alt text][image9]


After the debug process got a decent result, I modified `inverse_kinematics.launch` file to set the demo mode to `false`, and added `x-terminal-emulator -e rosrun kuka_arm IK_server.py` to  `./safe_spawner.sh`, then ran it. 
![alt text][image10]


#### 2. result video
The directory of test video is `/result/result_video.mp4`.  


