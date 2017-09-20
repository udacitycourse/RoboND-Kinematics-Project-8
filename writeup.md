## Project: Kinematics Pick & Place

---


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/forward_eq.png
[image5]: ./misc_images/forward_tf.png
[image6]: ./misc_images/wc.png
[image7]: ./misc_images/fw_kin_results.png
[image8]: ./misc_images/theta1.png
[image9]: ./misc_images/theta23.png


## Kinematic Analysis
### Forward kinematics

The kuka 210 robot model in zero configuration
# ![alt text][image1]


* Red dot is the origin of the frame, define in the intersection point of X_i and Z_i
* Z_i is the rotation axis of a joint.
* X defines the common axes define the common normals between Z_i-1 and Z_i.       

Below is the DH parameter table derived from the robot model:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

where,
* i is the link number
* alpha_i-1 is the twist angle z_i-1 and z_i measured about x_i-1 in a right hand sense.
* d_i is the sign distance between x_i-1 and x_i measured along z_i axis
* a_i-1 is the link length between z_i-1 and z_i measured along x_i-1
* theta is the joint angle between x_i-1 and x_i measured about z_i in a right hand sense. Variable in the case of revolute joint.

```python
# Symbols for joint variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

### KUKA KR210 ###
# DH Parameters
DH_table = {alpha0:     0,  a0:      0, d1:  0.75,  q1: q1,
            alpha1: -pi/2,  a1:   0.35, d2:     0,  q2: q2-pi/2.,
            alpha2:     0,  a2:   1.25, d3:     0,  q3: q3,
            alpha3: -pi/2,  a3: -0.054, d4:  1.50,  q4: q4,
            alpha4:  pi/2,  a4:      0, d5:     0,  q5: q5,
            alpha5: -pi/2,  a5:      0, d6:     0,  q6: q7,
            alpha6:     0,  a6:      0, d7: 0.303,  q7: 0}
```
Equation for transformation from link_i-1 frame to link_i frame is:
![alt text][image4]

and in a matrix form:
![alt text][image5]
```python
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[             cos(q),              -sin(q),            0,               a],
                 [sin(q) * cos(alpha),  cos(q) * cos(alpha),  -sin(alpha), -sin(alpha) * d],
                 [sin(q) * sin(alpha),  cos(q) * sin(alpha),   cos(alpha),  cos(alpha) * d],
                 [                  0,                    0,            0,               1]])
    return TF
```     

```python
### Homogeneous Transforms between i and i-1 links
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_table)
T6_G = TF_Matrix(alpha6, a6, d7, q7).subs(DH_table)
T0_2 = T0_1 * T1_2  # tf base_link to link2
T0_3 = T0_2 * T2_3  # tf base_link to link3
T0_4 = T0_3 * T3_4  # tf base_link to link4
T0_5 = T0_4 * T4_5  # tf base_link to link5
T0_6 = T0_5 * T5_6  # tf base_link to link6
T0_G = T0_6 * T6_G  # tf base_link to link7
```

We have to fix the inconsistency between DH parameters and the parameters defined in the URDF file. 
```python
R_z = Matrix([[    cos(pi), -sin(pi),   0,  0],
              [    sin(pi),  cos(pi),   0,  0],
              [          0,        0,   1,  0],
              [          0,        0,   0,  1]])  # YAW

R_y = Matrix([[ cos(-pi/2),     0,   sin(-pi/2),  0],
              [          0,     1,            0,  0],
              [-sin(-pi/2),     0,   cos(-pi/2),  0],
              [          0,     0,            0,  1]])  # PITCH

R_corr = R_z * R_y
T_total = T0_G * R_corr
```
To calculate wrist center (WC), we use the equation:

![alt text][image6]
```python
r, p, y = symbols('r p y')
R_ee_x = Matrix([[      1,      0,       0, 0],
                 [      0, cos(r), -sin(r), 0],
                 [      0, sin(r),  cos(r), 0],
                 [      0,      0,       0, 1]])  # ROLL

R_ee_y = Matrix([[ cos(p),      0,  sin(p), 0],
                 [      0,      1,       0, 0],
                 [-sin(p),      0,  cos(p), 0],
                 [      0,      0,       0, 1]])  # PITCH

R_ee_z = Matrix([[ cos(y), -sin(y),   0, 0],
                 [ sin(y),  cos(y),   0, 0],
                 [      0,       0,   1, 0],
                 [      0,       0,   0, 1]])  # YAW

# First the gripper and WC frames are aligned
# and then translated along common z-axis
p = Matrix([[px], [py], [pz]])
d6 = Matrix([[0], [0], [0.193 + 0.11]])
R_ee = R_ee_z * R_ee_y * R_ee_x
R_ee = R_ee.subs({'r': roll, 'p': pitch, 'y': yaw})
WC = p - R_ee[:3, :3]*R_corr[:3, :3]*d6
```
where d6 is the distance from link_5 to gripper_link (check joint info in URDF), R_ee is the end-effector pose,
R_corr is the correction matrix and p is the end_effector position w.r.t. base_link.

Transformation from base_link to gripper_link:
# ![alt text][image7]


### Inverser Kinematics
#### Theta 1
Theta 1 is simply to calculate using the WC point:
```python
theta1 = atan2(WC[1],WC[0])
```
![alt text][image8]
#### Theta 2 and Theta 3
The A, B and C values can be found using the DH table, URDF file and WC position.
A is distance from link_3 to link_5 (for somereason in the reference implementation this value is 1.501 instead of 1.500),
C is the distance from link_2 to link_3 and B is calculated using Pythagorean equation.

Angles a, b, are all derived using Coisine Law theorem.

Finally the theta2 and theta3 can be calculated as:
```python
B = np.sqrt(float((np.sqrt(float(WC[0]**2+WC[1]**2))-DH_table[a1])**2 + (WC[2]-DH_table[d1])**2))
A = 1.501  
C = DH_table[a2]
a = acos((B * B + C * C - A * A) / (2 * B * C))
b = acos((A * A + C * C - B * B) / (2 * A * C))
c = acos((A * A + B * B - C * C) / (2 * A * B))
theta2 = pi / 2 - a  - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1]) - 0.35)
theta3 = pi / 2 - (b+0.036)
```
![alt text][image9]


#### Theta 4, 5 and 6



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


