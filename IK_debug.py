from sympy import *
from time import time
from mpmath import radians
import tf
import numpy as np

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[2.1529, 0.0, 1.9465],
                  [0.0, -0.00014835, 0.0, 1.0]],
                  [1.8499, 0.000, 1.9464],
                  [-4.0e-06, -4.0e-06, 0.0, -0.00012, -0.00017, 0.0, 0.0, 0.0]],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    # Extract end-effector a.k.a gripper_link position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
         req.poses[x].orientation.z, req.poses[x].orientation.w])


    theta1 = test_case[2][0]
    theta2 = test_case[2][1]
    theta3 = test_case[2][2]
    theta4 = test_case[2][3]
    theta5 = test_case[2][4]
    theta6 = test_case[2][5]


    ### Create symbols for joint variables
    # alpha: twist angle
    # a: link length
    # d: link offsets
    # q: theta variables
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

    def TF_Matrix(alpha, a, d, q):
        TF = Matrix([[             cos(q),              -sin(q),            0,               a],
                     [sin(q) * cos(alpha),  cos(q) * cos(alpha),  -sin(alpha), -sin(alpha) * d],
                     [sin(q) * sin(alpha),  cos(q) * sin(alpha),   cos(alpha),  cos(alpha) * d],
                     [                  0,                    0,            0,               1]])
        return TF



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



    R_z = Matrix([[    cos(pi),     -sin(pi),   0,  0],
                  [    sin(pi),      cos(pi),   0,  0],
                  [          0,            0,   1,  0],
                  [          0,            0,   0,  1]])  # YAW

    R_y = Matrix([[ cos(-pi/2),     0,  sin(-pi/2),   0],
                  [          0,     1,            0,  0],
                  [-sin(-pi/2),     0,   cos(-pi/2),  0],
                  [          0,     0,            0,  1]])  # PITCH

    R_corr = R_z * R_y
    T_total = T0_G * R_corr
    # print(Matrix(T_total.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0, q7: 0})))
    ee = Matrix(T_total.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6, q7: 0}))
    print(ee[0, 3], " ", ee[1, 3], " ", ee[2, 3])

    # calculate o4o5o6 (wrist center location)
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

    # We will translate along z-xis
    # First the gripper and WC frames are aligned
    # and then translated
    p = Matrix([[px], [py], [pz]])
    d6 = Matrix([[0], [0], [0.193 + 0.11]])
    R_ee = R_ee_z * R_ee_y * R_ee_x
    R_ee = R_ee.subs({'r': roll, 'p': pitch, 'y': yaw})
    WC = p - R_ee[:3, :3]*R_corr[:3,:3]*d6  # we will use only the rotation part of the matrices

    WC = WC.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6, q7: 0})
    print(WC[0], " ", WC[1], " ", WC[2])


    ### calculate theta1, theta2 and theta3
    theta1 = atan2(WC[1], WC[0])
    theta1 = theta1.evalf()
    print("Theta1: ", theta1)
    # sides
    B = np.sqrt(float((np.sqrt(float(WC[0]**2+WC[1]**2))-DH_table[a1])**2 + (WC[2]-DH_table[d1])**2))
    A = DH_table[d4]  # link 3 to link 5 TODO: in the example code the value is 1.501 ??
    C = DH_table[a2]

    # angles
    a = acos((B * B + C * C - A * A) / (2 * B * C))
    b = acos((A * A + C * C - B * B) / (2 * A * C))
    c = acos((A * A + B * B - C * C) / (2 * A * B))

    # theta2 = pi / 2 - a - acos((np.sqrt(float(WC[0]**2+WC[1]**2))-DH_table[a1]) / B) # TODO: fix to atan2
    theta2 = pi / 2 - a  -atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1]) - 0.35)
    theta3 = pi / 2 - (b+0.036)  # TODO: CHECK
    theta2 = theta2.evalf()
    theta3 = theta3.evalf()
    print("Theta2: ", theta2)
    print("Theta3: ", theta3)


    ### calculate theta1, theta2 and theta3
    # check the lesson Euler Angles from a Rotation Matrix
    T0_3 = T0_1 * T1_2 * T2_3
    T0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    T3_6 = T0_3.inv("LU") * R_ee * R_corr
    T3_6 = T3_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    theta4 = atan2(T3_6[2, 2], -T3_6[0, 2]) # z component
    theta4 = theta4.evalf()
    print("Theta4: ", theta4)

    #theta5 = atan2(-T3_6[2, 0], np.sqrt(float(T3_6[0, 0]*T3_6[0, 0] +T3_6[0, 1]*T3_6[0, 1]))) # y component
    theta5 = atan2(sqrt(T3_6[0,2]*T3_6[0,2]+T3_6[2,2]*T3_6[2,2]),T3_6[1,2])
    theta5 = theta5.evalf()
    print("Theta5: ", theta5)

    #theta6 = atan2(T3_6[2, 1],T3_6[2, 2])# x component
    theta6 = atan2(-T3_6[1,1], T3_6[1,0])
    theta6 = theta6.evalf()
    print("Theta6: ", theta6)


    ##
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0], WC[1], WC[2]]  # <--- Load your calculated WC values in this array
    your_ee = [ee[0, 3], ee[1, 3], ee[2, 3]]  # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 4

    test_code(test_cases[test_case_number])
