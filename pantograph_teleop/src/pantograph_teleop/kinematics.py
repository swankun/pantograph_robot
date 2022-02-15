import numpy as np
import math
from math import atan2, sin, cos, sqrt, acos

def remainder(x, y):
    absx = abs(x)
    absy = abs(y)
    m = math.fmod(absx, absy)
    c = absy - m
    if (m < c):
        r = m
    elif (m > c):
        r = -c
    else:
        assert(m == c)
        r = m - 2.0 * math.fmod(0.5 * (absx - m), absy)
    return math.copysign(1.0, x) * r

class PantographKinematics(object):

    def __init__(self, 
                 link_lengths=np.array([0.530, 0.70, 0.70, 0.528]),
                 table_offset_x=np.array([0.765, 0.825]),
                 table_offset_y=0.14):
        self.link_lengths = link_lengths
        self.table_offset_x = table_offset_x
        self.table_offset_y = table_offset_y

    def inverse(self, x, y, xdot=None, ydot=None):
        # The q2, q3 this function finds is relative

        # Link constants
        a = self.link_lengths

        # Table constants
        u = self.table_offset_x
        v = self.table_offset_y

        eta = np.zeros((2, 1))
        eta[0] = x + u[0]
        eta[1] = y + v

        D = 1/2.0/a[0]/a[1]*( eta[0]**2 + eta[1]**2 - a[0]**2 - a[1]**2 )
        q = np.zeros((4, 1))
        q[1] = atan2(sqrt(1-D**2), D)
        q[0] = atan2(eta[1], eta[0]) - atan2(a[1]*sin(q[1]), a[0] + a[1]*cos(q[1]))

        eta[0] = x - u[1]
        D = 1/2.0/a[3]/a[2]*( eta[0]**2 + eta[1]**2 - a[3]**2 - a[2]**2 )
        q[2] = atan2(-sqrt(1-D**2), D)
        q[3] = atan2(eta[1], eta[0]) - atan2(a[2]*sin(q[2]), a[3] + a[2]*cos(q[2]))

        if xdot is None or ydot is None:
            qdot = None
        else:
            jac_left = np.array([
                [ -a[0]*sin(q[0]) - a[1]*sin(q[0]+q[1]), -a[1]*sin(q[0]+q[1]) ],
                [ a[0]*cos(q[0]) + a[1]*cos(q[0]+q[1]), a[1]*cos(q[0]+q[1]) ]
            ])
            jac_right = np.array([
                [ -a[2]*sin(q[2]+q[3]), -a[3]*sin(q[3]) - a[2]*sin(q[2]+q[3]) ],
                [ a[2]*cos(q[2]+q[3]), a[3]*cos(q[3]) + a[2]*cos(q[2]+q[3]) ]
            ])
            qdot_left = np.linalg.solve( jac_left, np.array([xdot,ydot]) )
            qdot_right = np.linalg.solve( jac_right, np.array([xdot,ydot]) )
            qdot = np.concatenate((qdot_left, qdot_right), axis=None)
        return q, qdot


    def forward(self, theta1, theta4, theta1_dot=None, theta4_dot=None):
        # The q2 this function finds is absolute.

        # These will be function arguments
        theta1 = -theta1*math.pi/180.0
        theta4 = -theta4*math.pi/180.0
        
        # Link constants
        a = self.link_lengths

        # Table constants
        u = self.table_offset_x
        v = self.table_offset_y

        r0 = np.array([np.sum(u), 0.0])
        r1 = -1.0*np.array([a[0]*cos(theta1), a[0]*sin(theta1)])
        r4 = np.array([a[3]*cos(theta4), a[3]*sin(theta4)])
        r = r0 + r1 + r4

        angle_r = atan2(r[1], r[0])
        norm_r = np.linalg.norm(r)

        D = abs( 1.0/2.0/norm_r/a[1]*( a[2]**2 - a[1]**2 - norm_r**2 ) )

        angle_btw = atan2(sqrt(1-D**2), D)
        # angle_btw = acos(D)

        theta2 = remainder(angle_btw + angle_r, 2.0*math.pi)
        # print(theta2)

        x = - u[0] + a[0]*cos(theta1) + a[1]*cos(theta2)
        y = - v + a[0]*sin(theta1) + a[1]*sin(theta2)

        if theta1_dot is None or theta4_dot is None:
            xdot, ydot = None, None
        else:
            theta1_dot = -theta1_dot*math.pi/180.0
            theta4_dot = -theta4_dot*math.pi/180.0
            theta3 = atan2(
                a[0]*sin(theta1) + a[1]*sin(theta2) - a[3]*sin(theta4),
                a[0]*cos(theta1) + a[1]*cos(theta2) - a[3]*cos(theta4) - r0[0]
            )
            q = np.array([
                theta1,
                theta2 - theta1,
                theta3 - theta4,
                theta4
            ])
            
            jac_left = np.array([
                [ -a[0]*sin(q[0]) - a[1]*sin(q[0]+q[1]), -a[1]*sin(q[0]+q[1]) ],
                [  a[0]*cos(q[0]) + a[1]*cos(q[0]+q[1]),  a[1]*cos(q[0]+q[1]) ]
            ])
            jac_right = np.array([
                [ -a[2]*sin(q[2]+q[3]), -a[3]*sin(q[3]) - a[2]*sin(q[2]+q[3]) ],
                [  a[2]*cos(q[2]+q[3]),  a[3]*cos(q[3]) + a[2]*cos(q[2]+q[3]) ]
            ])
            A = np.column_stack((  jac_left[:,1], -jac_right[:,0] ))
            B = np.column_stack(( -jac_left[:,0],  jac_right[:,1] ))
            rhs = np.dot( B, np.array((theta1_dot, theta4_dot)) )
            theta2_dot, theta3_dot = np.linalg.solve(A, rhs)

            xdot, ydot = np.dot( jac_left,  np.array([theta1_dot, theta2_dot]) )
            # xdot, ydot = np.dot( jac_right, np.array([theta3_dot, theta4_dot]) )

        return x, y, xdot, ydot
