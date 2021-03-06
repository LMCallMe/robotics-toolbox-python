"""
Jacobian matrix operations.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement of
the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""

from kinematics import *
from sympy import *
from sympyrobot import *

def jacob0(robot, q):
    """
    Compute manipulator Jacobian in world coordinates for joint coordinates C{q}.

    The manipulator Jacobian matrix maps differential changes in joint space
    to differential Cartesian motion (world coord frame) of the end-effector

    M{dX = J dQ}

    @type robot: Robot
    @type q: vector M{n x 1}
    @param q: joint coordinate
    @rtype: matrix M{6 x n}
    @return: Manipulator Jacobian
    @see: L{jacobn}, L{diff2tr}, L{tr2diff}
    """

    q = Matrix(q)
    Jn = jacobn(robot, q) # Jacobian from joint to wrist space
    #   convert to Jacobian in base coordinates
    Tn = fkine(robot,q) # end-effector transformation
    R = t2r(Tn)
    T = Matrix.eye(6)
    T[0:3,0:3] = R
    T[3:6,3:6] = R
    return T*Jn


def jacobn(robot, q):
    """
    Compute manipulator Jacobian in tool coordinates for joint coordinates C{q}.

    The manipulator Jacobian matrix maps differential changes in joint space
    to differential Cartesian motion (tool coord frame) of the end-effector.

    M{dX = J dQ}
		
    Reference
    =========
 	
 	Paul, Shimano, Mayer
    Differential Kinematic Control Equations for Simple Manipulators
    IEEE SMC 11(6) 1981
    pp. 456-460

    @type robot: Robot
    @type q: vector M{n x 1}
    @param q: joint coordinate
    @rtype: matrix M{6 x n}
    @return: Manipulator Jacobian
    @see: L{jacobn}, L{diff2tr}, L{tr2diff}
    """
    if not isinstance(q,Matrix):
        q = Matrix(q).vec()
    L = robot.links
    n = len(L)
    J = Matrix.zeros(6,n)
    U = robot.tool
    for j in range(n-1,-1,-1):
            if not robot.ismdh():    #standard DH convention
                    U = L[j].tr(q[j])*U
            if L[j].sigma == 0: #revolute axis
                    d = Matrix([[-U[0,0]*U[1,3] + U[1,0]*U[0,3]],\
                                [-U[0,1]*U[1,3] + U[1,1]*U[0,3]],\
                                [-U[0,2]*U[1,3] + U[1,2]*U[0,3]]])
                    delta = U[2,0:3].transpose()   # nz  oz  az
            else: #prismatic axis
                    d = U[2,0:3].transpose()        # nz  oz  az
                    delta = Matrix.zeros(3,1) # 0   0   0
            J[:3,j] = d
            J[3:6,j] = delta
            if robot.ismdh(): #modified DH convention
                    U=L[j].tr(q[j])*U
    return J



def tr2jac(t):
    """
    Compute a Jacobian to map differentials motion between frames.
    The M{6x6} Jacobian matrix to map differentials (joint velocity) between 
    frames related by the homogeneous transform C{t}.

    @rtype: matrix M{6 x 6}
    @return: Jacobian matrix
    @see: L{tr2diff}, L{diff2tr}
    """
    t = mat(t)
    return concatenate((
        concatenate((t[0:3,0].T, crossp(t[0:3,3],t[0:3,0]).T),1),
        concatenate((t[0:3,1].T, crossp(t[0:3,3],t[0:3,1]).T),1),
        concatenate((t[0:3,2].T, crossp(t[0:3,3],t[0:3,2]).T),1),
        concatenate((zeros((3,3)),t[0:3,0:3].T),1) ))
