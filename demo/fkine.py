# Copyright (C) 1993-2002, by Peter I. Corke

# $Log: rtfkdemo.m,v $
# Revision 1.3  2002/04/02 12:26:48  pic
# Handle figures better, control echo at end of each script.
# Fix bug in calling ctraj.
#
# Revision 1.2  2002/04/01 11:47:17  pic
# General cleanup of code: help comments, see also, copyright, remnant dh/dyn
# references, clarification of functions.
#

import robot.parsedemo as p
import sys

print sys.modules

if __name__ == '__main__':

    s = '''
# First we will define a robot
    from robot.puma560 import *

# Forward kinematics is the problem of solving the Cartesian position and 
# orientation of a mechanism given knowledge of the kinematic structure and 
# the joint coordinates.
#
# Consider the Puma 560 example again, and the joint coordinates of zero,
# which are defined by qz
    qz
#
# The forward kinematics may be computed using fkine() with an appropropriate 
# kinematic description, in this case, the matrix p560 which defines 
# kinematics for the 6-axis Puma 560.
    fkine(p560, qz)
#
# returns the homogeneous transform corresponding to the last link of the 
# manipulator
pause % any key to continue
#
# fkine() can also be used with a time sequence of joint coordinates, or 
# trajectory, which is generated by jtraj()
#
    t = arange(0, 2, 0.056) 	% generate a time vector
    (q, qd, qdd) = jtraj(qz, qr, t); % compute the joint coordinate trajectory
#
# then the homogeneous transform for each set of joint coordinates is given by
    T = fkine(p560, q)

#
# where T is a list of matrices.
#
# For example, the first point is
    T[0]
#
# and the tenth point is
    T[9]
pause % any key to continue
#
# Elements (0:2,3) correspond to the X, Y and Z coordinates respectively, and 
# may be plotted against time
    x = array([e[0,3] for e in T])
    y = array([e[1,3] for e in T])
    z = array([e[2,3] for e in T])
    subplot(3,1,1)
    plot(t, x)
    xlabel('Time (s)')
    ylabel('X (m)')
    subplot(3,1,2)
    plot(t, y)
    xlabel('Time (s)')
    ylabel('Y (m)')
    subplot(3,1,3)
    plot(t, z)
    xlabel('Time (s)')
    ylabel('Z (m)')
    show()
pause % any key to continue
#
# or we could plot X against Z to get some idea of the Cartesian path followed
# by the manipulator.
#
    clf()
    plot(x, z)
    xlabel('X (m)')
    ylabel('Z (m)')
    grid()
pause % any key to continue
echo off
'''

    p.parsedemo(s)
