#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import mpl_toolkits.mplot3d.axes3d as p3
from numpy import * # for outer and arange
import pylab as p   # for figure
from robot.Quaternion import *

def trplot(T, name=''):
    '''
    Plot a rotation as a set of axes aligned with the x, y and z
    directions of a frame rotated by C{r}.
    '''

    if type(T) is matrix:
        if T.shape == (3,3):
            q = quaternion(T)
            c = array([0,0,0])
        elif T.shape == (4,4) and ishomog(T):
            q = quaternion(T)
            c = array(T[:3,3]).flatten()
    elif isinstance(T, quaternion):
        q = T
        c = array([0,0,0])
    else:
        raise ValueError

    x = q * mat([1,0,0])
    y = q * mat([0,1,0])
    z = q * mat([0,0,1])
    fig=p.figure()
    ax=p3.Axes3D(fig)
    delta = 2
    ax.set_xlim([c[0]-delta,c[0]+delta])
    ax.set_ylim([c[1]-delta,c[1]+delta])
    ax.set_zlim([c[2]-delta,c[2]+delta])
    ax.plot3D([c[0],c[0] + x[0,0]], [c[1],c[1] + x[0,1]], [c[2],c[2]+x[0,2]], color='red')
    ax.plot3D([c[0],c[0] + y[0,0]], [c[1],c[1] + y[0,1]], [c[2],c[2]+y[0,2]], color='green')
    ax.plot3D([c[0],c[0] + z[0,0]], [c[1],c[1] + z[0,1]], [c[2],c[2]+z[0,2]], color='blue')
    p.show()

if __name__ == "__main__":
    from robot.transform import *
    trplot( rotx(0.2) )
