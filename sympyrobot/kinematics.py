"""
Robot kinematic operations.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement of
the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
"""
from sympy import *
from utility import ishomog,numcols,error,numrows
from transform import tr2diff
from numpy.linalg import norm

def fkine(robot, q):
    """
    Computes the forward kinematics for each joint space point defined by C{q}.
    ROBOT is a robot object.

    For an n-axis manipulator C{q} is an n element vector or an m x n matrix of
    robot joint coordinates.

    If C{q} is a vector it is interpretted as the generalized joint coordinates, and
    C{fkine} returns a 4x4 homogeneous transformation for the tool of
    the manipulator.

    If C{q} is a matrix, the rows are interpretted as the generalized 
    joint coordinates for a sequence of points along a trajectory.  q[i,j] is
    the j'th joint parameter for the i'th trajectory point.  In this case
    C{fkine} returns a list of matrices for each point
    along the path.

    The robot's base or tool transform, if present, are incorporated into the
    result.
    
    @type robot: Robot instance
    @param robot: The robot
    @type q: vector
    @param q: joint coordinate
    @see: L{Link}, L{Robot}, L{ikine}
    """

    q = Matrix(q)
    n = robot.n
    if (numrows(q)==1 and numcols(q)==n) or (numrows(q)==n and numcols(q)==1):
        q = q.vec()
        t = robot.base
        for i in range(0,n):
            t = t * robot.links[i].tr(q[i])
        t = t * robot.tool
        return t
    else:
        if numcols(q) != n:
            raise 'bad data'
        t = []
        for qv in q:        # for each trajectory point
            tt = robot.base
            for i in range(0,n):
                tt = tt * robot.links[i].tr(qv[0,i])
            t.append(tt*robot.tool)
        return t




def ikine(robot, tr, q0=None, m=None, **args):
    """
    Inverse manipulator kinematics.
    Computes the joint coordinates corresponding to the end-effector transform C{tr}.
    Typically invoked as

        - Q = IKINE(ROBOT, T)
        - Q = IKINE(ROBOT, T, Q)
        - Q = IKINE(ROBOT, T, Q, M)

    Uniqueness
    ==========
    Note that the inverse kinematic solution is generally not unique, and 
    depends on the initial guess C{q} (which defaults to 0).

    Iterative solution
    ==================
    Solution is computed iteratively using the pseudo-inverse of the
    manipulator Jacobian.

    Such a solution is completely general, though much less efficient 
    than specific inverse kinematic solutions derived symbolically.

    This approach allows a solution to obtained at a singularity, but 
    the joint angles within the null space are arbitrarily assigned.

    Operation on a trajectory
    =========================
    If C{tr} is a list of transforms (a trajectory) then the solution is calculated
    for each transform in turn.  The return values is a matrix with one row for each
    input transform.  The initial estimate for the iterative solution at 
    each time step is taken as the solution from the previous time step.

    Fewer than 6DOF
    ===============
    If the manipulator has fewer than 6 DOF then this method of solution
    will fail, since the solution space has more dimensions than can
    be spanned by the manipulator joint coordinates.  In such a case
    it is necessary to provide a mask matrix, C{m}, which specifies the 
    Cartesian DOF (in the wrist coordinate frame) that will be ignored
    in reaching a solution.  The mask matrix has six elements that
    correspond to translation in X, Y and Z, and rotation about X, Y and
    Z respectively.  The value should be 0 (for ignore) or 1.  The number
    of non-zero elements should equal the number of manipulator DOF.

    For instance with a typical 5 DOF manipulator one would ignore
    rotation about the wrist axis, that is, M = [1 1 1 1 1 0].


    @type robot: Robot instance
    @param robot: The robot
    @type tr: homgeneous transformation
    @param tr: End-effector pose
    @type q: vector
    @param q: initial estimate of joint coordinate
    @type m: vector
    @param m: mask vector
    @rtype: vector
    @return: joint coordinate
    @see: L{fkine}, L{tr2diff}, L{jacbo0}, L{ikine560}
    """
     
    #solution control parameters
    from numpy import mat,inf,multiply,vstack
    from numpy.linalg import pinv
    import jacobian as Jac
    print 'args', args
    
    n = robot.n

    
    if q0 == None:
        q0 = mat(zeros((n,1)))
    else:
        q0 = mat(q0).flatten().T
        
    if q0 != None and m != None:
        m = mat(m).flatten().T
        if len(m)!=6:
            error('Mask matrix should have 6 elements')
        if len(m.nonzero()[0].T)!=robot.n:
            error('Mask matrix must have same number of 1s as robot DOF')
    else:
        if n<6:
            print 'For a manipulator with fewer than 6DOF a mask matrix argument should be specified'
        m = mat(ones((6,1)))

    def solve(robot, tr, q, mask, ilimit=1000, stol=1e-6, gamma=1):
        print ilimit, stol, gamma
        nm = inf
        count = 0
        while nm > stol:
            e = multiply( tr2diff(fkine(robot, q.T),tr), mask )
            #dq = pinv(Jac.jacob0(robot, q.T)) * e
            dq = Jac.jacob0(robot, q.T).T * e
            q += gamma*dq
            nm = norm(e)
            count += 1
            if count > ilimit:
                error("Solution wouldn't converge")
        print count, 'iterations'
        return q

    if isinstance(tr, list):
        #trajectory case
        qt = mat(zeros((0,n)))
        for T in tr:
            q = solve(robot, T, q0, m, **args)
            qt = vstack( (qt, q.T) )
        return qt
    elif ishomog(tr):
        #single xform case
        q = solve(robot, tr, q0, m, **args)
        print q
        qt = q.T
        return qt
    else:
        error('tr must be 4*4 matrix')