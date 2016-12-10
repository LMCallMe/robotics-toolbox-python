__doc__ = """
The Robotics Toolbox for Python
Based on the Matlab version
Peter Corke 2007

use sympy
"""

######################
#   Import Section   #
######################

# Import transformations section
from transform import *

from jacobian import *

from kinematics import *

from Link import *

from Robot import *

doc = """
Robotics Toolbox for Python
Based on Matlab Toolbox Version 7  April-2002

What's new.
  Readme      - New features and enhancements in this version.

Homogeneous transformations
  eul2tr      - Euler angle to transform 
  oa2tr       - orientation and approach vector to transform 
  rotx        - transform for rotation about X-axis 
  roty        - transform for rotation about Y-axis 
  rotz        - transform for rotation about Z-axis 
  rpy2tr      - roll/pitch/yaw angles to transform 
  tr2eul      - transform to Euler angles 
  tr2rot      - transform to rotation submatrix
  tr2rpy      - transform to roll/pitch/yaw angles
  transl      - set or extract the translational component of a transform 
  trnorm      - normalize a transform
"""
