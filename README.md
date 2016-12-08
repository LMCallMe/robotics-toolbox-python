# robotics-toolbox-python
Robotics Toolbox for Python

This is an old first attempt at creating an open Python version of the venerable Robotics Toolbox for MATLAB.  
The MATLAB toolbox has support for:

* mobile robots
  - vehicle kinematic models and controllers
  - path planners (distance xform, D*, PRM, lattice, RRT)
  - dead-reckoning, localization, mapping, SLAM
  
* robot manipulator arms
  - kinematics forward and inverse
  - Jacobians
  - rigid-body dynamics
  
* common datastructures for
  - SO2/SE2 planar rotations and rigid-body motion
  - SO3/SE3 3D and rigid-body motion
  - quaternions
  - twists, Plucker lines
  
With matplotlib, scipy, numpy and jupyter it should be possible to create a very effective open robotics environment.

# lmcallme

## install virtualenv, virtualenvwrapper:

``` shell
$ pip install virtualenv
$ pip install virtualenvwrapper # linux
$ pip install virtualenvwrapper-win # windows

```

when on Linux, put into ~/.bashrc:

``` shell
export WORKON_HOME=$HOME/.virtualenvs
export PROJECT_HOME=$HOME/Devel
export VIRTUALENVWRAPPER_SCRIPT=/usr/local/bin/virtualenvwrapper.sh
source /usr/local/bin/virtualenvwrapper_lazy.sh
```

## setup

### make env

``` shell
$ mkvirtualenv robot_toolbox
```

### use env

``` shell
$ workon robot_toolbox
```

### install matplotlib, numpy and jupyter

``` shell
$ pip install -r requirements.txt
```

### install scipy

#### on linux:

``` shell
$ pip install scipy==0.18.1
```

#### on windows

frist download 
[whl](http://www.lfd.uci.edu/~gohlke/pythonlibs/#scipy) file
then:

``` shell
$ pip install xxx.whl
```

### use jupyter

``` shell
$ jupyter notebook
```