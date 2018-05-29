# robotx_packages
ROS packages for Maritime RobotX Challenge 2018.
This packages include Simulation,Remote Viewer, Control System, etc...

|master|
|:---:|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/robotx_packages.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/robotx_packages)|

# how to launch
## Simulation
roslaunch robotx_gazebo_plugins wam-v_gazebo.launch

# branch model

#### /master
branch for competitions (all nodes should be work completely)

#### /devel
branch for development (include under developing nodes)

#### /devel/feature/(feature_name)
branch for developing each feature

#### /devel/fix/(bug_name)
branch for fixing each bug

/devel/feature/(feature_name) and /devel/fix/(bug_name) feature branch merged to /master branch
