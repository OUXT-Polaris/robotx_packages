# robotx_packages
ROS packages for Maritime RobotX Challenge 2018.
This packages include Simulation,Remote Viewer, Control System, etc...

|master|
|:---:|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/robotx_packages.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/robotx_packages)|
|develop|
|:---:|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/robotx_packages.svg?branch=develop)](https://travis-ci.org/OUXT-Polaris/robotx_packages)|

# Installation
#### 1. Create **catkinized**  workspace.
#### 2. Clone this repository.
```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/OUXT-Polaris/robotx_packages.git
```
#### 3. Download depended packages by rosdep.
```bash
$ cd <catkin_ws>
$ rosdep install -i -r -y --from-paths src --rosdistro kinetic
```
#### 4. Build packages, and set the path for the packages.
```bash
$ cd <catkin_ws>
$ catkin_make
$ source devel/setup.bash
```

# how to launch
## Simulation
roslaunch robotx_gazebo robotx_gazebo.launch

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
