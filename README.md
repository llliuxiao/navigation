# Navigation

## Install

if using jackal robot in gazebo

```shell
git clone git@gitlab.geometryrobot.com:learning/rl/navigation.git --branch barn_challenge
```

else

```
git clone git@gitlab.geometryrobot.com:learning/rl/navigation.git --branch noetic-devel
```

## Usage

- the launch file are all included in isaac sim repository
- notice that a simple move base was added into move_base package
- the modification compared with ROS stack focuses on global planner

## Reference

A 2D navigation stack that takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base.

Related stacks:

- http://github.com/ros-planning/navigation_msgs (new in Jade+)
- http://github.com/ros-planning/navigation_tutorials
- http://github.com/ros-planning/navigation_experimental

For discussion, please check out the https://groups.google.com/group/ros-sig-navigation mailing list.