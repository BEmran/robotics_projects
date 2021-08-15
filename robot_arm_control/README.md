# Coding Test

## Setup

- [Setup ROS2](https://index.ros.org/doc/ros2/Installation/Foxy/)

Note, This has been tested on Ubuntu 20.04 with ROS2 Foxy.

You are welcome to try on another platform, but there is no guarantee that 

## Build

Create a workspace directory, and put this repository inside it.
For example,

```
jonathan@MakiseLinux:~/code/code_test_ws tree -L 2
.
└── src
    ├── coding_test
    ├── coding_test_msgs
    └── README.md

3 directories, 1 file
```

From this directory, run

`colcon build --merge --symlink`

## Run Tests

`colcon test --merge`

To get result information

`colcon test-result --verbose`

To get verbose information while running test

`colcon test --merge --event-handlers console_cohesion+ console_direct+`

## Run The Demo

First, source the workspace (after you have built it)

`source install/setup.bash`

`ros2 launch coding_test simple_robot.launch.py`

You should see rviz open, with a 3 jointed robot, moving between some random positions.

# Your Task

This repository includes two nodes.
1. DummyJointController
2. HighLevelController

Currently, the DummyJointController is responsible for emulating a robot hardware driver / controller.
It publishes sensor_msgs/msg/JointState, but currently it moves the joints between random positions.
You will need to modify DummyJointController to be a ROS Action Server for coding_test_msgs/action/MoveJoints.
This will enable other nodes in the system to control the robot.

You might have noticed that HighLevelController is printing a warning, that "coding_test_msgs::action::MoveToPoint server is not ready".
Your second task, is to implement that server.
Create a new node (in any language), that is an Action Server for MoveToPoint, and leverages MoveJoints to move the robot.

The simple robot in this example can only move in 2 dimensions. Specifically, its end effector is only capable to only move in x and z.
You can assume the value for y will always be 0.

The `target_point` of MoveToPoint will contain an x and z (y = 0), position to move the end effector to.
You must convert that into the set of joint positions that will get the end effector as close to the point specified as possible.
You may solve this however you see fit, however I will point out that the 2D constraint makes it possible without general IK.

A few of things to keep in mind:
- The action feedback of MoveJoints and MoveToPoint should publish at the same rate that JointStates current publishes. Note that the feedback reflects the current state, as the robot moves towards the target set by MoveJoints.
- When implementing MoveJoints, the velocity limit on joint movement must remain the same. It absolutely must not move instantaneously.
- You will need to add your new node to the launch file (coding_test/config/simple_robot.launch.py)
- Your new node should work as a server to HighLevelController, without any modification to the latter. Since it publishes random positions, it can be your first "test case".
- You must implement additional test cases for your new node, and add tests to cover any features you add to the existing nodes.
- You must document and comment your work. Use what is already here as a guideline for quality/quantity.
- You may not use any additional libraries, outside of the ROS libraries that are already leveraged by the nodes provided.

# Submission
Submit your entire (source) workspace as a tarball.

# Credits

The URDF in this package (simple_robot.urdf) is adapted from [ros2 demos](https://github.com/ros2/demos/blob/foxy/dummy_robot/dummy_robot_bringup/launch/single_rrbot.urdf)
which is licensed under [Apache 2.0](https://github.com/ros2/demos/blob/foxy/LICENSE), and is assumed to copyright of the Open Source Robotics Foundation, Inc.
