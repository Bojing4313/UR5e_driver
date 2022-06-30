# UR5-driver 🕹 + 🤖 = 👍
Pure python script to communicate with UR5/e robot via TCP/IP network.

## Motivation

Development of this driver was inspired by the python package writen by @gouxiangchen, Github: https://github.com/gouxiangchen/UR5-control-with-RG2.git

Instead of using ROS, this package uses TCP/IP network protocol to communicate with the UR5 serials robot in a pure python way.

Major improvments were made on surveillance of gesture using rtde and movement control to avoid potential error while commencing commands.

## Code Structure
The main functions are included in ` UR5e.py `.

All major commands were writen according to and are avialble on the manuscript from Universal Robot.

`get_pose()` is called when you want to get the current tool center position of UR5/e (x, y, z, rx, ry, rz) in reference to the world reference frame.

`get_joints()` is called when you want to get the current angular positions of UR5/e's 6 joints.

`move_l()` is called when you want to control UR5/e to move to a target tool center position (x, y, z, rx, ry, rz) in reference to the world reference frame.

`move_j()` is called when you want to control UR5/e to move to a target gesture by assigning specified angular position (theta_1, theta_2, theta_3, theta_4, theta_5, theta_6), or move to a tool center position (x, y, z, rx, ry, rz) in reference to the world reference when the argument 'joint' is False.

`increase_move()` is called when you want to control UR5 to move a increase distance from current position (delta_x, delta_y, delta_z, delta_theta), where delta_theta is the increase distance of the top-down orientation

`operate_gripper()` is used control the gripper to open/close by passing bool value to the argument `grip`, width control is not avialble since various types of gripper may be present.

`check_grasp()` is used to check if the gripper is grasping an object. while the gripper is not fully closed, the function returns True

`move_down()` move the tcp down to a desired z coordinate

`move_up()` move the tcp up to a desired z coordinate

`grasp()` open gripper then move down then close gripper then move up then check the grasp.

`go_home()` will first check the current tcp position then control UR5 to move to the initial position

file `util.py` contains some coordinate transformation functions.

## Enviroment setup
To setup the UR5e driver, all you need is a working python 3.7 installation and the and the file `rtde.py` and `serialize.py` from the UR5 official examples without any modification. Prior to the connection, you need to figure out the IP address of your UR5/e robot and change it in the 'ROBOT_HOST' constant. After that, you need to set your robot to the Remote Mode on the Teach Pendant.
