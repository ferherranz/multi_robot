multi_robot
===========

Multirobot package to ROS and Turtlebot.

This package provides an implementation of the multi robot configuration of the turtlebot platform (see https://github.com/turtlebot). It provides a correct configuration of namespaces and frames that allows the robots to perform localization and navigation in real environments. In addition, a decision making example based on smach is provided.
 
## Usage

Multi robot navigation:

 *  **Bring up both robots**


	$ roslaunch multi_robot multi_robot.launch 
 *  **Start Navigation**
 
 
 	$ roslaunch multi_robot multi_robot_navigation.launch


For visualization:

 *  **Start RVIZ**
 
 
    $ rosrun rviz rviz
 *  Load the config file multi_robot_full.rviz allocated in rviz folder