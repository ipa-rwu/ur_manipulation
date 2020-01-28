# ur_manipulation
For the Seher project. Demo using UR5e+Schunk EGP 50NNB gripper.   
This package provides only the control node and the gripper URDF for the demo.

# Requirements

<!--1. `universal_robots`
Provides base robot description. Customised version to work with the gripper is available under master branch from:   
`git clone https://github.com/ipa-kut/universal_robot.git`-->
1. `universal_robots_ros_driver`
This is a private repo under the ROS Industrial project which provides a more up-to-date driver for the UR5e series.  
`git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git`
Once cloned, follow all setup isntructions from this package and verify the robot driver is working.   
<!--3. `ur_modern_driver`
Package #2 builds upon this package, which can be used directly from the branch add-e-series-support:   
`git clone https://github.com/plusone-robotics/ur_modern_driver.git -b add-e-series-support`   -->
2. `ur5e_egp50_moveit_config`
This is the custom built MoveIt! config folder for the robot+gripper. Use master branch from :      
`git clone https://github.com/ipa-kut/ur5e_egp50_moveit_config.git`   

Finally, clone this package as well into the workspace.   

Install all other requirements using:   
`rosdep install --from-paths src --ignore-src -r -y`   

# Setup
The robot is currently configured to IP address `192.168.56.2` and expects the ROS PC to have the address `192.168.56.1`.   
Subnet mask for both needs to be `255.255.255.0`.   
Once the manual ethernet connection is correctly configured, you should be able to ping the robot from the PC.   

A known bug in ROS Melodic causes the RVIZ render of the UR5e to be messed up. To solve this, either manually run the following command in
the terminal for RVIZ each time, or add it to .bashrc:   
`export LC_NUMERIC="en_US.UTF-8"`

# Bringup

1. Start the robot driver with this :   
`roslaunch ur_manipulation ur5e_bringup.launch`
Once done, in the Teach Pendant, go to Program > URCaps > External Control and start this program.   
Robot IP is hard coded in launch, change as needed.

2. Start Moveit :   
`roslaunch ur5e_egp50_moveit_config ur5e_egp50_moveit_planning_execution.launch`   

3. Start Rviz (required if user prompts are enabled in compile time) :    
`roslaunch ur5e_egp50_moveit_config moveit_rviz.launch`

4. Start this demonstrator node :    
`roslaunch ur_manipulation seher_demo.launch`   


# Notes
Current state of the demonstrator :   
1. Attempts to reset to 'home' position each time.   
2. Pick and Place poses are hardcoded.   
3. The test piece is assumed to be a cube of fixed size, which is loaded and unloaded in/from the planning
scene only between the post pick and place states.      


# TODO
[] Test piece should be present in the planning scene throughout the demo.   
[] Update and Test the demo in endless function mode.   
[] Dynamically receive the Pick and Place poses (from camera or otherwise).   






