# ur_manipulation
For the Seher project. Demo using UR5e+Schunk EGP 50NNB gripper.   
This package provides only the control node and the gripper URDF for the demo.

# Requirements

1. `universal_robots_ros_driver`
This is the official ROS Industrial driver for the UR series.  
`git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git`
Once cloned, follow all setup isntructions from this package and verify the robot driver is working. This should also instruct you to clone Fmauch's universal robots package.   

2. `ur5e_egp50_moveit_config`
This is the custom built MoveIt! config folder for the robot+gripper. Use master branch from :      
`git clone https://github.com/ipa-kut/ur5e_egp50_moveit_config.git`   

Finally, clone this `ur_manipulation` package as well into the same workspace as well.   

From the workspace directory, install all other requirements using:   
`rosdep install --from-paths src --ignore-src -r -y`   

# Setup
The robot is currently configured to IP address `192.168.56.2` and expects the ROS PC to have the address `192.168.56.1`.   
Subnet mask for both needs to be `255.255.255.0`.   
Once the manual ethernet connection is correctly configured, you should be able to ping the robot from the PC.   

A known bug in ROS Melodic causes the RVIZ render of the UR5e to be messed up. To solve this, either manually run the following command in
the terminal for RVIZ each time, or add it to .bashrc:   
`export LC_NUMERIC="en_US.UTF-8"`

# Bringup

## UR5e: Simulation + Moveit

1. Start the sim:   
`roslaunch ur_manipulation gazebo_ur5e_egp50.launch`   

2. Start moveit + rviz:    
`roslaunch ur5e_egp50_moveit_config ur5e_egp50_moveit_planning_execution.launch sim:=true`   

## UR5e: Real Robot:

1. Start the robot driver with this :   
`roslaunch ur_manipulation ur5e_bringup.launch`   
Once done, in the Teach Pendant, go to Program > URCaps and tap on External Control to add it to the program.    
Start this program by pressing the play button at the bottom, verify in the driver launch window if this was recognised.     
Robot IP is hard coded in launch, change as needed.

2. Start Moveit + RVIZ :   
`roslaunch ur5e_egp50_moveit_config ur5e_egp50_moveit_planning_execution.launch`   

3. Start this demonstrator node :    
`roslaunch ur_manipulation seher_demo.launch`  

## Troublsehooting

If having controller issues, try to alter the default ur5e_moveit_config pkg of fmauch's fork to get it to work:
*ur5_e_moveit_config/launch/move_group.launch* -
Replace:  `move_group/MoveGroupExecuteService`    
With:     `move_group/MoveGroupExecuteTrajectoryAction`

Once done,

1. Start the simulation like this :    
`roslaunch ur5e_egp50_moveit_config gazebo.launch`

2. Start moveit :    
`roslaunch ur5e_egp50_moveit_config ur5e_egp50_moveit_planning_execution.launch sim:=true`

3. Start rviz :    
`roslaunch ur5e_egp50_moveit_config moveit_rviz.launch config:=true`

4. Start this demonstrator node :       
`roslaunch ur_manipulation seher_demo.launch`  

## Simulation with PRBT

1. Start moveit:
`roslaunch prbt_moveit_config moveit_planning_execution.launch`

2. Start endurance test:
`roslaunch ur_manipulation endurance_demo.launch robot:=prbt`

## RVIZ Demo

For rapid prototyping of some manipulation code, you can run a "light" version of the robot for visualization purposes only, This does not launch a sim or use any controllers, and therefore may not reflect real behaviour.

`roslaunch ur5e_egp50_moveit_config demo.launch `

# Notes
Current state of the demonstrator :   
1. Attempts to reset to 'home' position each time.   
2. Pick and Place poses are hardcoded.   
3. The test piece is assumed to be a cube of fixed size, which is loaded and unloaded in/from the planning
scene only between the post pick and place states.      

# TODO
[] Test piece should be present in the planning scene throughout the demo.    
