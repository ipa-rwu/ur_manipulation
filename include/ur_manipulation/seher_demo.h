#ifndef SEHER_DEMO_H
#define SEHER_DEMO_H

//Includes
#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/PoseStamped.h>

#include "ur_msgs/SetIO.h"

class SeherDemo
{
public:
  SeherDemo();
  ~SeherDemo();

  const std::string GROUP_MANIP = "manipulator";
  const std::string GROUP_GRIPP = "endeffector";

  int max_trials;
  bool user_prompts;

  void addCollissionObjects();
  void checkTrialsLimit(int trials);
  bool gripperClose(ros::NodeHandle nh);
  bool gripperOpen(ros::NodeHandle nh);
  void initialiseMoveit();
  bool moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan);
  void moveToNamedTarget(std::string target);
  moveit::planning_interface::MoveGroupInterface::Plan getPlanToPoseTarget(geometry_msgs::Pose target_pose, int trials, std::string display_name);
  void printBasicInfo();


private:

  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group ;
  moveit_visual_tools::MoveItVisualTools *visual_tools;
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  const int IO_SERVICE_FUN_LEVEL_ = 1;   // Not exactly sure what this is, but 1 seems to work. If it fails, try 2.
};


#endif // SEHER_DEMO_H
