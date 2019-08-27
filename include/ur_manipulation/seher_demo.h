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

class SeherDemo
{

public:
  SeherDemo() {};
  ~SeherDemo() {};

  const std::string GROUP_MANIP = "manipulator";
  const std::string GROUP_GRIPP = "endeffector";

  void printBasicInfo();

  void initialiseMoveit();


private:

  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group ;
  moveit_visual_tools::MoveItVisualTools *visual_tools;
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();



};


#endif // SEHER_DEMO_H
