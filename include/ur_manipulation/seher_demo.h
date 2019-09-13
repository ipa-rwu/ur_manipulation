#ifndef SEHER_DEMO_H
#define SEHER_DEMO_H

//Includes
#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

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
  SeherDemo(int max_trials, std::string user_prompts);
  ~SeherDemo();

  const std::string GROUP_MANIP = "manipulator";
  const std::string GROUP_GRIPP = "endeffector";
  const std::string COMMAND_ADD = "add";
  const std::string COMMAND_REMOVE = "remove";

  int max_trials;
  bool user_prompts;
  ros::Publisher planning_scene_diff_publisher;

  void addCollissionObjects();
  void addOrRemoveTestPieceCollissionObjectWRTRobot(std::string command);
  void checkTrialsLimit(int trials);
  bool gripperClose(ros::NodeHandle nh);
  bool gripperOpen(ros::NodeHandle nh);
  moveit::planning_interface::MoveGroupInterface::Plan getPlanToPoseTarget(geometry_msgs::Pose target_pose, int trials, std::string display_name);
  void initialiseMoveit(ros::NodeHandle nh);
  bool moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan);
  void moveToNamedTarget(std::string target);
  void pickAtPoseFromHeight(geometry_msgs::Pose target_pose, double height, ros::NodeHandle nh);
  void placeAtPoseFromHeight(geometry_msgs::Pose target_pose, double height, ros::NodeHandle nh);
  void printBasicInfo();
  bool comparePoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double delta_posistion=0.05, double delta_orientation=0.01);

private:

  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group ;
  moveit_visual_tools::MoveItVisualTools *visual_tools;
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  const int IO_SERVICE_FUN_LEVEL_ = 1;   // Not exactly sure what this is, but 1 seems to work. If it fails, try 2.

  const double BASE_OFFSET_FROM_BACK_WALL_ = 0.28;   //28cm
  const double BASE_OFFSET_FROM_LEFT_WALL_ = 0.46;   //46cm
  const double BASE_OFFSET_FROM_RIGHT_WALL_ = 0.5;   //50cm
  const double TOTAL_INNER_CELL_Y_DIMENSION_ = 1.47; //1470cm
  const double TOTAL_INNER_CELL_X_DIMENSION_ = 0.96; //960cm
  const double TOTAL_INNER_CELL_Z_DIMENSION = 1.15;  //115cm

};


#endif // SEHER_DEMO_H
