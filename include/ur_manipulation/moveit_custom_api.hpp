#ifndef MOVEIT_CUSTOM_API_HPP
#define MOVEIT_CUSTOM_API_HPP

#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "tf/transform_datatypes.h"
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <string>

#include "ur_msgs/SetIO.h"

class MoveitCustomApi
{
public:
  MoveitCustomApi();
  MoveitCustomApi(std::string user_prompts);
  ~MoveitCustomApi();

  const std::string COMMAND_ADD = "add";
  const std::string COMMAND_REMOVE = "remove";

  int max_trials;
  bool user_prompts;
  unsigned int failure_counter_;
  ros::Publisher planning_scene_diff_publisher;
  moveit::planning_interface::MoveGroupInterface *move_group;

  void addCollisionObjects();
  void addOrRemoveTestPieceCollisionObjectWRTRobot(std::string command);
  void checkTrialsLimit(int trials);
  bool gripperClose(ros::NodeHandle nh);
  bool gripperOpen(ros::NodeHandle nh);
  moveit::planning_interface::MoveGroupInterface::Plan getPlanToPoseTarget(geometry_msgs::Pose target_pose, int trials, std::string display_name);
  void initialiseMoveit(ros::NodeHandle nh, std::string prompts = "true");
  bool moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan);
  void moveToNamedTarget(std::string target);
  void pickAtPoseFromHeight(geometry_msgs::Pose target_pose, double height, ros::NodeHandle nh, bool do_gripper = true);
  void placeAtPoseFromHeight(geometry_msgs::Pose target_pose, double height, ros::NodeHandle nh, bool do_gripper = true);
  void printBasicInfo();
  bool comparePoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double delta_posistion = 0.05, double delta_orientation = 0.01);
  moveit::planning_interface::MoveGroupInterface::Plan getCartesianPathPlanToPose(geometry_msgs::Pose target_pose, std::string display_label, double eef_step = 0.01, double jump_threshold = 0.0);
  void sleepSafeFor(double duration);
  void executeCartesianTrajForWaypoints(std::vector<geometry_msgs::Pose> waypoints, double eef = 0.001, double jump_thresh = 0.0);
  void executeCartesianTrajtoPose(geometry_msgs::Pose target, std::string label);
  void adjustTrajectoryToFixTimeSequencing(moveit_msgs::RobotTrajectory &trajectory);

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group;
  moveit_visual_tools::MoveItVisualTools *visual_tools;
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  const int IO_SERVICE_FUN_LEVEL_ = 1; // Not exactly sure what this is, but 1 seems to work. If it fails, try 2.
  std::string robot_name_;
  std::string group_manip_;
  double robot_settle_time_;
  moveit::core::RobotModelConstPtr robot_model_;

  double BASE_OFFSET_FROM_BACK_WALL_;
  double BASE_OFFSET_FROM_LEFT_WALL_;
  double BASE_OFFSET_FROM_RIGHT_WALL_;
  double BASE_OFFSET_FROM_BOTTOM_;
  double TOTAL_INNER_CELL_Y_DIMENSION_;
  double TOTAL_INNER_CELL_X_DIMENSION_;
  double TOTAL_INNER_CELL_Z_DIMENSION_;
};

#endif // MOVEIT_CUSTOM_API_HPP
