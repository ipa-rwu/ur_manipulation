/// TODO:
/// [] Test the test piece collision object add/remove functionality more, including trajectories - Test object currently left nehind when the place operation is done. Fix this.

#include "ur_manipulation/moveit_custom_api.hpp"

#include <std_msgs/Header.h>
#include <std_msgs/Int64.h>

void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double &roll, double &pitch, double &yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation, quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);
}

MoveitCustomApi::MoveitCustomApi() : user_prompts(true)
{
}

MoveitCustomApi::MoveitCustomApi(std::string user_prompts)
{
  this->user_prompts = (user_prompts == "True" || user_prompts == "true") ? true : false;
}

MoveitCustomApi::~MoveitCustomApi() {}

void MoveitCustomApi::printBasicInfo()
{
  ROS_INFO("------------------------------------------------------");
  ROS_INFO_STREAM("Planning frame: " << move_group->getPlanningFrame().c_str());
  ROS_INFO_STREAM("End effector link: " << move_group->getEndEffectorLink().c_str());
  ROS_INFO_STREAM("User prompts : " << (this->user_prompts ? "True" : "False"));
  ROS_INFO_STREAM("Max trials : " << this->max_trials);
  ROS_INFO_STREAM("Robot settle time : " << this->robot_settle_time_);
  ROS_INFO("------------------------------------------------------");
}

bool MoveitCustomApi::
      comparePoses(geometry_msgs::Pose pose1,
                   geometry_msgs::Pose pose2,
                   double delta_posistion,
                   double delta_orientation)
{
  if (abs(pose1.position.x - pose2.position.x) <= delta_posistion &&
      abs(pose1.position.y - pose2.position.y) <= delta_posistion &&
      abs(pose1.position.z - pose2.position.z) <= delta_posistion &&
      abs(pose1.orientation.x - pose2.orientation.x) <= delta_orientation &&
      abs(pose1.orientation.y - pose2.orientation.y) <= delta_orientation &&
      abs(pose1.orientation.z - pose2.orientation.z) <= delta_orientation &&
      abs(pose1.orientation.w - pose2.orientation.w) <= delta_orientation)
  {
    return true;
  }
  else
  {
    tf::Quaternion q1, q2;
    tf::quaternionMsgToTF(pose1.orientation, q1);
    tf::quaternionMsgToTF(pose2.orientation, q2);
    if (tf::angleShortestPath(q1, q2) == 0)
    {
      return true;
    }
    return false;
  }
}

void MoveitCustomApi::
    executeCartesianTrajForWaypoints(std::vector<geometry_msgs::Pose> waypoints,
                                     double eef,
                                     double jump_thresh)
{
  namespace rvt = rviz_visual_tools;
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = 0.0;
  while (fraction < 0.5)
  {
    fraction = move_group->computeCartesianPath(waypoints, eef, jump_thresh, trajectory);
  }
  adjustTrajectoryToFixTimeSequencing(trajectory);

  // Visualize the plan in RViz
  ROS_INFO_STREAM("Visualizing Cartesian Path plan with waypoints (" << fraction * 100 << "% acheived)");
  visual_tools->deleteAllMarkers();
  visual_tools->publishText(text_pose, "Cartesian path", rvt::WHITE, rvt::XLARGE);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);
  visual_tools->trigger();
  if (user_prompts)
    visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  move_group->execute(my_plan);
}

void MoveitCustomApi::adjustTrajectoryToFixTimeSequencing(moveit_msgs::RobotTrajectory &trajectory)
{
  std::vector<ros::Duration> times_from_start;
  times_from_start.resize(trajectory.joint_trajectory.points.size());

  for (int i = 0; i < times_from_start.size(); i++)
  {
    times_from_start[i] = trajectory.joint_trajectory.points[i].time_from_start;
  }

  // Adjust starting from point 2 i.e. index 1
  bool adjusted_flag = false;
  for (int i = 1; i < times_from_start.size() - 1; i++)
  {
    if (times_from_start[i] == ros::Duration(0))
    {
      ros::Duration prev = times_from_start[i];
      times_from_start[i] = ros::Duration((times_from_start[i - 1].toSec() + times_from_start[i + 1].toSec()) / 2.0);
      ROS_WARN_STREAM("Recomputing point " << i << " from " << prev
                      << " to: " << times_from_start[i - 1] << " + "
                      << times_from_start[i + 1] << " = " << times_from_start[i]);
      adjusted_flag = true;
    }
  }

  if (times_from_start.size() > 1 && times_from_start[times_from_start.size() - 1] == ros::Duration(0))
  {
    ROS_WARN_STREAM("Final point in trajectory has 0 timestamp, incrementing logically");
    times_from_start[times_from_start.size() - 1] = times_from_start[times_from_start.size() - 2] + ros::Duration(0.1);
    adjusted_flag = true;
  }

  if (adjusted_flag)
  {
    for (int i = 0; i < times_from_start.size(); i++)
    {
      trajectory.joint_trajectory.points[i].time_from_start = times_from_start[i];
    }
  }
}

moveit::planning_interface::MoveGroupInterface::Plan MoveitCustomApi::
getCartesianPathPlanToPose(geometry_msgs::Pose target_pose,
                           std::string display_label,
                           double eef_step,
                           double jump_threshold)
{
  namespace rvt = rviz_visual_tools;
  move_group->setStartStateToCurrentState();

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(move_group->getCurrentPose().pose);
  waypoints.push_back(target_pose);

  eef_step = 0.005;
  jump_threshold = 0.0;

  moveit_msgs::RobotTrajectory trajectory;
  double fraction = 0.0;
  while (fraction < 0.5)
  {
    fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  }
  adjustTrajectoryToFixTimeSequencing(trajectory);
  // time parameterization params
  robot_trajectory::RobotTrajectory r_traj(robot_model_, joint_model_group);
  r_traj.setRobotTrajectoryMsg(*(move_group->getCurrentState()), trajectory);
  double max_vel_scaling_factor = 0.5;
  double max_acc_scaling_factor = 0.5;
  // using time optimal trajectory generation, parameter chosen randomly
  double path_tolerance = 0.1;
  double resample_dt = 0.1;
  double min_angle_change = 0.001;
  trajectory_processing::TimeOptimalTrajectoryGeneration TOTG(path_tolerance, resample_dt, min_angle_change);
  if (TOTG.computeTimeStamps(r_traj, max_vel_scaling_factor, max_acc_scaling_factor))
  {
    r_traj.getRobotTrajectoryMsg(trajectory);
  }
  else
  {
    ROS_WARN("TOTG parameterization failed.");
  }

  // Visualize the plan in RViz
  ROS_INFO_STREAM("Visualizing Cartesian Path plan to " << display_label << " (" << fraction * 100 << "% acheived)");
  visual_tools->deleteAllMarkers();
  visual_tools->publishText(text_pose, "Cartesian path", rvt::WHITE, rvt::XLARGE);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::MEDIUM);
  visual_tools->trigger();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  // display trajectory with time parameterization
  ros::NodeHandle nh_display = move_group->getNodeHandle();
  ros::Publisher display_publisher =
      nh_display.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_traj;
  display_traj.model_id = move_group->getRobotModel()->getName();
  display_traj.trajectory_start = my_plan.start_state_;
  display_traj.trajectory.push_back(trajectory);
  display_publisher.publish(display_traj);

  return my_plan;
}

void MoveitCustomApi::sleepSafeFor(double duration)
{
  ros::Time start = ros::Time::now();
  while (ros::Time::now() - start <= ros::Duration(duration))
  {
    ros::spinOnce();
  }
}

bool MoveitCustomApi::moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{
  if (user_prompts)
  {
    std::string message = "Planning completed, press Next to execute";
    visual_tools->prompt(message);
  }

  move_group->setStartStateToCurrentState();
  return move_group->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ;
}

void MoveitCustomApi::addCollisionObjects()
{
  namespace rvt = rviz_visual_tools;
  moveit_msgs::AttachedCollisionObject object;
  object.link_name = move_group->getPlanningFrame();
  object.object.header.frame_id = move_group->getPlanningFrame();
  object.object.id = "Floor";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = std::abs(TOTAL_INNER_CELL_Y_DIMENSION_);
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_ - BASE_OFFSET_FROM_RIGHT_WALL_) / 2; // Not perfectly symmetrical.
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_ / 2 - BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = BASE_OFFSET_FROM_BOTTOM_;

  // Since we are attaching the object to the robot base
  // we want the collision checker to ignore collisions between the object and the robot base
  object.touch_links = std::vector<std::string>{"base_link"};
  moveit_msgs::PlanningScene planning_scene;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Ceiling";

  // Define a box to add to the world.
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = std::abs(TOTAL_INNER_CELL_Y_DIMENSION_);
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_ - BASE_OFFSET_FROM_RIGHT_WALL_) / 2; // Not perfectly symmetrical.
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_ / 2 - BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION_;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Left Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = std::abs(TOTAL_INNER_CELL_Y_DIMENSION_);
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION_;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = BASE_OFFSET_FROM_RIGHT_WALL_;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_ / 2 - BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION_ / 2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Right Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = std::abs(TOTAL_INNER_CELL_Y_DIMENSION_);
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION_;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -BASE_OFFSET_FROM_LEFT_WALL_;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_ / 2 - BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION_ / 2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Back Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = 0;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION_;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_ - BASE_OFFSET_FROM_RIGHT_WALL_) / 2;
  box_pose.position.y = -BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION_ / 2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  object.object.id = "Bottom";
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = std::abs(TOTAL_INNER_CELL_Y_DIMENSION_);
  primitive.dimensions[2] = BASE_OFFSET_FROM_BOTTOM_;

  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_ - BASE_OFFSET_FROM_RIGHT_WALL_) / 2;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_ / 2 - BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = BASE_OFFSET_FROM_BOTTOM_ / 2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  ROS_INFO_NAMED("seher_demo", "Adding collision objects into the world");
}

void MoveitCustomApi::addOrRemoveTestPieceCollisionObjectWRTRobot(std::string command)
{
  if (command != COMMAND_ADD && command != COMMAND_REMOVE)
  {
    ROS_ERROR_STREAM("Unknown test piece collision object manipulation command : "
                     << command << ". Expecting " << COMMAND_ADD << " or " << COMMAND_REMOVE);
    return;
  }

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = move_group->getEndEffectorLink();
  attached_object.object.header.frame_id = move_group->getEndEffectorLink();
  attached_object.object.id = "box";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive1;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 0.02;
  primitive1.dimensions[1] = 0.02;
  primitive1.dimensions[2] = 0.02;

  // Define a pose for the box (specified relative to frame_id
  geometry_msgs::Pose box_pose1;
  box_pose1.orientation.w = 1.0;
  box_pose1.position.x = 0.0;
  box_pose1.position.y = 0.0;
  box_pose1.position.z = 0.01; // Object Z dimension/2

  attached_object.object.primitives.push_back(primitive1);
  attached_object.object.primitive_poses.push_back(box_pose1);
  attached_object.object.operation =
      (command == COMMAND_ADD) ? attached_object.object.ADD : attached_object.object.REMOVE;

  attached_object.touch_links = std::vector<std::string>{move_group->getEndEffectorLink(), "egp50_pincer_link"};

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.robot_state.is_diff = true;
  planning_scene.is_diff = true;
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);
}

void MoveitCustomApi::checkTrialsLimit(int trials)
{
  if (trials > max_trials)
  {
    ROS_WARN_STREAM("Selected number of trials for execution : "
                    << trials << ", is more than allowed max of : " << max_trials);
  }
}

bool MoveitCustomApi::gripperOpen(ros::NodeHandle nh)
{
  ur_msgs::SetIO io_msg;
  io_msg.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);
  io_msg.request.pin = static_cast<int8_t>(1); // Pin 1 is open
  io_msg.request.state = 1;
  ros::ServiceClient client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  if (client.call(io_msg))
  {
    ROS_INFO_STREAM("Open gripper initialise : " << ((io_msg.response.success == 0) ? "Failed" : "Succeeded"));
    sleepSafeFor(robot_settle_time_);
    io_msg.request.state = 0;
    if (client.call(io_msg))
    {
      ROS_INFO_STREAM("Open gripper conclude : " << ((io_msg.response.success == 0) ? "Failed" : "Succeeded"));
      return true;
    }
    else
    {
      ROS_INFO_STREAM("Open gripper conclude : Failed");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM("Open gripper initialise : Failed");
    return false;
  }
}

bool MoveitCustomApi::gripperClose(ros::NodeHandle nh)
{
  ur_msgs::SetIO io_msg;
  io_msg.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);
  io_msg.request.pin = static_cast<int8_t>(0); // Pin 0 is close
  io_msg.request.state = 1;
  ros::ServiceClient client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  if (client.call(io_msg))
  {
    ROS_INFO_STREAM("Close gripper initialise :  " << ((io_msg.response.success == 0) ? "Failed" : "Succeeded"));
    sleepSafeFor(robot_settle_time_);
    io_msg.request.state = 0;
    if (client.call(io_msg))
    {
      ROS_INFO_STREAM("Close gripper conclude :  " << ((io_msg.response.success == 0) ? "Failed" : "Succeeded"));
      return true;
    }
    else
    {
      ROS_INFO_STREAM("Close gripper conclude : Failed");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM("Close gripper initialise : Failed");
    return false;
  }
}

void MoveitCustomApi::initialiseMoveit(ros::NodeHandle nh, std::string prompts)
{
  user_prompts = (prompts == "True" || prompts == "true") ? true : false;
  failure_counter_ = 0;
  namespace rvt = rviz_visual_tools;
  nh.param<std::string>("robot", robot_name_, "robot");
  nh.param<std::string>("group_manip", group_manip_, "manip");
  nh.param<int>("max_planning_attempts", max_trials, 3);
  nh.param<double>("robot_settle_time", robot_settle_time_, 0.5);
  // load parameters for collision objects
  if (!(nh.getParam("collision/backwall", BASE_OFFSET_FROM_BACK_WALL_) ||
        nh.getParam("collision/leftwall", BASE_OFFSET_FROM_LEFT_WALL_) ||
        nh.getParam("collision/rightwall", BASE_OFFSET_FROM_RIGHT_WALL_) ||
        nh.getParam("collision/bottom", BASE_OFFSET_FROM_BOTTOM_)))
    ROS_ERROR("Collision object not set");
  if (!(nh.getParam("cell/x_dim", TOTAL_INNER_CELL_X_DIMENSION_) ||
        nh.getParam("cell/y_dim", TOTAL_INNER_CELL_Y_DIMENSION_) ||
        nh.getParam("cell/z_dim", TOTAL_INNER_CELL_Z_DIMENSION_)))
    ROS_ERROR("Cell dimension not set");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = std::make_shared<moveit::core::RobotModel>(robot_model_loader.getURDF(), robot_model_loader.getSRDF());

  move_group = new moveit::planning_interface::MoveGroupInterface(group_manip_);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(group_manip_);

  visual_tools = new moveit_visual_tools::MoveItVisualTools(move_group->getPlanningFrame().c_str());
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();
  text_pose.translation().z() = 0.75;
  visual_tools->publishText(text_pose, boost::to_upper_copy<std::string>(robot_name_), rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
}

void MoveitCustomApi::moveToNamedTarget(std::string target)
{

  move_group->setNamedTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  int trials = 0;
  while (trials++ < max_trials)
  {
    if (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      break;
    }
    else
    {
      ROS_ERROR_STREAM("Named target planning failed, consider quitting");
    }
  }

  ROS_INFO_STREAM("Visualising plan for: " << target);
  visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools->trigger();

  if (user_prompts)
  {
    std::string display_text = "Moving to named target : " + target + ", press Next to begin";
    visual_tools->prompt(display_text);
  }

  move_group->move();
}

void MoveitCustomApi::executeCartesianTrajtoPose(geometry_msgs::Pose target, std::string label)
{

  int trial = 0;
  while (trial < max_trials)
  {
    if (moveGroupExecutePlan(getCartesianPathPlanToPose(target, label)))
    {
      return;
    }
    ROS_ERROR_STREAM("Execution to " << label << " trial " << trial++ << " failed. Reattempting");
    failure_counter_++;
  }
  ROS_ERROR_STREAM("Maxx execution attempts reached, aborting program");
  ros::shutdown();
  exit(-1);
}

void MoveitCustomApi::
  pickAtPoseFromHeight(geometry_msgs::Pose target_pose,
                       double height, ros::NodeHandle nh,
                       bool do_gripper)
/** Assuming  the provided @param{target_pose}is the pose of the target object itself,
 * the gripper first goes to the same XY lcoation, but at a height of @param{height} above it,
 * then moves down to grasp it,
 * and then moves back up.
 */
{
  // Make sure gripper is open
  if (do_gripper)
    gripperOpen(nh);

  // Go to a set height above given pose
  target_pose.position.z += height;
  geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;

  if (comparePoses(current_pose, target_pose))
  {
    ROS_INFO_STREAM("Poses same, skipping Pre Pick Pose");
  }
  else
  {
    executeCartesianTrajtoPose(target_pose, "Pre Pick Pose");
  }
  ROS_INFO("---------------------------");
  sleepSafeFor(robot_settle_time_);

  // Go down to reach and grasp the object
  target_pose.position.z -= height;
  executeCartesianTrajtoPose(target_pose, "Pick Pose");
  if (do_gripper)
    gripperClose(nh);
  sleepSafeFor(robot_settle_time_);
  //  addOrRemoveTestPiececollisionObjectWRTRobot(COMMAND_ADD);
  ROS_INFO("---------------------------");

  // Go back up
  target_pose.position.z += height;
  executeCartesianTrajtoPose(target_pose, "Post Pick Pose");
  ROS_INFO("---------------------------");
}

void MoveitCustomApi::
    placeAtPoseFromHeight(geometry_msgs::Pose target_pose,
                          double height,
                          ros::NodeHandle nh,
                          bool do_gripper)
/** Assuming  the provided @param{target_pose}is the pose of the place position,
 * the gripper first goes to the same XY lcoation, but at a height of @param{height} above it,
 * then moves down to place the object,
 * and then moves back up.
 */
{
  // Go to a set height above given pose
  target_pose.position.z += height;
  executeCartesianTrajtoPose(target_pose, "Pre Place Pose");
  ROS_INFO("---------------------------");
  sleepSafeFor(robot_settle_time_);
  // Go down and place the object
  target_pose.position.z -= height;
  executeCartesianTrajtoPose(target_pose, "Place Pose");
  ROS_INFO("---------------------------");
  if (do_gripper)
    gripperOpen(nh);
  sleepSafeFor(robot_settle_time_);
  //  addOrRemoveTestPiececollisionObjectWRTRobot(COMMAND_REMOVE);

  // Go back up
  target_pose.position.z += height;
  executeCartesianTrajtoPose(target_pose, "Post Place Pose");
  ROS_INFO("---------------------------");
}

moveit::planning_interface::MoveGroupInterface::Plan MoveitCustomApi::
      getPlanToPoseTarget(geometry_msgs::Pose target_pose,
                          int trials = 3,
                          std::string display_name = "target pose")
{
  namespace rvt = rviz_visual_tools;
  checkTrialsLimit(trials);
  bool plan_success = false;
  int trial = 0;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  while (trial++ < trials)
  {
    move_group->setPoseTarget(target_pose);
    plan_success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Visualising plan for: " << display_name);
    visual_tools->publishAxisLabeled(target_pose, display_name);
    visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools->trigger();
    ROS_INFO("Plan Attemp %d of %d : %s", trial, trials, plan_success ? "SUCCESS" : "FAILED");
    if (plan_success)
    {
      ROS_INFO_STREAM("Succeeded plan, continuing to execute");
      break;
    }
    else if (trial < max_trials)
    {
      ROS_WARN_STREAM("Planning failed, reattempting");
    }
    else
    {
      ROS_ERROR_STREAM("Planning failed, consider quitting");
    }
  }
  return my_plan;
}
