/// TODO:
/// [x] Wrap moveToNamedPose in multiple trials as well
/// [x] Try to merge plan and execute functions (and hence number of trial loops)
/// [x] Incorporate gripper functionality
/// [x] Change up and down motion from target pose movement to cartesian pose move to ensure smooth trajectory
/// [] Try with moveit pick and place interface instead - Requires implementation of prismatic joints in URDF and controllers for the same
/// [] Test the test piece collission object add/remove functionality more, including trajectories - Test object currently left nehind when the place operation is done. Fix this.


#include "ur_manipulation/seher_demo.h"
#include "tf/transform_datatypes.h"
#include <angles/angles.h>


void getRPYFromQuaternionMSG(geometry_msgs::Quaternion orientation, double& roll,double& pitch, double& yaw)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation,quat);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch,yaw);
}


SeherDemo::SeherDemo()  :
  max_trials(5),
  user_prompts(true)
{

}

SeherDemo::SeherDemo(int max_trials, std::string user_prompts)
{
  this->max_trials = max_trials;
  this->user_prompts =  (user_prompts=="True"|| user_prompts=="true")? true : false;
  ROS_INFO_STREAM("User prompts : " << this->user_prompts << " | Max planning trials : " << max_trials);
}

SeherDemo::~SeherDemo() {}

void SeherDemo::printBasicInfo()
{
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;

}

bool SeherDemo::comparePoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double delta_posistion, double delta_orientation)
{

  if (  abs(pose1.position.x-pose2.position.x ) <= delta_posistion
        && abs(pose1.position.y-pose2.position.y ) <= delta_posistion
        && abs(pose1.position.z-pose2.position.z ) <= delta_posistion
        && abs(pose1.orientation.x - pose2.orientation.x) <= delta_orientation
        && abs(pose1.orientation.y - pose2.orientation.y) <= delta_orientation
        && abs(pose1.orientation.z - pose2.orientation.z) <= delta_orientation
        && abs(pose1.orientation.w - pose2.orientation.w) <= delta_orientation
     )
  {
    return true;
  }
  else
  {
    return false;
  }
}

moveit::planning_interface::MoveGroupInterface::Plan SeherDemo::getCartesianPathPlanToPose(geometry_msgs::Pose target_pose, std::string display_label, double eef_step, double jump_threshold)
{
  namespace rvt = rviz_visual_tools;
  move_group->setStartStateToCurrentState();
  std::vector<geometry_msgs::Pose> waypoints;
//  waypoints.push_back(move_group->getCurrentPose().pose);
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  double fraction=0.0;
  while(fraction<0.5)
  {
    fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  }

  ROS_INFO_STREAM("Visualizing Cartesian Path plan to "  <<  display_label <<" (" << fraction*100  << "% acheived)");


  std::vector<ros::Duration> times_from_start;
    int iter=0;
    for(auto x: trajectory.joint_trajectory.points)
    {
  //    ROS_INFO_STREAM("Iter : "  << iter++ << " point_time_from_start: " << x.time_from_start);
      times_from_start.push_back(x.time_from_start);
    }

    // Adjust starting from point 2 i.e. index 1
    for(int i=1; i< times_from_start.size();i++)
    {

      if(times_from_start[i]==ros::Duration(0) && i<times_from_start.size())
      {
        ros::Duration prev = times_from_start[i];
        times_from_start[i] = ros::Duration((times_from_start[i-1].toSec()+times_from_start[i+1].toSec())/2.0);
        ROS_INFO_STREAM("Recomputing point " << i << " from " << prev <<  " to: " << times_from_start[i-1] << " + " << times_from_start[i+1] << " = " <<times_from_start[i]);
      }
    }

    for(int i=0; i< times_from_start.size(); i++)
    {
      trajectory.joint_trajectory.points[i].time_from_start = times_from_start[i];
  //    ROS_INFO_STREAM("Recomputed time point " << i << " : " << trajectory.joint_trajectory.points[i].time_from_start );
    }



  // Visualize the plan in RViz
  visual_tools->deleteAllMarkers();
  visual_tools->publishText(text_pose, "Cartesian path", rvt::WHITE, rvt::XLARGE);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools->trigger();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  return my_plan;

}

void SeherDemo::sleepSafeFor(double duration)
{
  ros::Time start = ros::Time::now();
  while(ros::Time::now() - start <= ros::Duration(duration))
  {
    ros::spinOnce();
  }

}


bool SeherDemo::moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{

  if(user_prompts)
  {
    std::string message = "Planning completed, press Next to execute";
    visual_tools->prompt(message);
  }

  move_group->setStartStateToCurrentState();
  return move_group->execute(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

void SeherDemo::addCollissionObjects()
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
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;  // Not perfectly symmetrical.
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_; // Base is ofset by (0.1470/2-.275)
  box_pose.position.z = -0.01; //Push it slightly down to avoid collission with base plate.

  // Since we are attaching the object to the robot base
  // we want the collision checker to ignore collisions between the object and the robot base
  object.touch_links = std::vector<std::string>{ "base_link"};
  moveit_msgs::PlanningScene planning_scene;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);


  // The id of the object is used to identify it.
  object.object.id = "Cieling";

  // Define a box to add to the world.
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;  // Not perfectly symmetrical.
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_; // Base is ofset by (0.1470/2-.275)
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id  = "Left Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = BASE_OFFSET_FROM_RIGHT_WALL_;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Right Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -BASE_OFFSET_FROM_LEFT_WALL_;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Back Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = 0;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;
  box_pose.position.y = -BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);


  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  ROS_INFO_NAMED("seher_demo", "Adding collission objects into the world");
}

void SeherDemo::addOrRemoveTestPieceCollissionObjectWRTRobot(std::string command)
{
  if(command!= COMMAND_ADD && command!= COMMAND_REMOVE)
  {
    ROS_ERROR_STREAM("Unknown test piece collission object manipulation command : " << command << ". Expecting " << COMMAND_ADD << " or " << COMMAND_REMOVE);
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
  box_pose1.position.z = 0.01;  //Object Z dimension/2

  attached_object.object.primitives.push_back(primitive1);
  attached_object.object.primitive_poses.push_back(box_pose1);
  attached_object.object.operation = (command== COMMAND_ADD) ? attached_object.object.ADD : attached_object.object.REMOVE;

  attached_object.touch_links = std::vector<std::string>{ move_group->getEndEffectorLink(), "egp50_pincer_link" };

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.robot_state.is_diff = true;
  planning_scene.is_diff = true;
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);
}

void SeherDemo::checkTrialsLimit(int trials)
{
  if (trials > max_trials)
  {
    ROS_WARN_STREAM("Selected number of trials for execution : " << trials << ", is more than allowed max of : " <<max_trials );
  }
}

bool SeherDemo::gripperOpen(ros::NodeHandle nh)
{
  ur_msgs::SetIO io_msg;
  io_msg.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);
  io_msg.request.pin = static_cast<int8_t>(1);  //Pin 1 is open
  io_msg.request.state = 1;
  ros::ServiceClient client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  if(client.call(io_msg))
  {
    ROS_INFO_STREAM("Open gripper initialise : " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
    sleep(1);
    io_msg.request.state = 0;
    if(client.call(io_msg))
    {
      ROS_INFO_STREAM("Open gripper conclude : " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
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

bool SeherDemo::gripperClose(ros::NodeHandle nh)
{
  ur_msgs::SetIO io_msg;
  io_msg.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);
  io_msg.request.pin = static_cast<int8_t>(0);    //Pin 0 is close
  io_msg.request.state = 1;
  ros::ServiceClient client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  if(client.call(io_msg))
  {
    ROS_INFO_STREAM("Close gripper initialise :  " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
    sleep(1);
    io_msg.request.state = 0;
    if(client.call(io_msg))
    {
      ROS_INFO_STREAM("Close gripper conclude :  " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
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

void SeherDemo::initialiseMoveit(ros::NodeHandle nh)
{
  namespace rvt = rviz_visual_tools;
  move_group = new moveit::planning_interface::MoveGroupInterface(GROUP_MANIP);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(GROUP_MANIP);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();
  text_pose.translation().z() = 1.75;
  visual_tools->publishText(text_pose, "Seher Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

}

void SeherDemo::moveToNamedTarget(std::string target)
{

  move_group->setNamedTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  int trials = 0;
  while (trials++ < max_trials)
  {
    if(move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
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

void SeherDemo::pickAtPoseFromHeight(geometry_msgs::Pose target_pose, double height, ros::NodeHandle nh)
/** Assuming  the provided @param{target_pose}is the pose of the target object itself,
 * the gripper first goes to the same XY lcoation, but at a height of @param{height} above it,
 * then moves down to grasp it,
 * and then moves back up.
 */
{
  // Make sure gripper is open
  gripperOpen(nh);

  // Go to a set height above given pose
  target_pose.position.z+=height;
  geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;

  if( comparePoses(current_pose, target_pose)  )
  {
    ROS_INFO_STREAM("Poses same, skipping Pre Pick Pose");
  }
  else
  {
//    moveGroupExecutePlan(getPlanToPoseTarget(target_pose, max_trials, "Pre Pick Pose"));
    moveGroupExecutePlan(getCartesianPathPlanToPose(target_pose, "Pre Pick Pose"));
  }
  ROS_INFO("---------------------------");
  sleepSafeFor(0.5);

  // Go down to reach and grasp the object
  target_pose.position.z-=height;
  moveGroupExecutePlan(getCartesianPathPlanToPose(target_pose, "Pick Pose"));
  gripperClose(nh);
  sleepSafeFor(0.5);
//  addOrRemoveTestPieceCollissionObjectWRTRobot(COMMAND_ADD);
  ROS_INFO("---------------------------");

  // Go back up
  target_pose.position.z+=height;
  moveGroupExecutePlan(getCartesianPathPlanToPose(target_pose, "Post Pick Pose"));
  ROS_INFO("---------------------------");
}

void SeherDemo::placeAtPoseFromHeight(geometry_msgs::Pose target_pose, double height, ros::NodeHandle nh)
/** Assuming  the provided @param{target_pose}is the pose of the place position,
 * the gripper first goes to the same XY lcoation, but at a height of @param{height} above it,
 * then moves down to place the object,
 * and then moves back up.
 */
{
  // Go to a set height above given pose
  target_pose.position.z+=height;
//  moveGroupExecutePlan(getPlanToPoseTarget(target_pose, max_trials, "Pre Place Pose"));
  moveGroupExecutePlan(getCartesianPathPlanToPose(target_pose, "Pre Place Pose"));
  ROS_INFO("---------------------------");
  sleepSafeFor(0.5);
  // Go down and place the object
  target_pose.position.z-=height;
  moveGroupExecutePlan(getCartesianPathPlanToPose(target_pose, "Place Pose"));
  ROS_INFO("---------------------------");
  gripperOpen(nh);
  sleepSafeFor(0.5);
//  addOrRemoveTestPieceCollissionObjectWRTRobot(COMMAND_REMOVE);

  // Go back up
  target_pose.position.z+=height;
  moveGroupExecutePlan(getCartesianPathPlanToPose(target_pose, "Post Place Pose"));
  ROS_INFO("---------------------------");
}

moveit::planning_interface::MoveGroupInterface::Plan SeherDemo::getPlanToPoseTarget(geometry_msgs::Pose target_pose, int trials=3, std::string display_name="target pose")
{
  namespace rvt = rviz_visual_tools;
  checkTrialsLimit(trials);
  bool plan_success = false;
  int trial = 0;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  while(trial++ < trials)
  {
    move_group->setPoseTarget(target_pose);
    plan_success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_STREAM("Visualising plan for: " << display_name);
    visual_tools->publishAxisLabeled(target_pose, display_name);
    visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools->trigger();
    ROS_INFO("Plan Attemp %d of %d : %s", trial, trials, plan_success? "SUCCESS" : "FAILED" );
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "seher_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  SeherDemo seher_obj(atoi(argv[2]),argv[1]);
  seher_obj.initialiseMoveit(nh);
  seher_obj.printBasicInfo();
  ROS_INFO("---------------------------");
  seher_obj.addCollissionObjects();
  ROS_INFO("Moving to home pose");
  seher_obj.moveToNamedTarget("home");

  ROS_INFO("Starting PnP");
  ROS_INFO("---------------------------");

  geometry_msgs::Pose target_pose1;

  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.012;
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(180),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
  target_pose1.orientation = quat_msg;

  geometry_msgs::Pose target_pose2 = target_pose1;
  target_pose1.position.x = 0.05;


  bool switcher=false;
  while(ros::ok())
  {
    seher_obj.pickAtPoseFromHeight((switcher)?target_pose1:target_pose2, 0.03, nh);
    seher_obj.sleepSafeFor(0.5);
    seher_obj.placeAtPoseFromHeight((switcher)?target_pose2:target_pose1, 0.03, nh);
    switcher = !switcher;
    seher_obj.sleepSafeFor(0.5);
  }

  return 0;
}
