/// TODO:
/// [x] Wrap moveToNamedPose in multiple trials as well
/// [x] Try to merge plan and execute functions (and hence number of trial loops)
/// [x] Incorporate gripper functionality
/// [] Try with moveit pick and place interface instead - Requires implementation of prismatic joints in URDF and controllers for the same
/// [] Test the test piece collission object add/remove functionality more, including trajectories


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

bool SeherDemo::moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{

  if(user_prompts)
  {
    std::string message = "Planning completed, press Next to execute";
    visual_tools->prompt(message);
  }

  return move_group->execute(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
}


void SeherDemo::addCollissionObjects()
{
  namespace rvt = rviz_visual_tools;
  moveit_msgs::CollisionObject object;
  object.header.frame_id = move_group->getPlanningFrame();

  // The id of the object is used to identify it.
  object.id = "Floor";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.97;
  primitive.dimensions[1] = 1.47;
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0.460; //  Base is ofset by -(0.1470/2-.275)
  box_pose.position.z = -0.001;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(box_pose);
  object.operation = object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(object);

  // The id of the object is used to identify it.
  object.id = "Left Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = 1.47;
  primitive.dimensions[2] = 1.1;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.485;
  box_pose.position.y = 0.460;
  box_pose.position.z = 0.55;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(box_pose);
  object.operation = object.ADD;

  collision_objects.push_back(object);

  // The id of the object is used to identify it.
  object.id = "Right Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = 1.47;
  primitive.dimensions[2] = 1.1;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.485;
  box_pose.position.y = 0.460;
  box_pose.position.z = 0.55;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(box_pose);
  object.operation = object.ADD;

  collision_objects.push_back(object);

  // The id of the object is used to identify it.
  object.id = "Back Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0.97;
  primitive.dimensions[1] = 0;
  primitive.dimensions[2] = 1.1;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = -0.275;
  box_pose.position.z = 0.55;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(box_pose);
  object.operation = object.ADD;

  collision_objects.push_back(object);




  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Adding collission objects into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools->publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
}

void SeherDemo::addOrRemoveTestPieceCollissionObject(std::string command)
{
  if(command!= COMMAND_ADD && command!= COMMAND_REMOVE)
  {
    ROS_ERROR_STREAM("Unknown test piece collission object manipulation command : " << command << ". Expecting " << COMMAND_ADD << " or " << COMMAND_REMOVE);
    return;
  }

  // The id of the object is used to identify it.
  moveit_msgs::CollisionObject object1;
  object1.id = "Test Cube";
  object1.header.frame_id=move_group->getEndEffectorLink();

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive1;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 0.03;
  primitive1.dimensions[1] = 0.03;
  primitive1.dimensions[2] = 0.03;

  // Define a pose for the box (specified relative to frame_id
  geometry_msgs::Pose box_pose1;
  box_pose1.orientation.w = 1.0;
  box_pose1.position.x = 0.0;
  box_pose1.position.y = 0.0;
  box_pose1.position.z = 0.015;

  object1.primitives.push_back(primitive1);
  object1.primitive_poses.push_back(box_pose1);
  object1.operation = (command== COMMAND_ADD) ? object1.ADD : object1.REMOVE;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(object1);


  ROS_INFO_STREAM( command << " test piece collission object.");
  planning_scene_interface.addCollisionObjects(collision_objects);


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
  io_msg.request.pin = static_cast<int8_t>(0);
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
  io_msg.request.pin = static_cast<int8_t>(1);
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

void SeherDemo::initialiseMoveit()
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

  SeherDemo seher_obj;
  seher_obj.initialiseMoveit();
  seher_obj.printBasicInfo();
  seher_obj.addCollissionObjects();
  seher_obj.moveToNamedTarget("home");

  //Pick

  geometry_msgs::Pose target_pose1;

  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.05;
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(180),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
  target_pose1.orientation = quat_msg;

  int trials = 5;

  seher_obj.gripperOpen(nh);
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,trials,"pre pick pose"));

  target_pose1.position.z -= 0.03;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,trials,"pick pose"));
  seher_obj.gripperClose(nh);
  seher_obj.addOrRemoveTestPieceCollissionObject(seher_obj.COMMAND_ADD);

  target_pose1.position.z += 0.03;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,trials,"post pick pose"));


  // Place

  target_pose1.position.x = -0.25;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,trials,"pre place pose"));

  target_pose1.position.z -= 0.03;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,trials,"place pose"));
  seher_obj.gripperOpen(nh);
  seher_obj.addOrRemoveTestPieceCollissionObject(seher_obj.COMMAND_REMOVE);

  target_pose1.position.z += 0.03;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,trials,"post place pose"));

  return 0;
}
