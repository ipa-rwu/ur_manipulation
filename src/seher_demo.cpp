/// TODO:
/// [] Wrap moveToNamedPose in multiple trials as well
/// [] Try to merge plan and execute functions (and hence number of trial loops)
/// [] Incorporate gripper functionality
/// [] Try with moveit pick and place interface instead
///


#include "ur_manipulation/seher_demo.h"

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
  box_pose.position.z = -0.01;

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

void SeherDemo::checkTrialsLimit(int trials)
{
  if (trials > max_trials)
  {
    ROS_WARN_STREAM("Selected number of trials for execution : " << trials << ", is more than allowed max of : " <<max_trials );
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
  if (user_prompts)
  {
    visual_tools->prompt("Moving to starting pose up, press Next to continue");
  }

  move_group->setNamedTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group->plan(my_plan);
  move_group->move();
}

moveit::planning_interface::MoveGroupInterface::Plan SeherDemo::getPlanToPoseTarget(geometry_msgs::Pose target_pose, int trials=3, std::string display_name="target pose")
{
  namespace rvt = rviz_visual_tools;
  checkTrialsLimit(trials);
  bool plan_success = false;
  int trial = 0;

  if(user_prompts)
  {
    std::string message = "Attempting planning for " + display_name + ", press Next to continue";
    visual_tools->prompt(message);
  }

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
    else if (trials < max_trials)
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
  seher_obj.moveToNamedTarget("up");

  //Pick

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.25;
  target_pose1.position.y = 0.25;
  target_pose1.position.z = 0.25;
  target_pose1.orientation.x = 0;
  target_pose1.orientation.y = 0;
  target_pose1.orientation.z = 0;
  target_pose1.orientation.w = 1.0;

  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,3,"pre pick pose"));

  target_pose1.position.z -= 0.2;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,3,"pick pose"));

  target_pose1.position.z += 0.2;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,3,"post pick pose"));


  // Place

  target_pose1.position.x = -0.25;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,3,"pre place pose"));

  target_pose1.position.z -= 0.2;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,3,"place pose"));

  target_pose1.position.z += 0.2;
  seher_obj.moveGroupExecutePlan(seher_obj.getPlanToPoseTarget(target_pose1,3,"post place pose"));



  return 0;
}
