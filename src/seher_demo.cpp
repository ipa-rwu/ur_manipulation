#include "ur_manipulation/seher_demo.h"

#include <geometry_msgs/PoseStamped.h>


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

}

void SeherDemo::initialiseMoveit()
{
  move_group = new moveit::planning_interface::MoveGroupInterface(GROUP_MANIP);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(GROUP_MANIP);

  namespace rvt = rviz_visual_tools;
  visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();
  text_pose.translation().z() = 1.75;
  visual_tools->publishText(text_pose, "Seher Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();

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

  return 0;
}
