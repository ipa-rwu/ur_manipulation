#include "ur_manipulation/moveit_custom_api.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seher_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Publisher pub_seq = nh.advertise<std_msgs::Header>("/ur_manipulation/sequence", 1);
  ros::Publisher pub_fail = nh.advertise<std_msgs::Header>("/ur_manipulation/failure_counter", 1);

  MoveitCustomApi seher_obj(argv[1]);
  seher_obj.initialiseMoveit(nh);
  seher_obj.printBasicInfo();
  seher_obj.addCollissionObjects();
  ROS_INFO("Moving to ready pose");
  seher_obj.moveToNamedTarget("ready");
  int seq = 0;

  //  ROS_INFO("Starting PnP");
  //  ROS_INFO("---------------------------");

  //  geometry_msgs::Pose target_pose1;

  //  target_pose1.position.x = 0.3;
  //  target_pose1.position.y = 0.4;
  //  target_pose1.position.z = 0.012;
  //  geometry_msgs::Quaternion quat_msg;
  //  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(180),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
  //  target_pose1.orientation = quat_msg;

  //  geometry_msgs::Pose target_pose2 = target_pose1;
  //  target_pose1.position.x = 0.05;

  //  bool switcher=false;
  //  while(ros::ok())
  //  {
  //    ROS_INFO_STREAM("----------------------SEQ " << seq << "-------------------------------------");
  //    seher_obj.pickAtPoseFromHeight((switcher)?target_pose1:target_pose2, 0.03, nh);
  //    seher_obj.placeAtPoseFromHeight((switcher)?target_pose2:target_pose1, 0.03, nh);
  //    switcher = !switcher;
  //    std_msgs::Header msg;
  //    msg.stamp = ros::Time::now();
  //    msg.seq = seq;
  //    pub_seq.publish(msg);
  //    msg.seq =seher_obj.failure_counter_;
  //    pub_fail.publish(msg);
  //    ROS_INFO_STREAM("----------------------SEQ " << seq++ << "-------------------------------------");
  //  }

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose wp_pose1 = seher_obj.move_group->getCurrentPose().pose;
  waypoints.push_back(wp_pose1);

  wp_pose1.position.x += 0.1;
  waypoints.push_back(wp_pose1);
  wp_pose1.position.x += 0.1;
  waypoints.push_back(wp_pose1);

  wp_pose1.position.z -= 0.1;
  waypoints.push_back(wp_pose1);
  wp_pose1.position.z -= 0.1;
  waypoints.push_back(wp_pose1);

  wp_pose1.position.x -= 0.1;
  waypoints.push_back(wp_pose1);
  wp_pose1.position.x -= 0.1;
  waypoints.push_back(wp_pose1);

  wp_pose1.position.z += 0.1;
  waypoints.push_back(wp_pose1);
  wp_pose1.position.z += 0.1;
  waypoints.push_back(wp_pose1);

  while (ros::ok())
  {
    ROS_INFO_STREAM("----------------------SEQ " << seq << "-------------------------------------");
    seher_obj.executeCartesianTrajForWaypoints(waypoints, 0.1);
    seher_obj.sleepSafeFor(0.5);
    ROS_INFO_STREAM("----------------------SEQ " << seq++ << "-------------------------------------");
  }

  std::cout << "Hello world " << std::endl;

  return 0;
}
