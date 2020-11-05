#include "ur_manipulation/moveit_custom_api.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seher_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Logger test. Remove when no longer necessary.
  //Test server: https://github.com/ipa-kut/socket_learning/blob/master/3.multi_client/multi-client-server-threaded.py
  //Enable the print statement in the server code to print received messages to console
  EnduranceTestLogger logger1("127.0.0.1",65432);;
  logger1.connect_to_server();
  for (int i=0; i<10; i++)
  {
      logger1.send_data_timestamped("Hello", true, 0.1 );
  }

  ///TODO: Create a button masher program.
  /// 1. Send meaningful log messages
  /// 2. Perhaps logging should be added inside the pick/placeAtPoseFromHeight functions?

  //-------------Pick and place program-------------------

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Publisher pub_seq = nh.advertise<std_msgs::Header>("/ur_manipulation/sequence",1);
  ros::Publisher pub_fail = nh.advertise<std_msgs::Header>("/ur_manipulation/failure_counter",1);

  MoveitCustomApi seher_obj(argv[1]);
  seher_obj.initialiseMoveit(nh);
  seher_obj.printBasicInfo();
  ROS_INFO("---------------------------");
  seher_obj.addCollissionObjects();
  ROS_INFO("Moving to home pose");
  seher_obj.moveToNamedTarget("home");
  unsigned int seq = 0;

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
    ROS_INFO_STREAM("----------------------SEQ " << seq << "-------------------------------------");
    seher_obj.pickAtPoseFromHeight((switcher)?target_pose1:target_pose2, 0.03, nh, false);
    seher_obj.placeAtPoseFromHeight((switcher)?target_pose2:target_pose1, 0.03, nh, false);
    switcher = !switcher;
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    msg.seq = seq;
    pub_seq.publish(msg);
    msg.seq =seher_obj.failure_counter_;
    pub_fail.publish(msg);
    ROS_INFO_STREAM("----------------------SEQ " << seq++ << "-------------------------------------");
  }

  //-------------Pick and place program-------------------


  return 0;
}
