#include "ur_manipulation/moveit_custom_api.hpp"

void loggedButtonMash( MoveitCustomApi obj,
                       geometry_msgs::Pose target_pose,
                       double height,
                       int button_no,
                       EnduranceTestLogger logger,
                       std::string source
                       )
/** Assuming  the provided @param{target_pose}is the pose of the target object itself,
 * the end effector first goes to the same XY lcoation, but at a height of @param{height} above it,
 * then moves down to it & sends a socket log msg,
 * and then moves back up,
 * thereby pressing the button
 */
{
    // Go to a set height above given pose
    target_pose.position.z+=height;
    obj.executeCartesianTrajtoPose(target_pose,"Pre Mash Pose");
    ROS_INFO("---------------------------");
//    obj.sleepSafeFor(0.5);

    //Check if pre mash pose is reached
    ROS_INFO_STREAM("State "
                    << "Pre Mash - Button"+std::to_string(button_no)
                    << (obj.comparePoses(obj.move_group->getCurrentPose().pose, target_pose)?
                            " succesfully achieved":" failed"));
    ROS_INFO(" ");

    // Go down to press the button
    target_pose.position.z-=height;
    obj.executeCartesianTrajtoPose(target_pose,"Mash Pose");
//    obj.sleepSafeFor(0.5);

    //Check if mash pose is reached, and log attempt
    ROS_INFO_STREAM("State "
                    << "Mash - Button"+std::to_string(button_no)
                    << (obj.comparePoses(obj.move_group->getCurrentPose().pose, target_pose)?
                            " succesfully achieved":" failed"));
    ROS_INFO(" ");
    logger.setMessage(source+";"+logger.get_now_as_string()+";"+std::to_string(button_no));
    logger.send_data();

    // Go back up---------------------------------------------
    target_pose.position.z+=height;
    obj.executeCartesianTrajtoPose(target_pose,"Post Mash Pose");
//    obj.sleepSafeFor(0.5);

    //Check if post mash pose is reached
    ROS_INFO_STREAM("State "
                    << "Post Mash - Button"+std::to_string(button_no)
                    << (obj.comparePoses(obj.move_group->getCurrentPose().pose, target_pose)?
                            " succesfully achieved":" failed"));
    ROS_INFO(" ");
    ROS_INFO("---------------------------");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ret_button_masher");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Test server: https://github.com/ipa-kut/socket_learning/blob/master/3.multi_client/multi-client-server-threaded.py
  //Enable the print statement in the server code to print received messages to console
  EnduranceTestLogger logger1("127.0.0.1",65432);
  logger1.connect_to_server();

  //-------------Button Masher program-------------------

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Publisher pub_seq = nh.advertise<std_msgs::Header>("/ur_manipulation/sequence",1);
  ros::Publisher pub_fail = nh.advertise<std_msgs::Header>("/ur_manipulation/failure_counter",1);

  MoveitCustomApi ret_obj(argv[1]);
  std::string robot = argv[2];
  ret_obj.initialiseMoveit(nh);
  ret_obj.printBasicInfo();
  ret_obj.addCollissionObjects(); // Should no longer be needed if PRBT and UR5e both have this defined natively
  ROS_INFO("Moving to ready pose");
  ret_obj.moveToNamedTarget("ready");
  unsigned int seq = 0;

  ROS_INFO("Starting Button Masher");
  ROS_INFO("---------------------------");

  geometry_msgs::Pose target_pose1;

  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.012;
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(180),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
  target_pose1.orientation = quat_msg;

  geometry_msgs::Pose target_pose2 = target_pose1;
  target_pose2.position.x = 0.05;

  bool switcher=false;
  while(ros::ok())
  {
    ROS_INFO_STREAM("----------------------SEQ " << seq << "-------------------------------------");
    loggedButtonMash(ret_obj,
                     (switcher)?target_pose2:target_pose1,
                     0.03,
                     (switcher)?1:2,
                     logger1,
                     robot+"_ros"
                     );
    switcher = !switcher;
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    msg.seq = seq;
    pub_seq.publish(msg);
    msg.seq =ret_obj.failure_counter_;
    pub_fail.publish(msg);
    ROS_INFO_STREAM("----------------------SEQ " << seq++ << "-------------------------------------");
  }

  //-------------End Button Masher program-------------------
  return 0;
}
