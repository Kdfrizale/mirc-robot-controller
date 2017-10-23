#include "ros/ros.h"
#include <arm_mimic_capstone/HandStampedPose.h>
#include <geometry_msgs/PoseStamped.h>
//#include "sensor_msgs/JointState.h"
#include <cstdlib>
#include <iostream>
#include <stdio.h>

void ouch(int sig){
  ROS_INFO("I caught a signal");
}

int main(int argc, char **argv)
{
  ros::init(argc,argv, "myDummyPub");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<arm_mimic_capstone::HandStampedPose>("/handPoseTopic", 10);
  ros::Rate loop_rate(10);
  int count =0;

  geometry_msgs::PoseStamped sensedPoseTip2;//Right Finger tip---Demo data
  sensedPoseTip2.header.frame_id = "m1n6s200_link_base";
  sensedPoseTip2.pose.position.x = 0.247616;
  sensedPoseTip2.pose.position.y = 0.180879;
  sensedPoseTip2.pose.position.z = 0.653225;
  sensedPoseTip2.pose.orientation.x = 0.704808;
  sensedPoseTip2.pose.orientation.y = 0.0573433;
  sensedPoseTip2.pose.orientation.z = 0.704747;
  sensedPoseTip2.pose.orientation.w = 0.0573497;

  geometry_msgs::PoseStamped sensedPoseLink6;//wrist --- Demo Data
  sensedPoseLink6.header.frame_id = "m1n6s200_link_base";
  sensedPoseLink6.pose.position.x = 0.155941;
  sensedPoseLink6.pose.position.y = 0.166744;
  sensedPoseLink6.pose.position.z = 0.854475;
  sensedPoseLink6.pose.orientation.x = 0.000000;
  sensedPoseLink6.pose.orientation.y = 0.000000;
  sensedPoseLink6.pose.orientation.z = 0.000000;
  sensedPoseLink6.pose.orientation.w = 1;

  geometry_msgs::PoseStamped sensedPoseTip1;//Left Finger tip---Demo data
  sensedPoseTip1.header.frame_id = "m1n6s200_link_base";
  sensedPoseTip1.pose.position.x = 0.247615;
  sensedPoseTip1.pose.position.y = 0.103661;
  sensedPoseTip1.pose.position.z = 0.453269;
  sensedPoseTip1.pose.orientation.x = 0.0181078;
  sensedPoseTip1.pose.orientation.y = -0.706845;
  sensedPoseTip1.pose.orientation.z = -0.0181207;
  sensedPoseTip1.pose.orientation.w = 0.706905;

  while (ros::ok()){
    sensedPoseTip2.pose.position.y += 0.001;
    sensedPoseLink6.pose.position.y += 0.001;
    sensedPoseTip1.pose.position.y += 0.001;
    sensedPoseTip2.pose.position.x += 0.001;
    sensedPoseLink6.pose.position.x += 0.001;
    sensedPoseTip1.pose.position.x += 0.001;

    arm_mimic_capstone::HandStampedPose msg;
    msg.poseTip2 = sensedPoseTip2;
    msg.poseLink6 = sensedPoseLink6;
    msg.poseTip1 = sensedPoseTip1;

    ROS_INFO("tip1 X: [%f]", sensedPoseTip1.pose.position.x);
    ROS_INFO("tip2 X: [%f]", sensedPoseTip2.pose.position.x);
    ROS_INFO("palm X: [%f]", sensedPoseLink6.pose.position.x);

    ROS_INFO("tip1 orientation X: [%f]", sensedPoseTip1.pose.orientation.x);
    ROS_INFO("tip2 orientation X: [%f]", sensedPoseTip2.pose.orientation.x);
    ROS_INFO("palm orientation X: [%f]", sensedPoseLink6.pose.orientation.x);


  //  sensor_msgs::JointState msg;
  //  msg.name.push_back("j2n6s300_joint_1");
  //  msg.position.push_back(rand() %10);
  //  msg.name.push_back("j2n6s300_joint_2");
  //  msg.position.push_back(rand() %10);
    //msg.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_4', 'j2n6s300_joint_3'];
    //msg.position = [2, 3.59, 7.01, 3.07];

    chatter_pub.publish(msg);
    std::cout << "Press Enter to Continue" ;
    getchar();
    //ROS_INFO("%s",msg.data.c_str());
    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  }
  return 0;
}
