#include <string>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <arm_mimic_capstone/HandStampedPose.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/SetFingersPositionGoal.h>
#include <kinova_msgs/JointAngles.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>

class RoboticArm {
public:
  RoboticArm(ros::NodeHandle &nh);
  ~RoboticArm();
  geometry_msgs::Pose rotatePose(geometry_msgs::PoseStamped &inputPose);//maybe change this to accept axis of rotations as parameter



private:
  ros::NodeHandle nh_;
  double yOffset_;

  geometry_msgs::PoseStamped sensedPoseTip2_;//Right Finger tip
  geometry_msgs::PoseStamped sensedPosePalm_;//wrist
  geometry_msgs::PoseStamped sensedPoseTip1_;//Left Finger tip
  geometry_msgs::Pose goalArmEEPose_;

  moveit::planning_interface::MoveGroupInterface* group_;
  moveit::planning_interface::MoveGroupInterface* gripper_group_;

  ros::Subscriber sub_leap_hand_;


  void beginListening();
  bool planAndMove();
  void updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg);


};
