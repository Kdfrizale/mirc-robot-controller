#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <arm_mimic_capstone/HandStampedPose.h>
#include <utilities.h>

class RoboticArm {
public:
  RoboticArm(ros::NodeHandle &nh);
  ~RoboticArm();
  void beginListening();

private:
  ros::NodeHandle nh_;
  bool receivedNewPose_;
  std::map<std::string, double> closedJointValues_;
  std::map<std::string, double> openedJointValues_;

  geometry_msgs::PoseStamped sensedPoseTip2_;//Right Finger tip
  geometry_msgs::PoseStamped sensedPosePalm_;//wrist
  geometry_msgs::PoseStamped sensedPoseTip1_;//Left Finger tip

  moveit::planning_interface::MoveGroupInterface* group_;
  moveit::planning_interface::MoveGroupInterface* gripper_group_;
  moveit::planning_interface::MoveGroupInterface::Plan planArm_;
  moveit::planning_interface::MoveGroupInterface::Plan planGripper_;

  ros::Subscriber sub_leap_hand_;

  bool calculateMove();
  bool executeMove();
  void updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg);
};
