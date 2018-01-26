#include <ros/ros.h>
#include <leap_controller_capstone/HandPoseStamped.h>
#include <utilities.h>

class ControlledRobot {
public:
  ControlledRobot(ros::NodeHandle &nh);
  virtual ~ControlledRobot()=0;
  void beginListening();

protected:
  ros::NodeHandle nh_;
  bool receivedNewPose_;

  geometry_msgs::PoseStamped sensedPoseTip2_;//Right Finger tip
  geometry_msgs::PoseStamped sensedPosePalm_;//wrist
  geometry_msgs::PoseStamped sensedPoseTip1_;//Left Finger tip

  ros::Subscriber sub_leap_hand_;

  virtual bool calculateMove();
  virtual bool executeMove();
  void updatePoseValues(const leap_controller_capstone::HandPoseStamped::ConstPtr& msg);
};
