#include <string>

class RoboticArm {
public:
  RoboticArm(ros::NodeHandle &nh);
  bool planAndMove(geometry_msgs::Pose targetPose);
  ~RoboticArm();


private:
  double yOffset;
  std::string armPlanningGroupName;
  void updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg);
  void beginListening();
  void updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg);

}
bool RoboticArm::planAndMove(geometry_msgs::Pose targetPose){
  ROS_INFO_THROTTLE(1, "here is were the move group would be accessed");
  return true;
}

//Sets the desired poseTargets to the received input poses
void RoboticArm::updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"I received a message.. now processing...");
  //set desiredPose to match the HandStampedPose
  RoboticArm::planAndMove(desiredPose);
}

RoboticArm::RoboticArm(ros::NodeHandle &nh):nh_(nh){
  ros::Subscriber sub = nh_.subscribe("/handPoseTopic",1,updatePoseValues);
  geometry_msgs::PoseStamped poseTip2;//Right Finger tip
  geometry_msgs::PoseStamped posePalm;//wrist
  geometry_msgs::PoseStamped poseTip1;//Left Finger tip

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  beginListening();
  }

void RoboticArm::beginListening(){
  while (ros::ok()){
    ros::spinOnce();
  }
}
