#include <myClassTest.h>

RoboticArm::RoboticArm(ros::NodeHandle &nh):nh_(nh){
  ros::Subscriber sub = nh_.subscribe("/handPoseTopic",1,&RoboticArm::updatePoseValues, this);//TODO change this to member sub
  group_ = new moveit::planning_interface::MoveGroupInterface("arm");

  beginListening();
}

RoboticArm::~RoboticArm(){
    delete group_;
    delete gripper_group_;
}

bool RoboticArm::planAndMove(){
  group_->setPoseTarget(rotatePoseStamped(this->sensedPosePalm_));
  bool validPlanArm = group_->plan(this->planArm_);

  if(validPlanArm){
    return group_->execute(this->planArm_);
  }
  return validPlanArm;
}

//Sets the desired poseTargets to the received input poses
void RoboticArm::updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"Received Input. Now processing...");
  //set desiredPose to match the HandStampedPose
  this->sensedPosePalm_ = msg->posePalm;
  this->planAndMove();
}

void RoboticArm::beginListening(){
  while (ros::ok()){
    ROS_INFO_THROTTLE(1,"Waiting for input...");
    ros::spinOnce();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "myClassTestCode");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("RoboticArm Node has started");

  ros::WallDuration(10.0).sleep();//allow for Rviz/MoveIt to start before continuing
  RoboticArm myArm = RoboticArm(node);

  ros::spin();
  return 0;
}
