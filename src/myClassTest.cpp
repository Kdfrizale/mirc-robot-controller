#include <myClassTest.h>

RoboticArm::RoboticArm(ros::NodeHandle &nh):nh_(nh){
  sub_leap_hand_ = nh_.subscribe("/handPoseTopic",1,&RoboticArm::updatePoseValues, this);
  group_ = new moveit::planning_interface::MoveGroupInterface("arm");
  receivedNewPose_ = false;

  beginListening();
}

RoboticArm::~RoboticArm(){
    delete group_;
    delete gripper_group_;
}

bool RoboticArm::calculatePath(){
  group_->setPoseTarget(rotatePoseStamped(sensedPosePalm_));
  return group_->plan(planArm_);
}

bool RoboticArm::executePath(){
  return group_->execute(planArm_);
}

//Sets the desired poseTargets to the received input poses
void RoboticArm::updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"Received Input. Now processing...");
  //set desiredPose to match the HandStampedPose
  sensedPosePalm_ = msg->posePalm;
  receivedNewPose_ = true;
}

void RoboticArm::beginListening(){
  while (ros::ok()){
    ROS_INFO_THROTTLE(1,"Waiting for input...");
    ros::spinOnce();
    if(receivedNewPose_){
      receivedNewPose_ = false;
      if(this->calculatePath()){
        this->executePath();
      }
    }
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
