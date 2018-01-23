#include <myClassTest.h>

RoboticArm::RoboticArm(ros::NodeHandle &nh):nh_(nh){
  sub_leap_hand_ = nh_.subscribe("/handPoseTopic",1,&RoboticArm::updatePoseValues, this);
  group_ = new moveit::planning_interface::MoveGroupInterface("arm");
  gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");
  receivedNewPose_ = false;
}

RoboticArm::~RoboticArm(){
    delete group_;
    delete gripper_group_;
}

bool RoboticArm::calculateMove(){
  //Calculate require joint values for fingers
  std::vector<double> joints;
  joints = gripper_group_->getCurrentJointValues();
  joints.at(0) = 1.0;//TODO change this hardcode value to dynamic function to match user's input

  group_->setPoseTarget(rotatePoseStamped(sensedPosePalm_));
  gripper_group_->setJointValueTarget(joints);
  return (group_->plan(planArm_) && gripper_group_->plan(planGripper_));
}

bool RoboticArm::executeMove(){
  return (group_->execute(planArm_) && gripper_group_->execute(planGripper_));
}

//Sets the desired poseTargets to the received input poses
void RoboticArm::updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"Received Input. Now processing...");
  //set desiredPose to match the HandStampedPose
  sensedPosePalm_ = msg->posePalm;
  receivedNewPose_ = true;
}

//Begin executing callback functions for subscriptions
void RoboticArm::beginListening(){
  while (ros::ok()){
    ROS_INFO_THROTTLE(1,"Waiting for input...");
    ros::spinOnce();
    if(receivedNewPose_){
      receivedNewPose_ = false;
      if(this->calculateMove()){
        this->executeMove();
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
  myArm.beginListening();

  ros::spin();
  return 0;
}
