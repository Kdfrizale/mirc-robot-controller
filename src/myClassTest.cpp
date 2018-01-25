#include <myClassTest.h>

RoboticArm::RoboticArm(ros::NodeHandle &nh):nh_(nh){
  sub_leap_hand_ = nh_.subscribe("/handPoseTopic",1,&RoboticArm::updatePoseValues, this);
  group_ = new moveit::planning_interface::MoveGroupInterface("arm");
  gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");
  receivedNewPose_ = false;
  closedJointValues_ = gripper_group_->getNamedTargetValues("Close");
}

RoboticArm::~RoboticArm(){
    delete group_;
    delete gripper_group_;
}
bool RoboticArm::calculateFingerMove(){
  //std::vector<double> joints = gripper_group_->getCurrentJointValues();
  //double ratioClosed = getDistanceBetweenPoints(sensedPoseTip1_,sensedPoseTip2_)/10;
  double ratioClosed = 0.5;
  ROS_INFO("the ratio closed is: [%f]", ratioClosed);

  std::map<std::string, double> jointValueGoal = closedJointValues_;
  std::map<std::string, double>::iterator it = jointValueGoal.begin();
  while (it != jointValueGoal.end()){
    std::cout<<it->first<<" :: "<<it->second<<std::endl;
    it->second = it->second * ratioClosed;
    std::cout<<it->first<<" :: "<<it->second<<std::endl;
    it++;
  }
  gripper_group_->setJointValueTarget(jointValueGoal);
  return gripper_group_->plan(planGripper_);
}
bool RoboticArm::calculateMove(){
  //Calculate require joint values for fingers
  // std::vector<double> joints = gripper_group_->getCurrentJointValues();
  // //double ratioClosed = getDistanceBetweenPoints(sensedPoseTip1_,sensedPoseTip2_)/10;
  double ratioClosed = 0.5;
  ROS_INFO("the ratio closed is: [%f]", ratioClosed);

  std::map<std::string, double> jointValueGoal = closedJointValues_;
  std::map<std::string, double>::iterator it = jointValueGoal.begin();
  while (it != jointValueGoal.end()){
    it->second = it->second * ratioClosed;
    it++;
  }
  //joints = myMap.find("Close");
  //ROS_INFO("the joint value is [%f]", joints.at(0));
  //TODO change this hardcode value to dynamic function to match user's input
  //Need a way to find max limit on joint value when applied to both fingers
      //---Utilized the preset positions, open and closed to get the joint values
  //joints.at(0) = 2;//Left Finger
  //joints.at(2) = 0;//Right Finger

  group_->setPoseTarget(rotatePoseStamped(sensedPosePalm_));
  gripper_group_->setJointValueTarget(jointValueGoal);
  return (group_->plan(planArm_) && gripper_group_->plan(planGripper_));
  //return (group_->plan(planArm_));
}

bool RoboticArm::executeFingerMove(){
  return gripper_group_->execute(planGripper_);
}
bool RoboticArm::executeMove(){
  return (group_->execute(planArm_) && gripper_group_->execute(planGripper_));
  //return group_->execute(planArm_);
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
      // if(this->calculateFingerMove()){
      //   this->executeFingerMove();
      // }
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
