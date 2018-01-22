#include <myClassTest.h>


RoboticArm::RoboticArm(ros::NodeHandle &nh):nh_(nh){
  ros::Subscriber sub = nh_.subscribe("/handPoseTopic",1,&RoboticArm::updatePoseValues, this);
  group_ = new moveit::planning_interface::MoveGroupInterface("arm");

  beginListening();
}

RoboticArm::~RoboticArm(){
    delete group_;
    delete gripper_group_;
}

bool RoboticArm::planAndMove(){
  //ROS_INFO("The received Pose Position X is: [%f]", this->sensedPosePalm_.pose.position.x);
  group_->setPoseTarget(rotatePoseStamped(this->sensedPosePalm_));
  ROS_INFO("the altered orientation is: [%f]", this->sensedPosePalm_.pose.orientation.x);
  ROS_INFO("the altered orientation is: [%f]", rotatePoseStamped(this->sensedPosePalm_).pose.orientation.x);
  bool validPlanArm = group_->plan(this->planArm_);

  if(validPlanArm){
    return group_->execute(this->planArm_);
  }
  return validPlanArm;
}

//Sets the desired poseTargets to the received input poses
void RoboticArm::updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"I received a message.. now processing...");
  //set desiredPose to match the HandStampedPose
  this->sensedPosePalm_ = msg->posePalm;
  this->planAndMove();
}

void RoboticArm::beginListening(){
  while (ros::ok()){
    ROS_INFO_THROTTLE(1,"Listening...");
    ros::spinOnce();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "myClassTestCode");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO_THROTTLE(1,"Beginning...");
  RoboticArm myArm = RoboticArm(node);


  ros::spin();
  return 0;
}
