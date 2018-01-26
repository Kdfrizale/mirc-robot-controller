#include <myClassTest.h>

RoboticArm::RoboticArm(ros::NodeHandle &nh):nh_(nh){
  std::string input_pose_topic_name;
  nh_.getParam("input_pose_topic",input_pose_topic_name);
  sub_leap_hand_ = nh_.subscribe(input_pose_topic_name,1,&RoboticArm::updatePoseValues, this);

  group_ = new moveit::planning_interface::MoveGroupInterface("arm");
  gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");
  closedJointValues_ = gripper_group_->getNamedTargetValues("Close");
  openedJointValues_ = gripper_group_->getNamedTargetValues("Open");
  receivedNewPose_ = false;
}

RoboticArm::~RoboticArm(){
    delete group_;
    delete gripper_group_;
}

bool RoboticArm::calculateMove(){
  //Calculate require joint values for fingers
  double ratioOpen = std::min(1.0,calculateDistanceBetweenPoints(sensedPoseTip1_,sensedPoseTip2_)/DISTANCE_BETWEEN_USER_FINGERS_FULL_OPEN);
  std::map<std::string, double> jointValueGoal = closedJointValues_;

  std::map<std::string, double>::iterator it_jv = jointValueGoal.begin();
  std::map<std::string, double>::iterator it_open = openedJointValues_.begin();
  std::map<std::string, double>::iterator it_close = closedJointValues_.begin();
  while (it_jv != jointValueGoal.end() || it_open != openedJointValues_.end() || it_close != closedJointValues_.end()){
    //Set jointValueGoal to be the ratio between the robot's MoveIt defined Open and Close preset Poses
    it_jv->second = calculateValueBetweenRange(it_open->second, it_close->second, 1-ratioOpen);
    it_jv++;
    it_open++;
    it_close++;
  }
  group_->setPoseTarget(rotatePoseStamped(sensedPosePalm_));
  gripper_group_->setJointValueTarget(jointValueGoal);
  return (group_->plan(planArm_) && gripper_group_->plan(planGripper_));
}

bool RoboticArm::executeMove(){
  return (group_->execute(planArm_) && gripper_group_->execute(planGripper_));
}

//Sets the desired poseTargets to the received input poses
void RoboticArm::updatePoseValues(const leap_controller_capstone::HandPoseStamped::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"Received Input. Now processing...");
  //Record poses received from the ROS Topic
  sensedPosePalm_ = msg->posePalm;
  sensedPoseTip2_ = msg->poseMisc[0];
  sensedPoseTip1_ = msg->poseFingerTips[0];
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
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("RoboticArm Node has started");

  ros::WallDuration(10.0).sleep();//allow for Rviz/MoveIt to start before continuing
  RoboticArm myArm = RoboticArm(node);
  myArm.beginListening();

  ros::spin();
  return 0;
}
