#include <controlled_robot.h>

ControlledRobot::ControlledRobot(ros::NodeHandle &nh):nh_(nh){
  std::string input_pose_topic_name;
  nh_.getParam("leap_output_pose_topic",input_pose_topic_name);
  ROS_INFO("Listening to topic [%s] for input pose", input_pose_topic_name.c_str());
  sub_leap_hand_ = nh_.subscribe(input_pose_topic_name,1,&ControlledRobot::updatePoseValues, this);

  receivedNewPose_ = false;
}

ControlledRobot::~ControlledRobot(){
}

bool ControlledRobot::calculateMove(){
  }

bool ControlledRobot::executeMove(){
}

//Sets the desired poseTargets to the received input poses
void ControlledRobot::updatePoseValues(const leap_controller_capstone::HandPoseStamped::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"Received Input. Now processing...");
  //Record poses received from the ROS Topic
  sensedPosePalm_ = msg->posePalm;
  sensedPoseTip2_ = msg->poseMisc[0];
  sensedPoseTip1_ = msg->poseFingerTips[0];
  receivedNewPose_ = true;
}

//Begin executing callback functions for subscriptions
void ControlledRobot::beginListening(){
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
