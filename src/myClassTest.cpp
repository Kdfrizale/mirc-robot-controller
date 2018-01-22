#include <myClassTest.h>


RoboticArm::RoboticArm(ros::NodeHandle &nh):nh_(nh){
  ros::Subscriber sub = nh_.subscribe("/handPoseTopic",1,&RoboticArm::updatePoseValues, this);


  //group_ = new moveit::planning_interface::MoveGroupInterface("arm");
  ///////static const std::string PLANNING_GROUP = "arm";
  ///////////moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  beginListening();
}

RoboticArm::~RoboticArm(){
    delete group_;
    delete gripper_group_;
}

bool RoboticArm::planAndMove(){
  ROS_INFO_THROTTLE(1, "here is were the move group would be accessed");
  ROS_INFO("The received Pose Position X is: [%f]", this->sensedPosePalm_.pose.position.x);
  return true;
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

geometry_msgs::Pose rotatePose(geometry_msgs::PoseStamped &inputPose){
  tf::Quaternion xRotationQuaternion = tf::createQuaternionFromRPY(1.5707,0.0, 0.0);//pi/2,0,0....90 degree offset on x axis
  tf::Quaternion yRotationQuaternion = tf::createQuaternionFromRPY(0.0,1.5707, 0.0);//pi/2,0,0....90 degree offset on y axis
  tf::Quaternion zRotationQuaternion = tf::createQuaternionFromRPY(0.0, 0.0,1.5707);//0,0,pi/2....90 degree offset on z axis

  tf::Quaternion originalQuarternion;
  quaternionMsgToTF(inputPose.pose.orientation , originalQuarternion);
  originalQuarternion.normalize();
  originalQuarternion *= xRotationQuaternion;
  originalQuarternion.normalize();
  originalQuarternion *= zRotationQuaternion;
  originalQuarternion.normalize();
  quaternionTFToMsg(originalQuarternion, inputPose.pose.orientation);
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
