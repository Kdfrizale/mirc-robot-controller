#include <robotic_arm.h>

RoboticArm::RoboticArm(ros::NodeHandle &nh):ControlledRobot(nh){
  //TODO make these parameters
  std::string tempParam;
  nh_.getParam("moveit_planning_group_arm_name",tempParam);
  group_ = new moveit::planning_interface::MoveGroupInterface(tempParam);

  nh_.getParam("moveit_planning_group_gripper_name",tempParam);
  gripper_group_ = new moveit::planning_interface::MoveGroupInterface(tempParam);

  nh_.getParam("moveit_gripper_close_pose_name",tempParam);
  closedJointValues_ = gripper_group_->getNamedTargetValues(tempParam);

  nh_.getParam("moveit_gripper_open_pose_name",tempParam);
  openedJointValues_ = gripper_group_->getNamedTargetValues(tempParam);
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
  //group_->setPoseTarget(group_->getRandomPose());
  gripper_group_->setJointValueTarget(jointValueGoal);
  return (group_->plan(planArm_) && gripper_group_->plan(planGripper_));
}

bool RoboticArm::executeMove(){
  printPlanInfo(planArm_);
  planArm_.trajectory_.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(8.1);
  return (group_->asyncExecute(planArm_) && gripper_group_->asyncExecute(planGripper_));
}

void RoboticArm::printPlanInfo(moveit::planning_interface::MoveGroupInterface::Plan aPlan){
  //ROS_INFO("ROS TIME: [%f]", ros::Time::now().toSec());
  ROS_INFO("PLAN INFO: the planning time is [%f]", aPlan.planning_time_);
  ROS_INFO("PLAN INFO: the time stamp is [%f]", aPlan.trajectory_.joint_trajectory.header.stamp.sec);
  for (std::string name : aPlan.trajectory_.joint_trajectory.joint_names){
    ROS_INFO("PLAN INFO: Joint Name: [%s]", name.c_str());
  }
  for (trajectory_msgs::JointTrajectoryPoint aPoint : aPlan.trajectory_.joint_trajectory.points){
    ROS_INFO("PLAN INFO: Time from start for a point: [%f]", aPoint.time_from_start.toSec());
  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "robotic_arm_node");
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
