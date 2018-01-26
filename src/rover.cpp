#include <rover.h>

Rover::Rover(ros::NodeHandle &nh):ControlledRobot(nh){
  std::string output_topic_name;
  nh_.getParam("output_publish_topic",output_topic_name);
  move_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(output_topic_name,10);
  ROS_INFO("Publishing TwistStamped move to topic [%s]", output_topic_name.c_str());
}

Rover::~Rover(){
}

//Calculate require TwistStamped Message
bool Rover::calculateMove(){
  //Multiply the speed and turning rate by a scale coefficient based on size of rover
  goal_.twist.linear.x = -sensedPosePalm_.pose.position.y * 3;
  goal_.twist.angular.z = sensedPosePalm_.pose.position.x * 4.5;
  return true;
}

void Rover::setDefaultMove(){
  goal_.twist.linear.x = 0;
  goal_.twist.linear.y = 0;
  goal_.twist.linear.z = 0;
  goal_.twist.angular.x = 0;
  goal_.twist.angular.y = 0;
  goal_.twist.angular.z = 0;
}

bool Rover::executeMove(){
  move_publisher_.publish(goal_);
  return true;
}

//Begin executing callback functions for subscriptions
void Rover::beginListening(){
  while (ros::ok()){
    ROS_INFO_THROTTLE(1,"Waiting for input...");
    ros::spinOnce();
    if(receivedNewPose_){
      receivedNewPose_ = false;
      this->calculateMove();
    }
    else{
      //No Input Detected
      this->setDefaultMove();
    }
    this->executeMove();
    ros::WallDuration(0.1).sleep();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "rover_node");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Rover Node has started");

  Rover myRover = Rover(node);
  myRover.beginListening();

  ros::spin();
  return 0;
}
