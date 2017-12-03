#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <arm_mimic_capstone/HandStampedPose.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/SetFingersPositionGoal.h>
#include <kinova_msgs/JointAngles.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>

/////////////////////////////////////////GLOBALS//////////////////////////////////////////////
geometry_msgs::PoseStamped poseTip2;//Right Finger tip
geometry_msgs::PoseStamped posePalm;//wrist
geometry_msgs::PoseStamped poseTip1;//Left Finger tip

kinova_msgs::JointAngles last_joint_positions;
bool messageReceived = false;
bool finishedMoving = false;
bool errorCodeReceived = false;

const double FINGER_MAX = 6400;
const double distanceBetweenFingerTipsFullOpen = 0.1683; //17cm
tf::Quaternion xRotationQuaternion = tf::createQuaternionFromRPY(1.5707,0.0, 0.0);//pi/2,0,0....90 degree offset on x axis
tf::Quaternion yRotationQuaternion = tf::createQuaternionFromRPY(0.0,1.5707, 0.0);//pi/2,0,0....90 degree offset on y axis
tf::Quaternion zRotationQuaternion = tf::createQuaternionFromRPY(0.0, 0.0,1.5707);//0,0,pi/2....90 degree offset on z axis

ros::WallDuration sleep_time(3.0);
ros::WallDuration waitForSmall_time(0.5);
ros::WallDuration waitForJointMoves(0.75);

/////////////////////////////////////FUNCTIONS/////////////////////////////////////////////
//Sets the desired poseTargets to the received input poses
void updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"I received a message.. now processing...");
  tf::Quaternion originalQuarternion;
  posePalm = msg->posePalm;

  //Check if msg is pre-determined error code
  if (posePalm.pose.position.x == 123456789.0){
    ROS_INFO_THROTTLE(1,"Default Message received");
    errorCodeReceived = true;
    return;
  }
  poseTip2 = msg->poseTip2;
  poseTip1 = msg->poseTip1;

  posePalm.pose.position.y = posePalm.pose.position.y - 0.42; //offset for the arm not to be at origin --.46
  posePalm.pose.position.z = posePalm.pose.position.z + 0.10; //offset to give more vertical space

  //Switch the axis of orientation to correct the translation from the leap motion
  double temp = posePalm.pose.orientation.x;
  posePalm.pose.orientation.x = -posePalm.pose.orientation.y;
  posePalm.pose.orientation.y = temp;

  //Rotate the given pose to so the robot matchs the user's hand at 0 0 0 1 orientation
  quaternionMsgToTF(posePalm.pose.orientation , originalQuarternion);
  originalQuarternion.normalize();
  originalQuarternion *= xRotationQuaternion;
  originalQuarternion.normalize();
  originalQuarternion *= zRotationQuaternion;
  originalQuarternion.normalize();
  quaternionTFToMsg(originalQuarternion, posePalm.pose.orientation);

  messageReceived = true;
}

//Callback method to check if the robot has finished moving
void checkRobotStopped(const kinova_msgs::JointAngles::ConstPtr& msg){
    if(trunc(msg->joint1) == trunc(last_joint_positions.joint1) &&
        trunc(msg->joint2) == trunc(last_joint_positions.joint2) &&
        trunc(msg->joint3) == trunc(last_joint_positions.joint3) &&
        trunc(msg->joint4) == trunc(last_joint_positions.joint4) &&
        trunc(msg->joint5) == trunc(last_joint_positions.joint5) &&
        trunc(msg->joint6) == trunc(last_joint_positions.joint6)){
          ROS_INFO("Robot is done moving");
          finishedMoving = true;
    }
    last_joint_positions.joint1 = msg->joint1;
    last_joint_positions.joint2 = msg->joint2;
    last_joint_positions.joint3 = msg->joint3;
    last_joint_positions.joint4 = msg->joint4;
    last_joint_positions.joint5 = msg->joint5;
    last_joint_positions.joint6 = msg->joint6;
}

//Calculate the distance between two 3D points in space
double getDistanceBetweenPoints(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2){
  double result = 0.0;
  double intermediateDistance = 0.0;
  double xDistance =fabs(pose1.pose.position.x - pose2.pose.position.x);
  double yDistance =fabs(pose1.pose.position.y - pose2.pose.position.y);
  double zDistance =fabs(pose1.pose.position.z - pose2.pose.position.z);
  // ROS_INFO("the xDistance is: [%f]", xDistance);
  // ROS_INFO("the yDistance is: [%f]", yDistance);
  // ROS_INFO("the zDistance is: [%f]", zDistance);

  intermediateDistance = sqrt(pow(xDistance,2) + pow(yDistance,2));
  // ROS_INFO("the intermediateDistance is: [%f]", intermediateDistance);
  result = sqrt(pow(intermediateDistance,2) + pow(zDistance,2));
  // ROS_INFO("the final distance is: [%f]", result);
  return result;
}

//////////////////////////////////////////////////////////////////MAIN//////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "myPlannerStart");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlanningTime(0.1);
  move_group.setWorkspace(-100,-100,0.04,100,100,100);

  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> finger_client("/m1n6s200_driver/fingers_action/finger_positions", false);
  if (!finger_client.waitForServer(ros::Duration(5.0))){
    ROS_INFO_THROTTLE(1,"Waiting for the finger action server to come up...");
  }

  /* Sleep a little to allow time to startup rviz, etc. */
  sleep_time.sleep();

  ros::Subscriber sub = node_handle.subscribe("/handPoseTopic",1,updatePoseValues);
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //---------Loop of the MAIN PROGRAM------------------------------
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  while (ros::ok()){
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //get input Pose using subscriber--which calls updatesPoseValues
    //Only continue when a new pose is received
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    if(!messageReceived){
      ROS_INFO_THROTTLE(1,"Message not Received...");
    }
    else{//messageReceived so start planning
      ROS_INFO_THROTTLE(1,"Message was Received!");

      //Clock used to benchmark the system during Developement
      // std::clock_t start;
      // double duration;
      // start = std::clock();

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //PLANNING and MOVING
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      geometry_msgs::Pose desiredPose = posePalm.pose;
      move_group.setPoseTarget(desiredPose);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = move_group.plan(my_plan);
      // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      // ROS_INFO("It took [%f] seconds to get past the arm plan", duration);
      move_group.move();

      //Wait here until the arm has stopped moving
      //aka until the joints have remained the same for a period of time
      finishedMoving = false;
      ros::Subscriber subForMove = node_handle.subscribe("/m1n6s200_driver/out/joint_angles",1,checkRobotStopped);
      while(!finishedMoving){
        ROS_INFO_THROTTLE(1,"Waiting for arm to finish moving...");
        ros::spinOnce();
        if(errorCodeReceived){
          move_group.stop();
          finishedMoving = true;
          errorCodeReceived = false;
        }
        waitForJointMoves.sleep();
      }
      // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      // ROS_INFO("It took [%f] seconds to get past the arm move", duration);

      //Calculate Finger joint postions
      //ROS_INFO("this is the distance apart fingers: [%f]", getDistanceBetweenPoints(poseTip1,poseTip2) -0.01);
      double desiredDistanceApart = getDistanceBetweenPoints(poseTip1,poseTip2) -0.01; //-0.01,decrease the distance by 1cm due to the fact of bone width and skin
      desiredDistanceApart = (desiredDistanceApart * 1.35) -0.015; // Multiply the distance to allow the robot to open to its full width, which is wider than the user's hand can open
      double ratioClosed = desiredDistanceApart / distanceBetweenFingerTipsFullOpen;
      ratioClosed = 1 -ratioClosed; //flip the scale around as 6400 is closed, and 0 is open
      double finger_turn = ratioClosed * FINGER_MAX;

      //Set boundary limits on the how wide the fingers can open
      if (finger_turn < 0){
        finger_turn = 0.0;
      }
      else{
        finger_turn = std::min(finger_turn, FINGER_MAX);
      }

      kinova_msgs::SetFingersPositionGoal fingerGoal;
      fingerGoal.fingers.finger1 = finger_turn;
      fingerGoal.fingers.finger2 = finger_turn;
      finger_client.sendGoalAndWait(fingerGoal);

      // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      // ROS_INFO("It took [%f] seconds to get past the finger move", duration);

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //Reset
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      messageReceived = false;
      errorCodeReceived = false;

      // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      // ROS_INFO("It took [%f] seconds for this cycle", duration);
  }
  ros::spinOnce();
}//Loop back to start of main Program

  //Broke out of while
  //sleep_time.sleep();
  ROS_INFO("Done");
  return 0;
}
