#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <arm_mimic_capstone/HandStampedPose.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteTrajectoryGoal.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/SetFingersPositionGoal.h>
#include <kinova_msgs/JointAngles.h>
#include <boost/scoped_ptr.hpp>

#include <ctime>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <std_msgs/Float64.h>
#include <vector>
#include <cmath>

/////////////////////////////////////////GLOBALS//////////////////////////////////////////////
geometry_msgs::PoseStamped poseTip2;//Right Finger tip
geometry_msgs::PoseStamped posePalm;//wrist
geometry_msgs::PoseStamped poseTip1;//Left Finger tip

kinova_msgs::JointAngles last_joint_positions;
bool messageReceived = false;
bool finishedMoving = false;


// A tolerance of 0.01 m is specified in position
// and 0.01 radians in orientation
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);//0.01 is normal

std::string moveitTrajectoryActionServer = "execute_trajectory";
//std::string trajectoryActionServer = "/m1n6s200/follow_joint_trajectory";
std::string trajectoryActionServer = "m1n6s200_driver/arm_controller/follow_joint_trajectory";
std::string fingersPositionActionServer = "finger_positions";//might be /m1n6s200_driver/fingers_action/finger_positions

const double distanceBetweenFingersBase = 0.0625;//6.25cm
//const double lengthOfFingers = 0.09;//9cm
const double fingersAngleOffset = 60; //60 degrees
const double FINGER_MAX = 6400;
const double distanceBetweenFingerTipsFullOpen = 0.1683; //17cm
tf::Quaternion xRotationQuaternion = tf::createQuaternionFromRPY(1.5707,0.0, 0.0);//pi/2,0,0....90 degree offset on x axis
tf::Quaternion zRotationQuaternion = tf::createQuaternionFromRPY(0.0, 0.0,1.5707);//0,0,pi/2....90 degree offset on z axis

/////////////////////////////////////FUNCTIONS/////////////////////////////////////////////
//Sets the desired poseTargets to the received input poses
void updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"I received a message.. now processing...");
  tf::Quaternion originalQuarternion;
  poseTip2 = msg->poseTip2;
  posePalm = msg->posePalm;
  poseTip1 = msg->poseTip1;
  posePalm.pose.position.y = posePalm.pose.position.y - 0.3; //offset for the arm not to be at origin --.46
  posePalm.pose.position.z = posePalm.pose.position.z + 0.15; //offset to give more vertical

  posePalm.pose.orientation.x = 0;
  posePalm.pose.orientation.y = 0;
  posePalm.pose.orientation.z = 0;
  posePalm.pose.orientation.w = 1;
  
  ROS_INFO("this is the quaternion before:[%f] ", posePalm.pose.orientation.x);
  ROS_INFO("this is the quaternion before:[%f] ", posePalm.pose.orientation.y);
  ROS_INFO("this is the quaternion before:[%f] ", posePalm.pose.orientation.z);
  ROS_INFO("this is the quaternion before:[%f] ", posePalm.pose.orientation.w);
  quaternionMsgToTF(posePalm.pose.orientation , originalQuarternion);
  originalQuarternion *= xRotationQuaternion;
  originalQuarternion.normalize();
  originalQuarternion *= zRotationQuaternion;
  quaternionTFToMsg(originalQuarternion, posePalm.pose.orientation);
  ROS_INFO("this is the quaternion after:[%f] ", posePalm.pose.orientation.x);
  ROS_INFO("this is the quaternion after:[%f] ", posePalm.pose.orientation.y);
  ROS_INFO("this is the quaternion after:[%f] ", posePalm.pose.orientation.z);
  ROS_INFO("this is the quaternion after:[%f] ", posePalm.pose.orientation.w);



  messageReceived = true;
}

void checkRobotStopped(const kinova_msgs::JointAngles::ConstPtr& msg){
  ROS_INFO("checking if robot has stopped moving...");
  ROS_INFO("this is a joint position: [%f]", msg->joint1);
  ROS_INFO("this is a joint position: [%f]", msg->joint2);
  ROS_INFO("this is a joint position: [%f]", msg->joint3);
  ROS_INFO("this is a joint position: [%f]", msg->joint4);
  ROS_INFO("this is a joint position: [%f]", msg->joint5);
  ROS_INFO("this is a joint position: [%f]", msg->joint6);
  ROS_INFO("Now below is the previous joint Values");
  ROS_INFO("this is a joint position: [%f]", last_joint_positions.joint1);
  ROS_INFO("this is a joint position: [%f]", last_joint_positions.joint2);
  ROS_INFO("this is a joint position: [%f]", last_joint_positions.joint3);
  ROS_INFO("this is a joint position: [%f]", last_joint_positions.joint4);
  ROS_INFO("this is a joint position: [%f]", last_joint_positions.joint5);
  ROS_INFO("this is a joint position: [%f]", last_joint_positions.joint6);
    if(trunc(msg->joint1) == trunc(last_joint_positions.joint1) &&
        trunc(msg->joint2) == trunc(last_joint_positions.joint2) &&
        trunc(msg->joint3) == trunc(last_joint_positions.joint3) &&
        trunc(msg->joint4) == trunc(last_joint_positions.joint4) &&
        trunc(msg->joint5) == trunc(last_joint_positions.joint5) &&
        trunc(msg->joint6) == trunc(last_joint_positions.joint6)){
          ROS_INFO("Im done moving");
          finishedMoving = true;
    }
    last_joint_positions.joint1 = msg->joint1;
    last_joint_positions.joint2 = msg->joint2;
    last_joint_positions.joint3 = msg->joint3;
    last_joint_positions.joint4 = msg->joint4;
    last_joint_positions.joint5 = msg->joint5;
    last_joint_positions.joint6 = msg->joint6;

}

double getDistanceBetweenPoints(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2){
  double result = 0.0;
  double intermediateDistance = 0.0;
  double xDistance =fabs(pose1.pose.position.x - pose2.pose.position.x);
  double yDistance =fabs(pose1.pose.position.y - pose2.pose.position.y);
  double zDistance =fabs(pose1.pose.position.z - pose2.pose.position.z);
  ROS_INFO("the xDistance is: [%f]", xDistance);
  ROS_INFO("the yDistance is: [%f]", yDistance);
  ROS_INFO("the zDistance is: [%f]", zDistance);

  intermediateDistance = sqrt(pow(xDistance,2) + pow(yDistance,2));
  ROS_INFO("the intermediateDistance is: [%f]", intermediateDistance);
  result = sqrt(pow(intermediateDistance,2) + pow(zDistance,2));
  ROS_INFO("the final distance is: [%f]", result);
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
  //moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
  //gripper_group_ = new moveit::planning_interface::MoveGroup("gripper");
  //move_group->setEndEffectorLink("m1n6s200_end_effector");
  //
  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> finger_client("/m1n6s200_driver/fingers_action/finger_positions", false);
  // if (!finger_client.waitForServer(ros::Duration(5.0))){
  //   ROS_INFO_THROTTLE(1,"Waiting for the finger action server to come up...");
  // }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(10.0);
  ros::WallDuration waitForPlan_time(0.5);
  ros::WallDuration waitForError(2);
  ros::WallDuration waitForJointMoves(0.75);
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

      std::clock_t start;
      double duration;
      start = std::clock();

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //PLANNING and MOVING
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      geometry_msgs::Pose desiredPose = posePalm.pose;
      move_group.setPoseTarget(desiredPose);
      //move_group.setPlanningTime(5);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = move_group.plan(my_plan);

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the arm plan", duration);

      move_group.move();
      //move_group.execute(my_plan);
      ROS_INFO("This shouldn't appear until after the robot has finished moving..");
      //Could add a check to check that all the published joints have remained the same to verify move is over?
      // if (success){
      //   move_group.move();
      // }

      //Wait here until the arm has stopped moving
      //aka until the joints have remained the same for a period of time
      finishedMoving = false;
      ros::Rate r(10); //10 Hz
      ros::Subscriber subForMove = node_handle.subscribe("/m1n6s200_driver/out/joint_angles",1,checkRobotStopped);
      while(!finishedMoving){
        ROS_INFO("im in the while loop");
        //maybe add a wait
        ros::spinOnce();
        r.sleep();
      }


      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the arm move", duration);


      //Calculate Finger joint postions here, dont forget to update model in planning scene
      double desiredDistanceApart = getDistanceBetweenPoints(poseTip1,poseTip2) - 0.015; //decrease the distance by 15mm due to the fact of bone width and skin
      //DesiredDistanceApart = DistanceBetweenFingersBase + 2* LengthOfFinger*cos(JointPosition*60degrees +StationaryAngleDegreeOffset)
      //cos only does radians, so need to convert to degrees with equation -> cos(degrees * pi/180)
      //above wont work due to the finger possiblitiles being 0 to 6400 instead of 0 to 2

      //in simulation move fingers
      //gripper_group.setNamedTarget("Close");
      //gripper_group.move();

      //move fingers to desired joint angle in safe maner
      double ratioClosed = desiredDistanceApart / distanceBetweenFingerTipsFullOpen;
      ratioClosed = 1 -ratioClosed; //flip the scale around as 6400 is closed, and 0 is open
      //double finger_turn = 3200;//This should turn halfway-demo data
      double finger_turn = ratioClosed * FINGER_MAX;
      ROS_INFO("Calculated finger_turn is : [%f]", finger_turn);

      if (finger_turn < 0){
        finger_turn = 0.0;
      }
      else{
        finger_turn = std::min(finger_turn, FINGER_MAX);
      }

      //sleep_time.sleep();//give time for the robot to move to the desired goal
      ROS_INFO("Final finger_turn is : [%f]", finger_turn);
      kinova_msgs::SetFingersPositionGoal fingerGoal;
      fingerGoal.fingers.finger1 = finger_turn;
      fingerGoal.fingers.finger2 = finger_turn;
      finger_client.sendGoal(fingerGoal);

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the finger move", duration);


      //
      // if (finger_client.waitForResult(ros::Duration(5.0))){
      //   finger_client.getResult();
      //   return true;
      // }
      // else{
      //   finger_client.cancelAllGoals();
      //   ROS_WARN_STREAM("The gripper action timed-out");
      //   return false;
      // }



      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //Reset
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      messageReceived = false;

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds for this cycle", duration);
  }
  ros::spinOnce();

  //Maybe add wait here...
  waitForPlan_time.sleep();
}//Loop back to start of main Program

  //Broke out of while
  sleep_time.sleep();
  ROS_INFO("Done");

  return 0;
}
