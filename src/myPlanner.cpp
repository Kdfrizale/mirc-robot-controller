#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <arm_mimic_capstone/HandStampedPose.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteTrajectoryGoal.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <kinova_msgs/SetFingersPositionGoal.h>
#include <boost/scoped_ptr.hpp>

#include <ctime>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <moveit/planning_pipeline/planning_pipeline.h>

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
bool messageReceived = false;

// A tolerance of 0.01 m is specified in position
// and 0.01 radians in orientation
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);//0.01 is normal

std::string moveitTrajectoryActionServer = "execute_trajectory";
std::string trajectoryActionServer = "/m1n6s200/follow_joint_trajectory";
std::string fingersPositionActionServer = "finger_positions";//might be /m16s200_driver/fingers_action/finger_positions


/////////////////////////////////////FUNCTIONS/////////////////////////////////////////////
//Sets the desired poseTargets to the received input poses
void updatePoseValues(const arm_mimic_capstone::HandStampedPose::ConstPtr& msg){
  ROS_INFO_THROTTLE(1,"I received a message.. now processing...");
  poseTip2 = msg->poseTip2;
  posePalm = msg->posePalm;
  poseTip1 = msg->poseTip1;
  messageReceived = true;
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


//Combines two trajectories so that the action server can have just one move for two plans
moveit_msgs::RobotTrajectory combineTrajectories(const moveit_msgs::RobotTrajectory mainTrajectory, const moveit_msgs::RobotTrajectory secondaryTrajectory){
  moveit_msgs::RobotTrajectory combineTrajectories = secondaryTrajectory;
  moveit_msgs::RobotTrajectory otherTrajectory = mainTrajectory;
  int smallestSize = mainTrajectory.joint_trajectory.points.size();
  int biggestSize = secondaryTrajectory.joint_trajectory.points.size();
  ROS_INFO(" the size of main is: [%zd]",mainTrajectory.joint_trajectory.points.size());
  ROS_INFO(" the size of secondary is: [%zd]",secondaryTrajectory.joint_trajectory.points.size());

  if (secondaryTrajectory.joint_trajectory.points.size() < mainTrajectory.joint_trajectory.points.size()){
    combineTrajectories = mainTrajectory;
    otherTrajectory = secondaryTrajectory;
    smallestSize = secondaryTrajectory.joint_trajectory.points.size();
    biggestSize = mainTrajectory.joint_trajectory.points.size();
    combineTrajectories.joint_trajectory.joint_names.push_back("m1n6s200_joint_finger_1");
    ROS_INFO("Switched to secondaryTrajectory being smaller");
  }
  else{
    ROS_INFO("Switched to mainTrajectory being smaller");
    combineTrajectories.joint_trajectory.joint_names.push_back("m1n6s200_joint_finger_2");
  }
  //moveit_msgs::RobotTrajectory combineTrajectories = mainTrajectory;
  //Then add additional joint_names and their corresponding values(pos,vel,accel) to the combineTrajectories
  //Also beaware of the time from start for each point, it may be neseccary to take the largest one out of main and secondary
  //combineTrajectories.joint_trajectory.joint_names.push_back("m1n6s200_joint_finger_1");
  ROS_INFO("ABOUT TO COMBINE trajectories");
  //ROS_INFO("There are [%zd] points to go through",mainTrajectory.joint_trajectory.points.size());
//  ROS_INFO("There are [%zd] points to go through",secondaryTrajectory.joint_trajectory.points.size());

  auto positionValue = otherTrajectory.joint_trajectory.points[0].positions.back();
  auto  velocityValue = otherTrajectory.joint_trajectory.points[0].velocities.back();
  auto  accelValue = otherTrajectory.joint_trajectory.points[0].accelerations.back();
  double timeDifferenceofLast = 0.0;

  for (int i =0; i < smallestSize;i++){
  //  ROS_INFO("IM in a loop for combing...");
    positionValue = otherTrajectory.joint_trajectory.points[i].positions.back();
    velocityValue = otherTrajectory.joint_trajectory.points[i].velocities.back();
    accelValue = otherTrajectory.joint_trajectory.points[i].accelerations.back();
    timeDifferenceofLast = 0.0;//reset every loop to only get the value of the last difference if differnece orrcured

    //push back the last value from secondary onto each points last postion,vel,accel //maybe change start form time
    ROS_INFO("value of finger joint posiiton pushed back: [%f]",positionValue);
    combineTrajectories.joint_trajectory.points[i].positions.push_back(positionValue);
    combineTrajectories.joint_trajectory.points[i].velocities.push_back(velocityValue);
    combineTrajectories.joint_trajectory.points[i].accelerations.push_back(accelValue);

    ROS_INFO("combines orginal starttime: [%f]", combineTrajectories.joint_trajectory.points[i].time_from_start.toSec());
    ROS_INFO("secondary orginal starttime: [%f]", otherTrajectory.joint_trajectory.points[i].time_from_start.toSec());


    if (combineTrajectories.joint_trajectory.points[i].time_from_start < otherTrajectory.joint_trajectory.points[i].time_from_start){
      timeDifferenceofLast = otherTrajectory.joint_trajectory.points[i].time_from_start.toSec() - combineTrajectories.joint_trajectory.points[i].time_from_start.toSec();
      combineTrajectories.joint_trajectory.points[i].time_from_start = otherTrajectory.joint_trajectory.points[i].time_from_start;
    }
    ROS_INFO("combines final starttime: [%f]", combineTrajectories.joint_trajectory.points[i].time_from_start.toSec());
  }

  for( int i = smallestSize; i < biggestSize; i++){
    combineTrajectories.joint_trajectory.points[i].positions.push_back(positionValue);
    combineTrajectories.joint_trajectory.points[i].time_from_start = ros::Duration(timeDifferenceofLast + combineTrajectories.joint_trajectory.points[i].time_from_start.toSec());
  }
  return combineTrajectories;
}

//get the last joint postition for each joint name in a trajectory
std::vector<double> getFinalJointPositions(moveit_msgs::RobotTrajectory inputTrajectory){
  std::vector<double> result;
  int numberOfJoints =0;
  for(auto joint_name : inputTrajectory.joint_trajectory.joint_names){
    result.push_back(0);
    numberOfJoints++;
  }

  for(auto point : inputTrajectory.joint_trajectory.points){
    int positionNum = 0;
    for(auto position : point.positions){
      result.at(positionNum) = position;
      positionNum++;
    }
  }
  return result;
}


//////////////////////////////////////////////////////////////////MAIN//////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "myPlannerStart");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(15.0);
  ros::WallDuration waitForPlan_time(0.5);
  ros::WallDuration waitForError(2);
  sleep_time.sleep();

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //---------Loop of the MAIN PROGRAM------------------------------
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  while (ros::ok()){

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //get input Pose using subscriber--which calls updatesPoseValues
    //Only continue when a new pose is received
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Subscriber sub = node_handle.subscribe("/handPoseTopic",10,updatePoseValues);
    if(!messageReceived){
      ROS_INFO_THROTTLE(1,"Message not Received...");

    }
    else{//messageReceived so start planning
      ROS_INFO_THROTTLE(1,"Message was Received!");

      std::clock_t start;
      double duration;
      start = std::clock();

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //PLANNING
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      planning_interface::MotionPlanRequest req;
      planning_interface::MotionPlanResponse res;

      getDistanceBetweenPoints(poseTip1,poseTip2);

      req.group_name = "chainArmEnd";
      moveit_msgs::Constraints pose_goal_end = kinematic_constraints::constructGoalConstraints("m1n6s200_end_effector", posePalm, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal_end);

      ROS_INFO("about to start planning");
      planning_pipeline->generatePlan(planning_scene,req,res);

      if (res.error_code_.val != res.error_code_.SUCCESS){
        ROS_ERROR("Could not compute plan successfully");
        waitForError.sleep();
        continue;//Go back and get new poseTarget
      }

      moveit_msgs::MotionPlanResponse firstResponse;
      res.getMessage(firstResponse);//Get the first motion plan


      /* First, set the state in the planning scene to the final state of the last plan */
      robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
      planning_scene->setCurrentState(firstResponse.trajectory_start);
      const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("chainArmEnd");
      robot_state.setJointGroupPositions(joint_model_group, firstResponse.trajectory.joint_trajectory.points.back().positions);

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the first plan", duration);

      //Calculate Finger joint postions here, dont forget to update model in planning scene













/*
//------------------------------------------------------------------------------------
      req.group_name = "chainArm";
      moveit_msgs::Constraints pose_goal_tip_2 = kinematic_constraints::constructGoalConstraints("m1n6s200_link_finger_tip_2", poseTip2, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal_tip_2);

      moveit_msgs::Constraints pose_goal_link6 = kinematic_constraints::constructGoalConstraints("m1n6s200_link_6", posePalm, tolerance_pose, tolerance_angle);
      req.goal_constraints.push_back(pose_goal_link6);

      ROS_INFO("about to start planning");
      planning_pipeline->generatePlan(planning_scene,req,res);

      if (res.error_code_.val != res.error_code_.SUCCESS){
        ROS_ERROR("Could not compute plan successfully");
        waitForError.sleep();
        continue;//Go back and get new poseTarget
      }

      //-------------Robot has planned to the first motion------------------------------------
      moveit_msgs::MotionPlanResponse midResponse;
      res.getMessage(midResponse);//Get the first motion plan

*/
      /* First, set the state in the planning scene to the final state of the last plan */
      /*
      robot_state = planning_scene->getCurrentStateNonConst();
      planning_scene->setCurrentState(midResponse.trajectory_start);
      joint_model_group = robot_state.getJointModelGroup("chainArm");
      robot_state.setJointGroupPositions(joint_model_group, midResponse.trajectory.joint_trajectory.points.back().positions);

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the first plan", duration);

      req2.group_name = "chainArmLeft";
      moveit_msgs::Constraints pose_goal_tip_1 = kinematic_constraints::constructGoalConstraints("m1n6s200_link_finger_tip_1", poseTip1, tolerance_pose, tolerance_angle);
      req2.goal_constraints.push_back(pose_goal_tip_1);
      req2.goal_constraints.push_back(pose_goal_link6);

      ROS_INFO("about to start planning2");
      planning_pipeline->generatePlan(planning_scene,req2,res2);

      if (res2.error_code_.val != res2.error_code_.SUCCESS){
        ROS_ERROR("Could not compute plan successfully");
        waitForError.sleep();
        continue;//Go Back and get new Pose Target
      }

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the second plan", duration);

      moveit_msgs::MotionPlanResponse endResponse;
      res2.getMessage(endResponse);

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //Update the Planning Scene Robot Model to show where the plans ended
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      robot_state = planning_scene->getCurrentStateNonConst();
      planning_scene->setCurrentState(endResponse.trajectory_start);
      joint_model_group = robot_state.getJointModelGroup("chainArmLeft");

      robot_state.setJointGroupPositions(joint_model_group, endResponse.trajectory.joint_trajectory.points.back().positions);
      */
    /*  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
      planning_scene->setCurrentState(midResponse.trajectory_start);
      int positionInVector = 0;
      std::vector<double> finalPositions = getFinalJointPositions(goal.trajectory);
      for(auto joint_name : goal.trajectory.joint_trajectory.joint_names){
        ROS_INFO("updating position for joint: %s", joint_name.c_str());
        ROS_INFO("this is the %d update this cycle", positionInVector);
        const double fingerPosition = finalPositions.at(positionInVector);
        ROS_INFO("HERE:::joint position value is: %f ",fingerPosition);
        const double* fingerPositionPointer= &fingerPosition;
        //const std::string jointFingerName = endResponse.trajectory.joint_trajectory.joint_names.back();
        ROS_INFO("The before value of the joint is: [%f]", *robot_state.getJointPositions(joint_name));
        robot_state.setJointPositions(joint_name, fingerPositionPointer );
        positionInVector++;
      }
      planning_scene->setCurrentState(robot_state);
*/

      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //Move the physical robot to the planned coordinates
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      moveit_msgs::ExecuteTrajectoryGoal goal;
      goal.trajectory = firstResponse.trajectory;
      //goal.trajectory = combineTrajectories(midResponse.trajectory, endResponse.trajectory);

      actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> ac(moveitTrajectoryActionServer,false);
      ROS_INFO("Waiting for action server to start.");
      ac.waitForServer();

      ac.sendGoal(goal);
      bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
      if (finished_before_timeout){
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
      }
      else{
        ROS_INFO("Action did not finish before the time out.");
      }

      //May need to have a wait statement here so the fingers move last, not at the same time
      //Now to move the fingers
      kinova_msgs::SetFingersPositionGoal fingerGoal;
      //kinova_msgs::FingerPosition fingers;
      fingerGoal.fingers.finger1 = 1.0;
      fingerGoal.fingers.finger2 = 1.0; //should be halfway closed

      actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac2(fingersPositionActionServer,false);
      ROS_INFO("Waiting for action server to start.");
      ac2.waitForServer();

      ac2.sendGoal(fingerGoal);
      bool finished_before_timeout2 = ac2.waitForResult(ros::Duration(30.0));
      if (finished_before_timeout2){
        actionlib::SimpleClientGoalState state2 = ac2.getState();
        ROS_INFO("Action finished: %s",state2.toString().c_str());
      }
      else{
        ROS_INFO("Action did not finish before the time out.");
      }


      //////////////////////////////////////////////////////////////////////////////////////////////////////
      //show the result in Rviz
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
      moveit_msgs::DisplayTrajectory display_trajectory;

      ROS_INFO("Visualizing the trajectory");
      //moveit_msgs::MotionPlanResponse response;

      //res.getMessage(response);//Get the first motion plan
      display_trajectory.trajectory_start = firstResponse.trajectory_start;
      display_trajectory.trajectory.push_back(goal.trajectory);

      //display_trajectory.trajectory_start = endResponse.trajectory_start;
      //display_trajectory.trajectory.push_back(goal2.trajectory);
      /*
      res2.getMessage(response);//Get the second motion plan
      display_trajectory.trajectory_start = response.trajectory_start;
      display_trajectory.trajectory.push_back(response.trajectory);
      */
      display_publisher.publish(display_trajectory);
      waitForPlan_time.sleep();

      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      ROS_INFO("It took [%f] seconds to get past the rviz display", duration);


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
