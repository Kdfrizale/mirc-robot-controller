#include <moveit/move_group_interface/move_group_interface.h>
#include <controlled_robot.h>

class RoboticArm:public ControlledRobot {
public:
  RoboticArm(ros::NodeHandle &nh);
  ~RoboticArm();

private:
  std::map<std::string, double> closedJointValues_;
  std::map<std::string, double> openedJointValues_;

  moveit::planning_interface::MoveGroupInterface* group_;
  moveit::planning_interface::MoveGroupInterface* gripper_group_;
  moveit::planning_interface::MoveGroupInterface::Plan planArm_;
  moveit::planning_interface::MoveGroupInterface::Plan planGripper_;

  bool calculateMove();
  bool executeMove();

  void printPlanInfo(moveit::planning_interface::MoveGroupInterface::Plan aPlan);
};
