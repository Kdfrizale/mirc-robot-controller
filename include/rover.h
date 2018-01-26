#include <controlled_robot.h>
#include <geometry_msgs/TwistStamped.h>

class Rover:public ControlledRobot {
public:
  Rover(ros::NodeHandle &nh);
  ~Rover();
  void beginListening();

private:
  ros::Publisher move_publisher_;
  geometry_msgs::TwistStamped goal_;

  void setDefaultMove();
  bool calculateMove();
  bool executeMove();
};
