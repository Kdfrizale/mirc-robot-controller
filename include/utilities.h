#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

const double DISTANCE_BETWEEN_USER_FINGERS_FULL_OPEN = 0.10;//meters

geometry_msgs::PoseStamped rotatePoseStamped(geometry_msgs::PoseStamped inputPose);//maybe change this to accept axis of rotations as parameter
double calculateDistanceBetweenPoints(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
double calculateValueBetweenRange(double rangeLow, double rangeHigh, double ratio);
