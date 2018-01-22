#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped rotatePoseStamped(geometry_msgs::PoseStamped inputPose);//maybe change this to accept axis of rotations as parameter
double getDistanceBetweenPoints(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
