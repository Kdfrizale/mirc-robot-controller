#include <utilities.h>

//Return a pose that has been rotated 90 degrees on the X then Z axis
geometry_msgs::PoseStamped rotatePoseStamped(geometry_msgs::PoseStamped inputPose){
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
  return inputPose;
}

//Calculate the distance between two 3D points in space
double getDistanceBetweenPoints(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2){
  double xDistance =fabs(pose1.pose.position.x - pose2.pose.position.x);
  double yDistance =fabs(pose1.pose.position.y - pose2.pose.position.y);
  double zDistance =fabs(pose1.pose.position.z - pose2.pose.position.z);

  double intermediateDistance = sqrt(pow(xDistance,2) + pow(yDistance,2));
  return sqrt(pow(intermediateDistance,2) + pow(zDistance,2));
}
