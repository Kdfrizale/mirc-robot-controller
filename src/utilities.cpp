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
