#ifndef UTIL_H
#define UTIL_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointT;

void pose_to_transform(const geometry_msgs::Pose& pose, Eigen::Affine3f& transform){
  transform = Eigen::Affine3f::Identity();
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  transform.translation()<<pose.position.x, pose.position.y, pose.position.z;
  transform.rotate(q);
}

#endif // UTIL_H
