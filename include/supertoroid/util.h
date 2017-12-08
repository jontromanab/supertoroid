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

void create_rotation_matrix(const double rx, const double ry, const double rz, Eigen::Affine3d &rot_matrix){
  Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1,0,0)));
  Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0,1,0)));
  Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0,0,1)));
  rot_matrix =  rx * rz * ry;
}

void create_transformation_matrix(const double tx, const double ty, const double tz, const double rx, const double ry,
                                  const double rz, Eigen::Affine3d &trans_mat){
  trans_mat = Eigen::Affine3d::Identity();
  Eigen::Affine3d rot_matrix = Eigen::Affine3d::Identity();
  create_rotation_matrix(rx, ry, rz, rot_matrix);
  Eigen::Affine3d translation_matrix(Eigen::Translation3d(Eigen::Vector3d(tx, ty, tz)));
  trans_mat = translation_matrix * rot_matrix;
}

#endif // UTIL_H
