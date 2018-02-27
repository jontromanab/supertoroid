#ifndef UTIL_H
#define UTIL_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include<supertoroid/st.h>
typedef pcl::PointXYZRGB PointT;

void pose_to_transform(const geometry_msgs::Pose& pose, Eigen::Affine3f& transform){
  transform = Eigen::Affine3f::Identity();
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  transform.translation()<<pose.position.x, pose.position.y, pose.position.z;
  transform.rotate(q);
}

void create_rotation_matrix(const double ax, const double ay, const double az, Eigen::Affine3d &rot_matrix){
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

void getParamFromTransform(const Eigen::Affine3d& trans, double& tx, double& ty, double& tz, double& ax,
                           double& ay, double& az){
  Eigen::Vector3d t = trans.translation();
  Eigen::Matrix3d rot_matrix = trans.rotation();
  tx = t(0);
  ty = t(1);
  tz = t(2);
  Eigen::Vector3d rot_angle = rot_matrix.eulerAngles(0,1,2);
  ax = rot_angle[0];
  ay = rot_angle[1];
  az = rot_angle[2];
}



void st_clampParameters(double& e1_clamped, double& e2_clamped)
{
  if(e1_clamped < 0.1)
    e1_clamped = 0.1;
  else if(e1_clamped > 1.9)
    e1_clamped = 1.9;
  if(e2_clamped < 0.1)
    e2_clamped = 0.1;
  else if(e2_clamped > 1.9)
    e2_clamped = 1.9;
}

double st_function(const PointT& point, const supertoroid::st& param)
{
  double e1_clamped = param.e1;
  double e2_clamped = param.e2;
  st_clampParameters(e1_clamped, e2_clamped);
  double t1 = pow(std::abs(point.x / param.a1), 2.0/e2_clamped);
  double t2 = pow(std::abs(point.y / param.a2), 2.0/e2_clamped);
  double t3 = pow(std::abs(point.z / param.a3), 2.0/e1_clamped);
  double f = (pow(std::abs(t1+t2), e2_clamped/e1_clamped) - param.a4) + t3;
  return (pow(f,e1_clamped) - 1.0 );
}

double st_normPoint(const PointT &point){
  double value = sqrt(point.x * point.x + point.y * point.y + point.z * point.z );
  return value;
}

double st_function_scale_weighting(const PointT& point, const supertoroid::st& param){
  double e1_clamped = param.e1;
  double e2_clamped = param.e2;
  st_clampParameters(e1_clamped, e2_clamped);
  double t1 = pow(std::abs(point.x / param.a1), 2.0/e2_clamped);
  double t2 = pow(std::abs(point.y / param.a2), 2.0/e2_clamped);
  double t3 = pow(std::abs(point.z / param.a3), 2.0/e1_clamped);
  double f = (pow(std::abs(t1+t2), e2_clamped/e1_clamped)  - param.a4)+ t3;
  double value = (pow(f, e1_clamped/2.0) - 1.0) * pow(param.a1 * param.a2 * param.a3, 0.25);
  return (value);
}

double st_error(const pcl::PointCloud<PointT>::Ptr cloud, const supertoroid::st &param){
  Eigen::Affine3f transform;
  pose_to_transform(param.pose, transform);
  pcl::PointCloud<PointT>::Ptr new_cloud(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud, *new_cloud, transform);
  double error = 0.0;
  for(size_t i=0;i<new_cloud->size();++i){
    double op =st_normPoint(new_cloud->at(i));
    double val = op * st_function_scale_weighting(new_cloud->at(i), param);
    error += val * val;
  }
  error /=new_cloud->size();
  return error;
}





double st_function(const double &x, const double &y,
                   const double &z, const double &a,
                   const double &b, const double &c,
                   const double &d, const double &e1,
                   const double &e2)
{
  double e1_clamped = e1;
  double e2_clamped = e2;
  st_clampParameters(e1_clamped, e2_clamped);

  //std::cout<<"e1: "<<e1_clamped<<" e2: "<<e2_clamped<<std::endl;
  double t1 = pow(std::abs(x / a), 2.0/e2_clamped);
  double t2 = pow(std::abs(y / b), 2.0/e2_clamped);
  double t3 = pow(std::abs(z / c), 2.0/e1_clamped);
  double g = pow(std::abs(pow((t1+t2), e2_clamped/e1_clamped)-d), 2.0/e1_clamped) + t3;
  //double f = pow((pow(std::abs(t1+t2), e2_clamped/e1_clamped)  - d,2.0/e1_clamped)+ t3;
  //double value = pow((g - 1.0),e1_clamped);// * pow(a * b * c * d, 0.025);
  double value = (pow(g, e1_clamped/2.0) - 1.0);//* pow(a * b * c *d, 0.25);
  return (value);
}

double st_function_b2(const double& x, const double& y, const double& a, const double& b,
                      const double& e2)
{
  double e2_clamped = e2;
  double e1 = 1.0;
  st_clampParameters(e1, e2_clamped);
  double t1 = pow(std::abs(x/a), 2.0/e2_clamped);
  double t2 = pow(std::abs(y/b), 2.0/e2_clamped);
  double t3 = t1 + t2;
  double value = (pow(t3, e2_clamped/2.0) - 1.0);
  return value;
}

double st_function_b1(const double &x, const double &y,
                      const double &z, const double &a,
                      const double &b, const double &c,
                      const double &d, const double &e1,
                      const double &e2)
{
  double e1_clamped = e1;
  double e2_clamped = e2;
  st_clampParameters(e1_clamped, e2_clamped);
  double t1 = st_function_b2(x, y, a, b, e2)*sqrt((x*x)+(y*y));
  double t2 = pow(std::abs(t1/d), 2.0/e1_clamped);
  double t3 = pow(std::abs(z/c), 2.0/e1_clamped);
  double t4 = t2+t3;
  double value = (pow(t4, e1_clamped/2.0)-1.0);
  return value;

}


#endif // UTIL_H
