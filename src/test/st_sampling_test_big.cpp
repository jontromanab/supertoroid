#include <iostream>
#include<supertoroid/st_sampling.h>
#include<supertoroid/st.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<memory>
typedef pcl::PointCloud<PointT>::Ptr pointCloudPtr;


pointCloudPtr create_st_cloud(const double e1, const double e2, const double x,
                                                   const double y, const double z){
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  supertoroid::st super;
  super.a1 = 0.01;
  super.a2 = 0.01;
  super.a3 = 0.01;
  super.a4 = 0.2;
  super.e1 = e1;
  super.e2 = e2;
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  super.pose = pose;
  std::unique_ptr<Sampling> samp(new Sampling(super));
  samp->sample();
  samp->getCloud(cloud);
  return cloud;
}

int main(int argc, char **argv)
{
  pointCloudPtr cloud_combined(new pcl::PointCloud<PointT>());
  pointCloudPtr cloud1 = create_st_cloud(0.1, 0.1, -0.07, 0.07, 0.0);
  pointCloudPtr cloud2 = create_st_cloud(0.1, 0.5, -0.045, 0.07, 0.0);
  pointCloudPtr cloud3 = create_st_cloud(0.1, 1.0, -0.015, 0.07, 0.0);
  pointCloudPtr cloud4 = create_st_cloud(0.1, 1.5, 0.015, 0.07, 0.0);
  pointCloudPtr cloud5 = create_st_cloud(0.1, 1.9, 0.045, 0.07, 0.0);
  pointCloudPtr cloud6 = create_st_cloud(0.1, 2.5, 0.07, 0.07, 0.0);
  pointCloudPtr cloud11 = create_st_cloud(0.5, 0.1, -0.07, 0.045, 0.0);
  pointCloudPtr cloud12 = create_st_cloud(0.5, 0.5, -0.045, 0.045, 0.0);
  pointCloudPtr cloud13 = create_st_cloud(0.5, 1.0, -0.015, 0.045, 0.0);
  pointCloudPtr cloud14 = create_st_cloud(0.5, 1.5, 0.015, 0.045, 0.0);
  pointCloudPtr cloud15 = create_st_cloud(0.5, 1.9, 0.045, 0.045, 0.0);
  pointCloudPtr cloud16 = create_st_cloud(0.5, 2.5, 0.07, 0.045, 0.0);
  pointCloudPtr cloud21 = create_st_cloud(1.0, 0.1, -0.07, 0.015, 0.0);
  pointCloudPtr cloud22 = create_st_cloud(1.0, 0.5, -0.045, 0.015, 0.0);
  pointCloudPtr cloud23 = create_st_cloud(1.0, 1.0, -0.015, 0.015, 0.0);
  pointCloudPtr cloud24 = create_st_cloud(1.0, 1.5, 0.015, 0.015, 0.0);
  pointCloudPtr cloud25 = create_st_cloud(1.0, 1.9, 0.045, 0.015, 0.0);
  pointCloudPtr cloud26 = create_st_cloud(1.0, 2.5, 0.07, 0.015, 0.0);
  *cloud_combined+= *cloud1;
  *cloud_combined+= *cloud2;
  *cloud_combined+= *cloud3;
  *cloud_combined+= *cloud4;
  *cloud_combined+= *cloud5;
  *cloud_combined+= *cloud6;
  *cloud_combined+= *cloud11;
  *cloud_combined+= *cloud12;
  *cloud_combined+= *cloud13;
  *cloud_combined+= *cloud14;
  *cloud_combined+= *cloud15;
  *cloud_combined+= *cloud16;
  *cloud_combined+= *cloud21;
  *cloud_combined+= *cloud22;
  *cloud_combined+= *cloud23;
  *cloud_combined+= *cloud24;
  *cloud_combined+= *cloud25;
  *cloud_combined+= *cloud26;

  pcl::io::savePCDFileASCII ("big_supertoroid.pcd", *cloud_combined);
  std::cerr << "Saved " << cloud_combined->points.size () << " data points to big_supertoroid.pcd." << std::endl;
  return 0;
}
