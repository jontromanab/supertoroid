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
  super.a1 = 0.1;
  super.a2 = 0.1;
  super.a3 = 0.1;
  super.a4 = 2.0;
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
  pointCloudPtr cloud1 = create_st_cloud(0.5, 0.5, 0.0, 0.0, 0.0);
  *cloud_combined+= *cloud1;

  pcl::io::savePCDFileASCII ("big_supertoroid.pcd", *cloud_combined);
  std::cerr << "Saved " << cloud_combined->points.size () << " data points to supertoroid.pcd." << std::endl;
  return 0;
}
