#include<iostream>
#include<supertoroid/st_sampling.h>
#include<supertoroid/st.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



int main(int argc, char *argv[])
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined(new pcl::PointCloud<pcl::PointXYZRGB>());

supertoroid::st super;
  super.a1 = 0.012925;
  super.a2 = 0.0064596;
  super.a3 = 0.116491;
  super.a4 = 1.1;
  super.e1 = 1.307882;
  super.e2 = 1.268144;
  geometry_msgs::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 1.0;
  pose.orientation.w = 1.0;
  super.pose = pose;


 supertoroid::st super2;
  super2.a1 = 2;
  super2.a2 = 2;
  super2.a3 = 2;
  super2.a4 = 4;
  super2.e1 = 2.5;
  super2.e2 = 2.5;
  geometry_msgs::Pose pose2;
  pose2.position.x = 0;
  pose2.position.y = 0;
  pose2.position.z = 0;
  pose2.orientation.w = 1.0; // no rotation
  super2.pose = pose2;


  Sampling *sam = new Sampling(super);

  sam->sample_pilu_fisher_st();
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  sam->getCloud(cloud);
  std::cout<<"size of the first sampled cloud: "<<cloud->points.size()<<std::endl;

  Sampling *sam2 = new Sampling(super2);
  sam2->sample_pilu_fisher_st();
  pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
  sam2->getCloud(cloud2);
  std::cout<<"size of the second sampled cloud: "<<cloud2->points.size()<<std::endl;

  *cloud_combined = *cloud2;
  //*cloud_combined+= *cloud2;

  std::cout<<"Size of combined superquadrics cloud: "<<cloud_combined->points.size()<<std::endl;
  std::string fileName = "supt.pcd";
  pcl::io::savePCDFileASCII (fileName, *cloud_combined);
  std::cerr << "Saved " << cloud_combined->points.size () << " data points to " << fileName << std::endl;

  return 0;
}
