#include <ros/ros.h>
#include <supertoroid/st_fitting.h>
#include <supertoroid/st.h>
#include <supertoroid/st_sampling.h>

#include <pcl/io/pcd_io.h>

int main(int argc, char *argv[])
{
  if(argc !=2)
    return (0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  if(pcl::io::loadPCDFile(argv[1], *cloud)== -1)
      return -1;

  std::cout<<"Size of point cloud: "<<cloud->size()<<std::endl;

  Fitting* fit = new Fitting(cloud);
  supertoroid::st min_param;
  fit->fit();
  fit->getMinParams(min_param);


  std::cout<<"Minimum parameters is: "<<"a1:"<<min_param.a1<<"  a2:"
            <<min_param.a2<<" a3:"<<min_param.a3<<" a4: "<<min_param.a4<<" e1: "<<min_param.e1<<" e2:"<<min_param.e2<<" position:"
            <<min_param.pose.position.x<<" "<<min_param.pose.position.y<<min_param.pose.position.z<<" orientation:"
            <<min_param.pose.orientation.x<<" "<<min_param.pose.orientation.y<<" "<<min_param.pose.orientation.z<<" "
            <<min_param.pose.orientation.w<<std::endl;

  delete fit;

  Sampling *sam = new Sampling(min_param);
  sam->sample();
  pcl::PointCloud<PointT>::Ptr sampled_cloud(new pcl::PointCloud<PointT>);
  sam->getCloud(sampled_cloud);
  std::cout<<"Size of the sampled cloud: "<<sampled_cloud->points.size()<<std::endl;
  *sampled_cloud+=*cloud;
  pcl::io::savePCDFileASCII ("supertoroid_fitted.pcd", *sampled_cloud);
  std::cerr << "Saved " << sampled_cloud->points.size () << " supertoroid_fitted.pcd." << std::endl;



  ROS_INFO("Hello world!");
  return 0;
}
