#include<iostream>
#include<supertoroid/st_segmentation.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

int main(int argc, char * argv[])
{
  if(argc!=2)
    return 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  if(pcl::io::loadPCDFile(argv[1], *cloud)==-1)
    return -1;

  std::cout<<"Size of input cloud: "<<cloud->size()<<std::endl;

  Segmentation::Parameters params;
  params.zmin = 0.01;
  params.zmax = 2.0;

  Segmentation::ws_Parameters ws_params;
  ws_params.min_x = -0.5;
  ws_params.max_x = 0.3;
  ws_params.min_y = 0.0;
  ws_params.max_y = 3.0;
  ws_params.min_z = 0.0;
  ws_params.max_z = 0.9;


  Segmentation* seg = new Segmentation(cloud, params, ws_params);
  seg->segment();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  seg->getTablecloud(table_cloud);
  std::cout<<"Size of table cloud: "<<table_cloud->points.size()<<std::endl;

  pcl::io::savePCDFileASCII("table_cloud.pcd", *table_cloud);
  std::cerr<<"Saved "<<table_cloud->points.size()<<" data pointst to table_cloud.pcd"<<std::endl;


  return 0;
}
