#ifndef ST_SEGMENTATION_H
#define ST_SEGMENTATION_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/surface/concave_hull.h>
#include<pcl/segmentation/extract_polygonal_prism_data.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr CloudPtr;

class Segmentation{
public:
  struct Parameters{
    double zmin;
    double zmax;
    int th_points;
  }
  Segmentation(const CloudPtr& input_cloud, const Parameters& param);
  bool segment();
  bool initialized;
  void getTablecloud(CloudPtr &table_cloud);
  void getObjectsOnTable(CloudPtr& objects_on_table);

private:
  void detectObjectsOntable(CloudPtr cloud, double zmin, double zmax, bool filter_input_cloud);


  CloudPtr cloud_;
  CloudPtr table_plane_cloud_;
  CloudPtr objects_on_table_;

  PointCloud convexHull_;
  pcl::ModelCoefficients plane_coefficients_;

  double zmin_;
  double zmax_;
  int th_points_;

};

#endif // ST_SEGMENTATION_H
