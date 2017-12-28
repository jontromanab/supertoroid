#include<supertoroid/st_segmentation.h>
#include<pcl/filters/crop_box.h>


Segmentation::Segmentation(const CloudPtr &input_cloud, const Parameters &param)
  :table_plane_cloud_(new PointCloud), objects_on_table_(new PointCloud){
  cloud_ = input_cloud;
  this->zmin_ = param.zmin;
  this->zmax_ = param.zmax;
  this->initialized = true;
  this->ws_filter_ = false;
}

Segmentation::Segmentation(const CloudPtr &input_cloud, const Parameters &param,const ws_Parameters& ws_param)
  :table_plane_cloud_(new PointCloud), objects_on_table_(new PointCloud), ws_cloud_(new PointCloud), filtered_cloud_
(new PointCloud){
  cloud_ = input_cloud;
  this->zmin_ = param.zmin;
  this->zmax_ = param.zmax;
  this->initialized = true;

  this->min_x_ = ws_param.min_x;
  this->max_x_ = ws_param.max_x;
  this->min_y_ = ws_param.min_y;
  this->max_y_ = ws_param.max_y;
  this->min_z_ = ws_param.min_z;
  this->max_z_ = ws_param.max_z;
  this->ws_filter_ = true;
}


void Segmentation::detectObjectsOntable(CloudPtr cloud, double zmin, double zmax, bool filter_input_cloud){
  pcl::SACSegmentation<PointT> seg;
  if(ws_filter_){
    CloudPtr cloud_nan(new PointCloud);
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud);
    Eigen::Vector4f min;
    min<<min_x_, min_y_, min_z_,1;
    Eigen::Vector4f max;
    max<<max_x_, max_y_, max_z_,1;
    crop.setMin(min);
    crop.setMax(max);
    crop.filter(*cloud_nan);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_nan, *filtered_cloud_, indices);
    ws_cloud_ = filtered_cloud_;
    seg.setInputCloud(filtered_cloud_);
  }

  else  seg.setInputCloud(cloud);
  std::cout<<"We are here: "<<std::endl;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setOptimizeCoefficients(true);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  seg.segment(*planeIndices, this->plane_coefficients_);
  if(planeIndices->indices.size()==0)
    std::cout<<"Could not find a plane"<<std::endl;
  else{
    pcl::ExtractIndices<PointT> extract;
    if(ws_filter_)extract.setInputCloud(filtered_cloud_);
    else extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    extract.filter(*table_plane_cloud_);


    /*pcl::ConvexHull<PointT> hull;
    if(ws_filter_) hull.setInputCloud(filtered_cloud_);
    else hull.setInputCloud(cloud);

    hull.setDimension(2);
    hull.reconstruct(this->convexHull_);

    if(hull.getDimension()==2)
    {
      pcl::ExtractPolygonalPrismData<PointT> prism;
      if(ws_filter_) prism.setInputCloud(filtered_cloud_);
      else prism.setInputCloud(cloud);

      prism.setInputPlanarHull(convexHull_.makeShared());
      prism.setHeightLimits(zmin, zmax);
      pcl::PointIndices::Ptr obj_idx(new pcl::PointIndices());
      prism.segment(*obj_idx);
      extract.filter(*cloud);
      objects_on_table_ = cloud;
      if(filter_input_cloud){
        if(ws_filter_)
        {
          extract.filter(*filtered_cloud_);
          objects_on_table_ = filtered_cloud_;
        }
        else
        {
          extract.filter(*cloud);
          objects_on_table_ = cloud;
        }
      }
    }*/
  }
}

void Segmentation::getTablecloud(CloudPtr &table_cloud) const {
  table_cloud = table_plane_cloud_;
}

void Segmentation::getObjectsOnTable(CloudPtr &objects_on_table) const{
  objects_on_table = objects_on_table_;
}

void Segmentation::getWsCloud(CloudPtr &ws_cloud) const {
  ws_cloud = ws_cloud_;
}

bool Segmentation::segment(){
  if(this->initialized){
    detectObjectsOntable(this->cloud_, this->zmin_, this->zmax_, true);
  }
}
