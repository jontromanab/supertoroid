#include<supertoroid/st_sampling.h>
#include<ctime>
#include<supertoroid/util.h>

Sampling::Sampling(const supertoroid::st &st_params):
  params_(st_params), cloud_(new pcl::PointCloud<PointT>){

  struct timeval time;
  gettimeofday(&time, NULL);
  srand((time.tv_sec * 1000)+(time.tv_usec/1000));
  r_ = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
  g_ = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
  b_ = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
}


void Sampling::transformCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud,
                              pcl::PointCloud<PointT>::Ptr &output_cloud){
  Eigen::Affine3f transform;
  pose_to_transform(params_.pose, transform);
  pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
}


void Sampling::getCloud(pcl::PointCloud<PointT>::Ptr &cloud)
{
  cloud = cloud_;
}

void Sampling::sample()
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    double cn, sn, sw, cw;
    double n,w;
    int num_n, num_w;
    double dn, dw;
    dn = 5.0 * M_PI/180.0;
    dw = 5.0 * M_PI/180.0;
    num_n = (int)(M_PI/dn);
    num_w = (int)(2*M_PI/dw);
    n = -M_PI/2.0;
    for(int i=0;i<num_n;++i)
    {
      n+=dn;
      cn = cos(n);
      sn = sin(n);
      w = -M_PI;
      for(int j=0;j<num_w;++j)
      {
        w+=dw;
        cw = cos(w);
        sw = sin(w);
        PointT p;
        p.x = params_.a1 *(params_.a4+ pow(fabs(cn), params_.e1) )* pow (fabs(cw), params_.e2);
        p.y = params_.a2 *(params_.a4+pow(fabs(cn), params_.e1)) * pow (fabs(sw), params_.e2);
        p.z = params_.a3 * pow(fabs(sn), params_.e1);
        p.r =  r_*255;
        p.g = g_*255;
        p.b = b_*255;

        if(cn*cw <0){p.x = -p.x;}
        if(cn*sw <0){p.y = -p.y;}
        if(sn<0){p.z = -p.z;}
        cloud->points.push_back(p);
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = true;
      }
    }
    transformCloud(cloud, cloud_);
}
