#include<supertoroid/st_fitting.h>


int Fitting::OptimizationFunctor::operator ()(const Eigen::VectorXd &xvec, Eigen::VectorXd &fvec) const{
  pcl::PointCloud<PointT>::Ptr cloud_new(new pcl::PointCloud<PointT>);
  cloud_new = estimator_->prealigned_cloud_;
  double a = xvec[0], b = xvec[1], c = xvec[2], d = xvec[3], e1 = xvec[4], e2 = xvec[5];
  Eigen::Affine3d trans;
  create_transformation_matrix(xvec[6], xvec[7], xvec[8], xvec[9], xvec[10], xvec[11], trans);
  pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud_new, *cloud_transformed, trans);
  for(int i=0;i<values();++i)
  {
    Eigen::Matrix<double , 4, 1>xyz_tr(cloud_transformed->at(i).x, cloud_transformed->at(i).y, cloud_transformed->at(i).z, 1.);
    double op = Eigen::Matrix<double, 3, 1>(xyz_tr[0], xyz_tr[1], xyz_tr[2]).norm();
    PointT p;
    p.x = xyz_tr[0];
    p.y = xyz_tr[1];
    p.z = xyz_tr[2];
    fvec[i] = op * st_function(xyz_tr[0], xyz_tr[1], xyz_tr[2], a,b,c,d,e1,e2);
  }
  return (0);
}
