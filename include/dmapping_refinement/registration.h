#ifndef REGISTRATION_H
#define REGISTRATION_H
#include "lio_sam/generics.h"
#include "memory.h"
#include <map>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "odomEstimationClass.h"

namespace dmapping {

typedef struct{
    int src_scan;
    int src_idx;
    int target_scan;
    int target_idx;
}Correspondance;


class NScanRefinement
{
public:
  struct Parameters
  {
    int outer_iterations = 1;
    int inner_iterations = 1;
  };

  NScanRefinement(Parameters& par, std::map<int,Eigen::Isometry3d>& poses, std::map<int,NormalCloud::Ptr>& surf);

  void Solve();

private:

  std::map<int,int> AssociateScanPairsLogN();

  std::vector<Correspondance> FindCorrespondences(const int scan_i, const int scan_j);

  void addSurfCostFactor(const Correspondance& correspondance, ceres::Problem& problem);

  void TransformCommonFrame();




  std::map<int,Eigen::Isometry3d> poses_;
  std::map<int,NormalCloud::Ptr> surf_;
  std::map<int,NormalCloud::Ptr> filtered_;
  std::map<int,Eigen::MatrixXf> filtered_eig_;
  std::map<int,NormalCloud::Ptr> transformed_;
  std::map<int,pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr> kdtree_;
  std::map<int, Eigen::Isometry3d> poses;
  std::map<int,std::vector<double> > optimization_parameters; // 7dof

  Parameters par_;
  ceres::LossFunction *loss_function;
  ceres::Problem problem;
  ceres::Solver::Options options;
};

void GetParameters(std::map<int,Eigen::Isometry3d>& parameters, const boost::shared_ptr<PoseGraph> graph);

void SetParameters(const std::map<int,Eigen::Isometry3d>& parameters, boost::shared_ptr<PoseGraph> graph);


}

#endif // REGISTRATION_H
