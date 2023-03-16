#ifndef REGISTRATION_H
#define REGISTRATION_H
#include "lio_sam/generics.h"
#include "memory.h"
#include <map>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "odomEstimationClass.h"
#include "eigen_conversions/eigen_kdl.h"
#include "tf_conversions/tf_eigen.h"
#include "dmapping_refinement/cost_function.h"
#include "dmapping_refinement/utils.h"
namespace dmapping {

typedef struct{
    int src_scan;
    int src_idx;
    int target_scan;
    int target_idx;
    double weight;
}Correspondance;


class NScanRefinement
{
public:
  struct Parameters
  {
    int outer_iterations = 1;
    int inner_iterations = 1;
    double  max_dist_association = 0.5;
  };
  struct Pose3d
  {
      Eigen::Vector3d p;
      Eigen::Quaterniond q;
  };

  NScanRefinement(Parameters& par, const std::map<int,Pose3d>& poses, std::map<int,NormalCloud::Ptr>& surf, std::map<int,std::vector<double> >& stamps);

  void Solve(std::map<int,Pose3d>& solution);

  ceres::Problem* problem;
  ceres::Problem::Options problem_options;
  ceres::Solver::Summary summary;



private:

  std::vector<std::pair<int,int> > AssociateScanPairsLogN();

  std::vector<Correspondance> FindCorrespondences(const int scan_i, const int scan_j);

  void addSurfCostFactor(const Correspondance& c, ceres::Problem& problem);

  void TransformCommonFrame();

  void Visualize();

  int nr_residual = 0;

  /* InpUT */
  Parameters par_;
  std::map<int,Pose3d> poses_;
  std::map<int,NormalCloud::Ptr> surf_;
  std::map<int,std::vector<double> > stamps_;
  std::map<int,Pose3d> velocities_;

  std::map<int,NormalCloud::Ptr> filtered_;
  std::map<int,Eigen::MatrixXf> filtered_eig_;
  std::map<int,NormalCloud::Ptr> transformed_;
  std::map<int,std::vector<Eigen::Vector3d>> means_local_;
  std::map<int,std::vector<Eigen::Vector3d>> normals_local_;

  std::map<int,std::vector<Eigen::Vector3d>> means_transformed_;
  std::map<int,std::vector<Eigen::Vector3d>> normals_transformed_;

  std::map<int,pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr> kdtree_;
  std::map<int,std::vector<double> > optimization_parameters; // 7dof


  ceres::LossFunction *loss_function;
  ceres::Solver::Options options;
};

void GetParameters(std::map<int,NScanRefinement::Pose3d>& parameters, const boost::shared_ptr<PoseGraph> graph);

void SetParameters(const std::map<int,NScanRefinement::Pose3d>& parameters, boost::shared_ptr<PoseGraph> graph);




}



#endif // REGISTRATION_H
