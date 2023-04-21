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
#include "dmapping_refinement/d_utils.h"
#include <pcl/filters/normal_refinement.h>
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
    int outer_iterations = 5;
    int inner_iterations = 5;
    double  max_dist_association = 1;
    std::string loss = "cauchy";
    float resolution = 0.1;
    bool estimate_velocity = true;

    //std::string toString() {return}
  };
  struct Pose3d
  {
      Eigen::Vector3d p;
      Eigen::AngleAxisd q;
      static const Pose3d Identity(){return {Eigen::Vector3d::Zero() , Eigen::AngleAxisd::Identity()};}
  };


  NScanRefinement(Parameters& par, const std::map<int,Pose3d>& poses, std::map<int,NormalCloud::Ptr>& surf, std::map<int,Eigen::Quaterniond>& imu, ros::NodeHandle& nh);

  void Solve(std::map<int,Pose3d>& solutionPose, std::map<int,Pose3d>& solutionVel);

  void Solve(std::map<int,Pose3d>& solutionPose, std::map<int,Pose3d>& solutionVel, const std::map<int,bool>& locked);

  void GetPointCloudsSurfTransformed(std::map<int,NormalCloud::Ptr>& output);



  //ceres::Problem problem;
  ceres::Problem::Options problem_options;
  ceres::Solver::Summary summary;


private:

  std::vector<std::pair<int,int> > AssociateScanPairsLogN();

  std::vector<Correspondance> FindCorrespondences(const int scan_i, const int scan_j);

  void addSurfCostFactor(const Correspondance& c, ceres::Problem& problem);

  void AddRotationTerm(int idx);

  void TransformCommonFrame(const std::map<int,NormalCloud::Ptr>& input, std::map<int,NormalCloud::Ptr>& output, const bool compute_kdtree);

  void Visualize(const std::string& topic);

  void VisualizeCorrespondance(std::vector<Correspondance>& corr);

  int nr_residual = 0;

  /* InpUT */
  Parameters par_;
  std::map<int,Pose3d> poses_;
  std::map<int,NormalCloud::Ptr> surf_;
  //std::map<int,std::vector<double> > stamps_;
  std::map<int,Eigen::Quaterniond > imu_;
  ros::NodeHandle& nh_;
  std::map<int,Pose3d> velocities_;
  std::map<int,Eigen::AngleAxisd> angularvelocity_;
  std::map<int,bool> locked_;

  std::map<int,NormalCloud::Ptr> filtered_;
  std::map<int,Eigen::MatrixXf> filtered_eig_;
  std::map<int,NormalCloud::Ptr> transformed_;
  std::map<int,std::vector<Eigen::Vector3d>> means_local_;
  std::map<int,std::vector<Eigen::Vector3d>> normals_local_;

  std::map<int,std::vector<Eigen::Vector3d>> means_transformed_;
  std::map<int,std::vector<Eigen::Vector3d>> normals_transformed_;

  std::map<int,pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr> kdtree_;
  std::map<int,std::vector<double> > optimization_parameters; // 7dof


  //ceres::LossFunction *loss_function;
  ceres::Solver::Options options;

  ros::Publisher vis_pub, normal_pub;
};

void NonRigidTransform(const NScanRefinement::Pose3d& vel, const Eigen::AngleAxisd rotVel, const NScanRefinement::Pose3d& pose, const NormalCloud::Ptr& input, NormalCloud::Ptr& output);

}


inline Eigen::Vector3d PntToEig(const pcl::PointXYZINormal& pnt){
    return Eigen::Vector3d(pnt.x, pnt.y, pnt.z);
}
inline pcl::PointXYZINormal EigToPnt(const Eigen::Vector3d& pnt, const Eigen::Vector3d& normal, const double intensity){
    pcl::PointXYZINormal p;
    p.x = pnt(0); p.y = pnt(1); p.z = pnt(2); p.intensity = intensity;
    p.normal_x = normal(0); p.normal_y = normal(1); p.normal_z = normal(2);
    return p;
}
inline Eigen::Vector3d NormalToEig(const pcl::PointXYZINormal& pnt){
   return Eigen::Vector3d(pnt.normal_x, pnt.normal_y, pnt.normal_z);
}


#endif // REGISTRATION_H
