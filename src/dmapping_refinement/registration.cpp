#include "dmapping_refinement/registration.h"

namespace dmapping {

NScanRefinement::NScanRefinement(Parameters& par, std::map<int,Eigen::Isometry3d>& poses, std::map<int,NormalCloud::Ptr>& surf) : par_(par), poses_(poses), surf_(surf){
   loss_function = new ceres::HuberLoss(0.1);
   options.linear_solver_type = ceres::DENSE_QR;
   options.max_num_iterations = par_.inner_iterations;
   options.minimizer_progress_to_stdout = true;
   for(auto itr = poses.begin() ; itr != poses.end() ; itr++){
     const size_t idx = itr->first;
     NormalCloud::Ptr tmp_filtered(new NormalCloud());
     pcl::VoxelGrid<pcl::PointXYZINormal> sor;
     sor.setInputCloud(surf_[idx]);
     sor.setLeafSize (0.05f, 0.05f, 0.05f);
     sor.filter (*tmp_filtered);
     filtered_[idx] = tmp_filtered;
     filtered_eig_[idx] = tmp_filtered->getMatrixXfMap();
     cout << "Expect  N x 7..."<< endl;
     cout << filtered_eig_[idx].rows() << " x " << filtered_eig_[idx].cols() << endl;
   }
}


std::map<int,int> NScanRefinement::AssociateScanPairsLogN(){
  std::map<int,int> scan_pairs;
  for(auto itr_from = poses_.begin() ; itr_from != std::prev(poses_.end()) ; itr_from++){
    for(auto itr_to = std::next(itr_from) ; itr_to != poses_.end() ; itr_to++){
      //log n instead of n
      if(std::distance(itr_from,itr_to) % 2 ==0 || std::distance(itr_from,itr_to) == 1){ // 1 2 4 8
        scan_pairs[itr_from->first] = itr_to->first;
      }
    }
  }
  cout << "scan pairs: " << scan_pairs.size() << endl;
}

std::vector<Correspondance> NScanRefinement::FindCorrespondences(const int scan_i, const int scan_j){
  return std::vector<Correspondance>();
}
void NScanRefinement::TransformCommonFrame(){
  //std::vector<SurfElCloud> surfels_;
  for(auto itr = filtered_.begin() ; itr != filtered_.end() ; itr++){
    const size_t idx = itr->first;
    NormalCloud::Ptr tmp_transformed(new NormalCloud());
    pcl::transformPointCloudWithNormals(*filtered_[idx], *tmp_transformed, poses[idx].matrix());
    transformed_[idx] = tmp_transformed;
    kdtree_[idx] = pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
  }
}

void NScanRefinement::addSurfCostFactor(const Correspondance& correspondance, ceres::Problem& problem){
  //ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
  //problem.AddResidualBlock(cost_function, loss_function, parameters);
}


void NScanRefinement::Solve(){

  for (int i = 0; i < par_.inner_iterations ; i++) {

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    //problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization()); repetera för alla

    TransformCommonFrame();
    std::map<int,int> scan_pairs = AssociateScanPairsLogN();
    std::vector<Correspondance> correspondances;
    for (auto& [scan_i, scan_j]: scan_pairs) {
      std::vector<Correspondance> tmp_corr = FindCorrespondences(scan_i, scan_j);
      correspondances.insert(std::end(correspondances), std::begin(tmp_corr), end(tmp_corr));
    }
    for(auto && c : correspondances){
      addSurfCostFactor(c, problem);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
  }
}

void GetParameters(std::map<int,Eigen::Isometry3d>& parameters, const boost::shared_ptr<PoseGraph> graph){
  parameters.clear();
  for (const auto& [index, node]: graph->nodes) {
    parameters[index] = node.T;
  }
}

void SetParameters(const std::map<int,Eigen::Isometry3d>& parameters, boost::shared_ptr<PoseGraph> graph){
  for (const auto& [index, parameter]: parameters) {
    graph->nodes[index].T = parameter;
  }
}

}
