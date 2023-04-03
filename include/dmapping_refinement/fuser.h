#include "lio_sam/generics.h"
#include "dmapping_refinement/d_utils.h"
#include "lidar.h"
#include "odomEstimationClass.h"
#include "dmapping_refinement/registration.h"


namespace dmapping{

class Fuser{
public:
  //Type definitions

  struct Parameters
  {
    double keyframe_min_transl;
    double keyframe_min_rot;
    bool use_keyframe;
    lidar::Lidar lidar_param;
    NScanRefinement::Parameters reg_par;
    int tot_scans;

    void GetParametersFromRos(ros::NodeHandle& nh){
      nh.getParam("/keyframe_min_transl", keyframe_min_transl);
      nh.getParam("/keyframe_min_rot", keyframe_min_rot);
      nh.getParam("/use_keyframe", keyframe_min_transl);
      nh.param<double>("/scan_period", lidar_param.scan_period, 0.1);
      nh.param<double>("/vertical_angle", lidar_param.horizontal_angle, 2);
      nh.param<double>("/max_dis", lidar_param.max_distance, 130);
      nh.param<double>("/min_dis", lidar_param.min_distance, 0);
      nh.param<int>("/scan_line", lidar_param.num_lines, 16);
      nh.param<int>("/inner_iterations", reg_par.inner_iterations, 3);
      nh.param<int>("/outer_iterations", reg_par.outer_iterations, 2);
      nh.param<double>("/max_dist_association", reg_par.max_dist_association, 1.0);
      nh.param<int>("/tot_scans", tot_scans, 30);
      //nh.getParam("/map_resolution", map_resolution);
    }
  };

  // Paremeters
  Parameters par_;
  boost::shared_ptr<PoseGraph> graph_;
  std::map<int,SurfElCloud> surfels_;
  std::map<int,NormalCloud::Ptr> surf_;
  std::map<int,std::vector<double>> stamps_;
  std::map<int,Eigen::Quaterniond> imu_;
  ros::NodeHandle& nh_;


  // Methods
  Fuser(Parameters& par, boost::shared_ptr<PoseGraph> graph, ros::NodeHandle& nh);

  bool ComputeSurfels();

  virtual void Run();

  virtual void Optimize();

  void Visualize();

private:
  void GetParameters(std::map<int,NScanRefinement::Pose3d>& parameters);

  void SetParameters(const std::map<int,NScanRefinement::Pose3d>& parameters);


};



}
