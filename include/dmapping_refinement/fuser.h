#include "lio_sam/generics.h"
#include "dmapping_refinement/utils.h"
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

    void GetParametersFromRos(ros::NodeHandle& nh){
      nh.getParam("/keyframe_min_transl", keyframe_min_transl);
      nh.getParam("/keyframe_min_rot", keyframe_min_rot);
      nh.getParam("/use_keyframe", keyframe_min_transl);
      nh.param<double>("/scan_period", lidar_param.scan_period, 0.1);
      nh.param<double>("/vertical_angle", lidar_param.horizontal_angle, 2);
      nh.param<double>("/max_dis", lidar_param.max_distance, 130);
      nh.param<double>("/min_dis", lidar_param.min_distance, 0);
      nh.param<int>("/scan_line", lidar_param.num_lines, 16);
      //nh.getParam("/map_resolution", map_resolution);
    }
  };

  // Paremeters
  Parameters par_;
  boost::shared_ptr<PoseGraph> graph_;
  std::map<int,SurfElCloud> surfels_;
  std::map<int,NormalCloud::Ptr> surf_;


  // Methods
  Fuser(Parameters& par, boost::shared_ptr<PoseGraph> graph);

  virtual void Run();

  virtual void Optimize();

  void Visualize();


};

}
