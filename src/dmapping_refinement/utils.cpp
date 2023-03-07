
#include "dmapping_refinement/utils.h"

namespace dmapping {

bool KeyFrameUpdate(const Eigen::Isometry3d& delta, const double keyframe_min_transl, const double keyframe_min_rot){
  const double delta_movement = delta.translation().norm();
  const Eigen::Vector3d ea = delta.linear().eulerAngles(0,1,2);//Eigen::AngleAxisd(delta.linear()).angle();
  const double delta_rot = std::max(ea(0),ea(1)); // exclude yaw
  if(delta_movement > keyframe_min_transl || delta_rot > keyframe_min_rot) {
    return true;
  }else{
    return false;
  }
}

boost::shared_ptr<PoseGraph> KeyFrameFilter(boost::shared_ptr<PoseGraph> input, const double keyframe_min_transl, const double keyframe_min_rot, const int max_size){
  boost::shared_ptr<PoseGraph> output = boost::make_shared<PoseGraph>();
  Eigen::Isometry3d Tprev = input->nodes.begin()->second.T;
  output->AddNode(input->nodes.begin()->second,input->nodes.begin()->first);
  //cout <<"keyframe_min_transl: "<< keyframe_min_transl << ", keyframe_min_rot:" << keyframe_min_rot << ", max_size: " << max_size << endl;
  for(auto itr = std::next(input->nodes.begin()) ; itr != input->nodes.end() ; itr++){
    const Eigen::Isometry3d Tnow = itr->second.T;
    const Eigen::Isometry3d delta =  Tprev.inverse()*Tnow;
    if(KeyFrameUpdate(delta, keyframe_min_transl, keyframe_min_rot)){
      Tprev = Tnow;
      output->AddNode(itr->second,itr->first);
    }
    if(output->nodes.size() == max_size){
        //cout << output->nodes.size() << endl;
        break;
    }
  }
  output->constraints = input->constraints;
  cout << "keyframe_min_transl: " << keyframe_min_transl << endl;
  cout << "keyframe_min_rot: " << keyframe_min_rot << endl;
  cout << "Before filter graph: " <<input->ToString() << endl;
  cout << "After filter graph: " <<output->ToString() << endl;
  return output;
}





}
