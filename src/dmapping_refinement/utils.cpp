
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
    Eigen::Isometry3d Tprev;
    //cout <<"keyframe_min_transl: "<< keyframe_min_transl << ", keyframe_min_rot:" << keyframe_min_rot << ", max_size: " << max_size << endl;
    for(auto itr = std::next(input->nodes.begin()) ; itr != input->nodes.end() ; itr++){
        const Eigen::Isometry3d Tnow = itr->second.T;
        const size_t idx = itr->first;

        if(itr == std::next(input->nodes.begin())){ // if first iteration
             Tprev = Tnow;
             output->nodes[idx] = itr->second;
             output->surfels_[idx] = input->surfels_[idx];
             continue;
        }else{ // if another iteration
            const Eigen::Isometry3d delta =  Tprev.inverse()*Tnow;
            if(KeyFrameUpdate(delta, keyframe_min_transl, keyframe_min_rot)){
                Tprev = Tnow;
                output->nodes[idx] = itr->second;
                output->surfels_[idx] = input->surfels_[idx];
            }
        }
        if(output->nodes.size() == max_size){
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

NormalCloud::Ptr TransformNonRigid(NormalCloud::Ptr cloud,
                       const Eigen::Vector3d& t,
                       const Eigen::Quaterniond& q,
                       const Eigen::Vector3d& t_prim,
                       const Eigen::Quaterniond& q_prim,
                       const std::vector<double>& stamp){
    NormalCloud::Ptr transformed(new NormalCloud());
    for(int i = 0 ; i < cloud->size() ; i++){
        const double  time = stamp[i];
        const Eigen::Vector3d pnt = PntToEig(cloud->points[i]);
        const Eigen::Vector3d normal = NormalToEig(cloud->points[i]);
        const Eigen::Vector3d v_comp = t_prim*time;
        //const Eigen::Vector3d q_comp = velocities_[idx].q*time;
        const Eigen::Quaterniond qIdentity = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond qNlerp;
        if( time > 0){
            qNlerp = qIdentity.slerp(time, q_prim);
        }else{
            Eigen::Quaterniond q_prim_inv = q_prim.inverse();
            qNlerp = qIdentity.slerp(-time, q_prim_inv);
        }
        //qNlerp.normalize();
        const Eigen::Vector3d pnt_comp = qNlerp*pnt + v_comp;
        const Eigen::Vector3d normal_comp = qNlerp*normal; // add motion compensation


        const Eigen::Vector3d p_transformed = q*pnt_comp + t; // rigid transform
        const Eigen::Vector3d n_transformed = q*normal_comp;// rigid transform
        transformed->push_back(EigToPnt(p_transformed, n_transformed, cloud->points[i].intensity));
    }
    return transformed;
}


}
