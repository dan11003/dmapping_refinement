#ifndef UTILS_H
#define UTILS_H
#include "lio_sam/generics.h"
#include "eigen_conversions/eigen_kdl.h"
#include "tf_conversions/tf_eigen.h"

namespace dmapping{


boost::shared_ptr<PoseGraph> KeyFrameFilter(boost::shared_ptr<PoseGraph> input, const double keyframe_min_transl, const double keyframe_min_rot, const int  max_size = -1);

bool KeyFrameUpdate(const Eigen::Isometry3d& delta, const double keyframe_min_transl, const double keyframe_min_rot);

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

NormalCloud::Ptr TransformNonRigid(NormalCloud::Ptr cloud,
                       const Eigen::Vector3d& t,
                       const Eigen::Quaterniond& q,
                       const Eigen::Vector3d& t_prim,
                       const Eigen::Quaterniond& q_prim,
                       const std::vector<double>& stamp);


}



#endif // UTILS_H
