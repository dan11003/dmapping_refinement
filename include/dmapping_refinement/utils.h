#ifndef UTILS_H
#define UTILS_H
#include "lio_sam/generics.h"
#include "eigen_conversions/eigen_kdl.h"
#include "tf_conversions/tf_eigen.h"

namespace dmapping{


boost::shared_ptr<PoseGraph> KeyFrameFilter(boost::shared_ptr<PoseGraph> input, const double keyframe_min_transl, const double keyframe_min_rot);

bool KeyFrameUpdate(const Eigen::Isometry3d& delta, const double keyframe_min_transl, const double keyframe_min_rot);


}



#endif // UTILS_H
