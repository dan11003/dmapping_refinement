#ifndef D_UTILS_H
#define D_UTILS_H
#include "lio_sam/generics.h"
#include "eigen_conversions/eigen_kdl.h"
#include "tf_conversions/tf_eigen.h"
#include "ceres/ceres.h"
#include "ceres/autodiff_cost_function.h"


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



//Runtime: 72 ms, faster than 99.56% of C++ online submissions for Find K Closest Elements.
//Memory Usage: 31 MB, less than 16.67% of C++ online submissions for Find K Closest Elements.
class NNSearchArray {
public:
    std::vector<int> findClosestElements(std::vector<double>& arr, int k, float max, float query);
};

class SurfelExtraction
{

public:
    SurfelExtraction(pcl::PointCloud<PointType>::Ptr& surf_in, int n_scan_lines, float scan_period);

    void Extract(SurfElCloud& surfelCloud);

private:

    void LineNNSearch( const int ring, const double query, int &row, Eigen::MatrixXd& neighbours);

    bool GetNeighbours(const vel_point::PointXYZIRTC& pnt, Eigen::MatrixXd& neighbours);

    bool EstimateNormal(const vel_point::PointXYZIRTC& pnt, SurfelPointInfo& surfEl);

    void Initialize();

    pcl::PointCloud<PointType>::Ptr surf_in_;
    std::vector<pcl::PointCloud<PointType>::Ptr> ringClouds_; //sorted in time, and segmented per ring
    std::vector<std::vector<double> > times_;

    pcl::PointXYZINormal defaultNormal;
    int n_scan_lines_;
    float scan_period_;
    int nr_neighbours = 2;

};

class lineRefinement{
    lineRefinement(const pcl::PointCloud<PointType>::Ptr input) : input_(input) {}

    void Filter(pcl::PointCloud<PointType>::Ptr output, int nr_neigh = 3, double expected_noise = 0.03);

    bool Fit(const Eigen::Matrix<double,3,1>& X, Eigen::Matrix<double,3,1>& p0, Eigen::Matrix<double,3,1>& v);

    const pcl::PointCloud<PointType>::Ptr input_;
};

class RotErrorTerm {
 public:
  RotErrorTerm(Eigen::Quaterniond q_a_measured,
                       Eigen::Matrix<double, 3, 3> sqrt_information)
      : q_a_measured_inv_(q_a_measured.inverse()),
        sqrt_information_(std::move(sqrt_information)) {}
  template <typename T>
  bool operator()(const T* const q_a_ptr,
                  const T* const q_new_calib_ptr,
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_new_calib(q_new_calib_ptr);

    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<T> delta_q = q_a_measured_inv_.template cast<T>() * q_a*;
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) = T(2.0) * delta_q.vec();
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
    return true;
  }
  static ceres::CostFunction* Create(
      const Eigen::Quaterniond& q_a_measured,
      const Eigen::Matrix<double, 3, 3>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<RotErrorTerm,3, 4>(
        new RotErrorTerm(q_a_measured, sqrt_information));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  // The measurement for the position of B relative to A in the A frame.
  const Eigen::Quaterniond q_a_measured_inv_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 3, 3> sqrt_information_;
};

/*struct LineCostFunctor {
    LineCostFunctor(const Eigen::Matrix<double,3,1> pnt) : pnt_(pnt){
        unit_ << 1, 0, 0;
    }
    static ceres::CostFunction* Create(const Eigen::Matrix<double,3,1>& pnt) {
        return (new ceres::AutoDiffCostFunction<LineCostFunctor, 1, 4, 3>(new LineCostFunctor(pnt)));
    }

    template <typename T>
    bool operator()(const T* const qv_in, const T* const p0_in, T* residual) const {
        const Eigen::Matrix<T,3,1> p = pnt_.template cast<T>();

        const Eigen::Quaternion<T> v(qv_in*unit_.template cast<T>());

        const Eigen::Matrix<T,3,1> p0(p0_in);
        //const T ceres::DotProduct( (p - p0).transpose(),v);
        residual[0] = T(0); //(p - p0).dot(v) - T(1.0);
        return true;
    }
    Eigen::Matrix<double,3,1> pnt_;
    Eigen::Matrix<double,3,1> unit_;
};
*/

}
#endif // D_UTILS_H
