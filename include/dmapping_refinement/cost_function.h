#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H
#include "ceres/autodiff_cost_function.h"

struct PointToPlaneErrorGlobal{

    const Eigen::Vector3d p_dst;
    const Eigen::Vector3d p_src;
    const Eigen::Vector3d p_nor;


    PointToPlaneErrorGlobal(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobal, 1, 4, 3, 4, 3>(new PointToPlaneErrorGlobal(dst, src, nor)));
    }

    template <typename T>
    bool operator()(const T* const src_camera_rot, const T* const src_camera_trans, const T* const dst_camera_rot, const T* const dst_camera_trans, T* residuals) const {

        const Eigen::Quaternion<T> q_dst(dst_camera_rot);
        const Eigen::Matrix<T,3,1> t_dst(dst_camera_trans);

        const Eigen::Quaternion<T> q_src(src_camera_rot);
        const Eigen::Matrix<T,3,1> t_src(src_camera_trans);

        const Eigen::Matrix<T,3,1> p_src_transf = q_src*p_src.cast<T>() + t_src;
        const Eigen::Matrix<T,3,1> p_dst_transf = q_dst*p_dst.cast<T>() + t_dst;
        const Eigen::Matrix<T,3,1> normal_dst_transf = q_dst*p_nor.cast<T>();
        residuals[0] = (p_src_transf - p_dst_transf).dot(normal_dst_transf);

        return true;
    }
};


struct PointToPlaneErrorGlobalTime{

    const Eigen::Vector3d p_dst;
    const Eigen::Vector3d p_src;
    const Eigen::Vector3d p_nor;
    const double t_src_;
    const double t_target_;


    PointToPlaneErrorGlobalTime(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor, const double t_src, const double t_target) :
    p_dst(dst), p_src(src), p_nor(nor), t_target_(t_target), t_src_(t_src)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor, const double t_src, const double t_target) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobalTime, 1, 4, 3, 4, 3, 3, 3>(new PointToPlaneErrorGlobalTime(dst, src, nor, t_src, t_target)));
    }

    template <typename T>
    bool operator()(const T* const src_camera_rot, const T* const src_camera_trans, const T* const dst_camera_rot, const T* const dst_camera_trans, const T* const velocity_src, const T* const velocity_dst, T* residuals) const {

        const Eigen::Quaternion<T> q_dst(dst_camera_rot);
        const Eigen::Matrix<T,3,1> t_dst(dst_camera_trans);

        const Eigen::Quaternion<T> q_src(src_camera_rot);
        const Eigen::Matrix<T,3,1> t_src(src_camera_trans);

        const Eigen::Matrix<T,3,1> v_src(v_src);
        const Eigen::Matrix<T,3,1> v_dst(v_dst);


        const Eigen::Matrix<T,3,1> p_src_err(v_src*T(t_src_));
        const Eigen::Matrix<T,3,1> p_dst_err(v_dst*T(t_target_));

        const Eigen::Matrix<T,3,1> p_src_transf = q_src*(p_src.cast<T>() + p_src_err) + t_src;
        const Eigen::Matrix<T,3,1> p_dst_transf = q_dst*(p_dst.cast<T>() + p_dst_err) + t_dst;
        const Eigen::Matrix<T,3,1> normal_dst_transf = q_dst*p_nor.cast<T>();
        residuals[0] = (p_src_transf - p_dst_transf).dot(normal_dst_transf);

        return true;
    }
};

#endif // COST_FUNCTION_H
