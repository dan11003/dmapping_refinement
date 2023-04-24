#pragma once
#ifndef TEST_H
#define TEST_H
#include "ceres/autodiff_cost_function.h"

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

#include <ceres/loss_function.h>


using namespace Eigen;
using namespace std;

namespace ICP_Ceres {

struct testAngAx{

    const Eigen::Vector3d p_dst;
    const Eigen::Vector3d p_src;
    const double scale_;


    testAngAx(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const double scale) :
    p_dst(dst), p_src(src), scale_(scale)
    {

    }
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const double t_target) {
        return (new ceres::AutoDiffCostFunction<testAngAx, 3, 4, 3>(new testAngAx(dst, src, t_target)));
    }
    template <typename T>
    bool operator()(const T* const src_camera_rot, const T* const src_camera_trans,  T* residuals) const {


        //const Eigen::Matrix<T,3,1> t_src(src_camera_trans);

/*
        T p_dst_trans[3];
        T src_rot_cam[3] = {T(scale_)*src_camera_rot[0], T(scale_)*src_camera_rot[1], T(scale_)*src_camera_rot[2]};
        T p_src_transf[3] = {T(p_src(0)), T(p_src(1)), T(p_src(2)) };
        const T p_dst_raw[3] = {T(p_dst(0)), T(p_dst(1)), T(p_dst(2)) };


        ceres::AngleAxisRotatePoint(src_rot_cam, p_dst_raw, p_dst_trans); //rotated*/

        const Eigen::Matrix<T,3,1> p_src_compensated = p_src.cast<T>();
        const Eigen::Quaternion<T> q_src(src_camera_rot);
        const Eigen::Matrix<T,3,1> t_src(src_camera_trans);
        const Eigen::Matrix<T,3,1> p_src_transformed = q_src*p_src_compensated + t_src;

        const Eigen::Matrix<T,3,1> p_dst_transformed = p_dst.cast<T>();


        residuals[0] = p_src_transformed[0] - p_dst_transformed[0];
        residuals[1] = p_src_transformed[1] - p_dst_transformed[1];
        residuals[2] = p_src_transformed[2] - p_dst_transformed[2];

        return true;
    }
};
}
#endif // TEST_H
