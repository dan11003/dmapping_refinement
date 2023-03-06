#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H
#include "ceres/autodiff_cost_function.h"

struct PointToPlaneErrorGlobal{

    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


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

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        /*Eigen::Matrix<T,3,1> src; src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> dst; dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
        Eigen::Matrix<T,3,1> nor; nor << T(p_nor[0]), T(p_nor[1]), T(p_nor[2]);

        // Map the T* array to an Eigen Quaternion object (with appropriate Scalar type)
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation);
        Eigen::Quaternion<T> qDst = Eigen::Map<const Eigen::Quaternion<T> >(camera_rotation_dst);

        // Map T* to Eigen Vector3 with correct Scalar type
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation);
        Eigen::Matrix<T,3,1> tDst = Eigen::Map<const Eigen::Matrix<T,3,1> >(camera_translation_dst);

        // Rotate the point using Eigen rotations
        Eigen::Matrix<T,3,1> p = q * src;
        p += t;
        Eigen::Matrix<T,3,1> p2 = qDst.toRotationMatrix() * dst;
        p2 += tDst;98
        Eigen::Matrix<T,3,1> n2 = qDst.toRotationMatrix() * nor; //no translation on normal

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p - p2).dot(n2) + T(0.1);*/
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

#endif // COST_FUNCTION_H
