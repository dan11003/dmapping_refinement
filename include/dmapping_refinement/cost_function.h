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


struct planarCostFunction{

    const Eigen::Vector3d normal_;

    planarCostFunction(const Eigen::Vector3d& nor) :normal_(nor)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& nor) {
        return (new ceres::AutoDiffCostFunction<planarCostFunction, 2, 4>(new planarCostFunction(nor)));
    }

    template <typename T>
    bool operator()(const T* const camera_rot, T* residuals) const {

        const Eigen::Quaternion<T> q(camera_rot);
        const Eigen::Matrix<T,3,1> normalTransformed = q*normal_.cast<T>();
        Eigen::Matrix<T,3,1> groundNormalZ;
        groundNormalZ << T(0.0),T(0.0),T(1.0);
        Eigen::Matrix<T,3,1> groundNormalX;
        groundNormalX << T(1.0),T(0.0),T(0.0);

        const T simZ = normalTransformed.dot(groundNormalZ);
        const T simX = normalTransformed.dot(groundNormalX);
        const T dot45Deg(0.707);

        if(simZ > dot45Deg){
          residuals[0] = T(1.0) - simZ; // ground or ceiling observation -  want to maximize similarity
        }
        else{
          residuals[0] = simZ; // Wall observation - want to minimize similarity
        }

        if(simX > dot45Deg){
          residuals[1] = T(1.0) - simX; // ground or ceiling observation -  want to maximize similarity
        }
        else{
          residuals[1] = simX; // Wall observation - want to minimize similarity
        }

        return true;
    }
};




struct PointToPlaneErrorGlobalP2P{

    const Eigen::Vector3d p_dst;
    const Eigen::Vector3d p_src;
    const Eigen::Vector3d p_nor;


    PointToPlaneErrorGlobalP2P(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobalP2P, 3, 4, 3, 4, 3>(new PointToPlaneErrorGlobalP2P(dst, src, nor)));
    }

    template <typename T>
    bool operator()(const T* const src_camera_rot, const T* const src_camera_trans, const T* const dst_camera_rot, const T* const dst_camera_trans, T* residuals) const {

        const Eigen::Quaternion<T> q_dst(dst_camera_rot);
        const Eigen::Matrix<T,3,1> t_dst(dst_camera_trans);

        const Eigen::Quaternion<T> q_src(src_camera_rot);
        const Eigen::Matrix<T,3,1> t_src(src_camera_trans);

        const Eigen::Matrix<T,3,1> p_src_transf = q_src*p_src.cast<T>() + t_src;
        const Eigen::Matrix<T,3,1> p_dst_transf = q_dst*p_dst.cast<T>() + t_dst;
        residuals[0] = p_src_transf(0) - p_dst_transf(0);
        residuals[1] = p_src_transf(1) - p_dst_transf(1);
        residuals[2] = p_src_transf(2) - p_dst_transf(2);

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
    p_dst(dst), p_src(src), p_nor(nor), t_src_(t_src), t_target_(t_target)
    {
//        cout<<nor.dot(nor)<<endl;
    }

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor, const double t_src, const double t_target) {
        //return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobalTime, 1, 4, 3, 4, 3, /*vel*/3, 3 /*RotVel*//*,  3, 3*/>(new PointToPlaneErrorGlobalTime(dst, src, nor, t_src, t_target)));
        return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobalTime, 1, 4, 3, 4, 3, /*vel*/3, 3 /*RotVel*/, 3, 3>(new PointToPlaneErrorGlobalTime(dst, src, nor, t_src, t_target)));
    }


    template <typename T>
      static Eigen::Matrix<T, 3, 1> Compensate(const Eigen::Matrix<T,3,1>& p, const T* const cam_rot_vel, const T* const cam_vel, const T t_scale) {
        const Eigen::Matrix<T,3,1> p_cam_vel(cam_vel);
        T p_vel_comp[3] = {p(0) + p_cam_vel(0)*t_scale, p(1) + p_cam_vel(1)*t_scale, p(2) + p_cam_vel(2)*t_scale};
        T camAxis[3] = {t_scale*cam_rot_vel[0], t_scale*cam_rot_vel[1], t_scale*cam_rot_vel[2]};
        T p_rot_comp[3];
        ceres::AngleAxisRotatePoint(camAxis, p_vel_comp, p_rot_comp); //rotated*/
        const Eigen::Matrix<T,3,1> p_compensated(p_rot_comp[0], p_rot_comp[1], p_rot_comp[2]);
        return p_compensated;
      }

    template <typename T>
    bool operator()(const T* const src_camera_rot,
                    const T* const src_camera_trans,
                    const T* const dst_camera_rot,
                    const T* const dst_camera_trans,
                    const T* const velocity_src,
                    const T* const velocity_dst,
                    const T* const rotVelocity_src,
                    const T* const rotVelocity_dst,
                    T* residuals) const {

        const Eigen::Quaternion<T> q_dst(dst_camera_rot);
        const Eigen::Matrix<T,3,1> t_dst(dst_camera_trans);

        const Eigen::Quaternion<T> q_src(src_camera_rot);
        const Eigen::Matrix<T,3,1> t_src(src_camera_trans);

        const Eigen::Matrix<T,3,1> v_src(velocity_src);
        const Eigen::Matrix<T,3,1> v_dst(velocity_dst);

        //const Eigen::Quaternion<T> rad_w_dst(rad_vel_dst);

        const Eigen::Matrix<T,3,1> p_src_jet = p_src.cast<T>();
        const Eigen::Matrix<T,3,1> p_dst_jet = p_dst.cast<T>();

        const Eigen::Matrix<T,3,1> p_src_compensated = Compensate(p_src_jet, rotVelocity_src, velocity_src, T(t_src_));
        const Eigen::Matrix<T,3,1> p_tar_compensated = Compensate(p_dst_jet, rotVelocity_dst, velocity_dst, T(t_target_));
        //const Eigen::Matrix<T,3,1> p_src_err(v_src*T(t_src_));
        //const Eigen::Matrix<T,3,1> p_dst_err(v_dst*T(t_target_));

        const Eigen::Matrix<T,3,1> p_src_transf = q_src*(p_src_compensated) + t_src;
        const Eigen::Matrix<T,3,1> p_dst_transf = q_dst*(p_tar_compensated) + t_dst;
        const Eigen::Matrix<T,3,1> normal_dst_transf = q_dst*p_nor.cast<T>(); // this is approximate, consider deskewing normal as well
        residuals[0] = (p_src_transf - p_dst_transf).dot(normal_dst_transf);

        return true;
    }

    /*template <typename T>
    bool operator()(const T* const src_camera_rot,
                    const T* const src_camera_trans,
                    const T* const dst_camera_rot,
                    const T* const dst_camera_trans,
                    const T* const velocity_src,
                    const T* const velocity_dst,
                    T* residuals) const {

        const Eigen::Quaternion<T> q_dst(dst_camera_rot);
        const Eigen::Matrix<T,3,1> t_dst(dst_camera_trans);

        const Eigen::Quaternion<T> q_src(src_camera_rot);
        const Eigen::Matrix<T,3,1> t_src(src_camera_trans);

        const Eigen::Matrix<T,3,1> v_src(velocity_src);
        const Eigen::Matrix<T,3,1> v_dst(velocity_dst);

        //const Eigen::Quaternion<T> rad_w_dst(rad_vel_dst);

        const Eigen::Matrix<T,3,1> p_src_jet = p_src.cast<T>();
        const Eigen::Matrix<T,3,1> p_dst_jet = p_dst.cast<T>();

        const Eigen::Matrix<T,3,1> p_src_compensated = p_src_jet + Eigen::Matrix<T,3,1>(v_src*T(t_src_));
        const Eigen::Matrix<T,3,1> p_dst_compensated = p_dst_jet + Eigen::Matrix<T,3,1>(v_dst*T(t_target_));

        const Eigen::Matrix<T,3,1> p_src_transf = q_src*(p_src_compensated) + t_src;
        const Eigen::Matrix<T,3,1> p_dst_transf = q_dst*(p_dst_compensated) + t_dst;
        const Eigen::Matrix<T,3,1> normal_dst_transf = q_dst*p_nor.cast<T>(); // this is approximate, consider deskewing normal as well
        residuals[0] = (p_src_transf - p_dst_transf).dot(normal_dst_transf);

        return true;
    }*/

};

struct VelocityConstraint{


    VelocityConstraint(const double scaling_factor) : scaling_factor_(scaling_factor) {}

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const double scaling_factor) {
        return (new ceres::AutoDiffCostFunction<VelocityConstraint,3, 3, 3, 3>(new VelocityConstraint(scaling_factor)));
    }

    template <typename T>
    bool operator()(const T* const t_now_par, const T* const v_now_par, const T* const t_next_par, T* residuals) const {


        const Eigen::Matrix<T,3,1> t_now(t_now_par);
        const Eigen::Matrix<T,3,1> v_now(v_now_par);
        const Eigen::Matrix<T,3,1> t_next(t_next_par);
        const T scaling = T(scaling_factor_);

        const Eigen::Matrix<T,3,1> err = t_now+v_now-t_next;
        residuals[0] = scaling*err[0];
        residuals[1] = scaling*err[1];
        residuals[2] = scaling*err[2];

        return true;
    }
    double scaling_factor_;
};

#endif // COST_FUNCTION_H
