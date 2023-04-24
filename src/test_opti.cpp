

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
//#include "odomEstimationClass.h"
#include "fstream"
#include "iostream"
#include "stdio.h"
#include "string.h"

#include <omp.h>
#include "dmapping_refinement/test.h"

#include "memory.h"
#include <map>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "eigen_conversions/eigen_kdl.h"
#include "tf_conversions/tf_eigen.h"


Eigen::Isometry3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez) {
    return Eigen::Translation<double, 3>(x, y, z) *
            Eigen::AngleAxis<double>(ex, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxis<double>(ey, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<double>(ez, Eigen::Vector3d::UnitZ());
}

Eigen::Quaterniond euler2Quaternion( const double roll, const double pitch, const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "refinement");
    ros::NodeHandle nh("~");

    Eigen::Vector3d pdst(0,1,0);
    Eigen::Vector3d psrc(1,0,0);



    ceres::Problem problem;
    ceres::Solver::Options problem_options;
    ceres::Solver::Summary summary;


    // Simulated velocity
    const Eigen::Vector3d transVel(0, 0, 0);
    const std::vector<double> rotVel{0, 0, 0};
    // Simulated pose
    const Eigen::Quaterniond rotRigid = Eigen::Quaterniond(Eigen::AngleAxis<double>(0.15, Eigen::Vector3d(0,0,1)));
    const Eigen::Vector3d transRigid(0.1, 0.2, 0.3);


    // Parameters velocity
    std::vector<double> estRotVel = {0,0,0};
    Eigen::Vector3d estVel(0,0,0);

    // Parameters pose
    Eigen::Quaterniond estRot = Eigen::Quaterniond::Identity();
    Eigen::Vector3d estTrans(0,0,0);

    Eigen::AngleAxisd d = Eigen::AngleAxisd::Identity();
    cout << d.angle() << endl;
    cout << d.axis().transpose() << endl;



    for(int i = -5 ; i <=5 ; i++){ // sweep
        const double t = ((double)i)/10.0;
        const Eigen::Quaterniond rotDist = euler2Quaternion(t*rotVel[0], t*rotVel[1], t*rotVel[2]).normalized();
        const Eigen::Vector3d transDist = transVel*t;
        Eigen::Vector3d target(i, i, 0);
        Eigen::Vector3d src = rotRigid*(/*rotDist**/target /*+ transDist*/) + transRigid;
        ceres::CostFunction* cost_function = ICP_Ceres::testAngAx::Create(target, src, t);
        cout << "tar: " << target.transpose() << endl;
        cout << "src: " << src.transpose() << endl << endl;


        problem.AddResidualBlock(cost_function, nullptr, estRot.coeffs().data(), estTrans.data());
    }

    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization();
    problem.SetParameterization(estRot.coeffs().data(), quaternion_local_parameterization);

    problem_options.minimizer_progress_to_stdout = true;
    ceres::Solve(problem_options, &problem, &summary);
    cout << summary.FullReport() << endl;
    cout << "result pos: " << estTrans.transpose() << endl;
    cout << "result orient: " << estRot.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << endl;


    ros::spin();

    return 0;
}


