
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

#include "eigen_conversions/eigen_kdl.h"
#include "tf_conversions/tf_eigen.h"
#include <dynamic_reconfigure/server.h>
#include "dmapping_refinement/calibParamsConfig.h"
#include "dmapping_refinement/dutils.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"


using std::endl;
using std::cout;



class calib{

public:
    Eigen::Affine3d Tcalib, Timu;
    ros::Publisher pub;
    tf::TransformBroadcaster Tbr;
    ros::NodeHandle nh_;
    ros::Subscriber subLaserCloud, subImu;
    double timeOffset;


    Eigen::Quaterniond euler2Quaternion( const double roll, const double pitch, const double yaw )
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        return q;
    }
    void CenterTime(const pcl::PointCloud<PointType>::Ptr cloud){
        ros::Time t;
        pcl_conversions::fromPCL(cloud->header.stamp,t);
        const double tScan = t.toSec();
        const double tEnd = tScan + cloud->points.back().time;
        const double tBegin = tScan + cloud->points.front().time;
        const double tCenter = tBegin + (tEnd - tBegin)/2.0;
        const ros::Time tRosCenter(tCenter);
        pcl_conversions::toPCL(tRosCenter, cloud->header.stamp);
        for(auto && pnt : cloud->points ){
            pnt.time = pnt.time + tScan - tCenter;
        }
        //std::cout << "adjust: " << tScan - tCenter << std::endl;
    }

    void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        pcl::PointCloud<PointType>::Ptr pointcloud_in(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*laserCloudMsg, *pointcloud_in);
        CenterTime(pointcloud_in);
        pointcloud_in->header.frame_id = "velodyneLink";
        pub.publish(pointcloud_in);
    }
    void callback(dmapping_refinement::calibParamsConfig &config, uint32_t level)
    {
        Tcalib  = vectorToAffine3d(0, 0, 0, config.ex*M_PI/180.0, config.ey*M_PI/180.0, config.ez*M_PI/180.0);
        timeOffset = config.t;
    }

    void ImuCallback(const sensor_msgs::ImuConstPtr& imuMsg){

        const ros::Time tStamp = imuMsg->header.stamp + ros::Duration(timeOffset);

        std::vector<tf::StampedTransform> trans_vek;

        tf::Transform Tfcalib;
        tf::transformEigenToTF(Tcalib, Tfcalib);
        trans_vek.push_back(tf::StampedTransform(Tfcalib, tStamp, "imuLink", "velodyneLink"));
//
        const Eigen::Quaterniond qImu(imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z );
        Timu =  EigenCombine(qImu, Eigen::Vector3d(0,0,0));
        tf::Transform Tfimu;
        tf::transformEigenToTF(Timu, Tfimu);
        trans_vek.push_back(tf::StampedTransform(Tfimu, tStamp, "odom", "imuLink"));

        Tbr.sendTransform(trans_vek);
    }

    calib(ros::NodeHandle& nh) :nh_(nh){
        Tcalib  = vectorToAffine3d(0, 0, 0, 0, 0, M_PI);
        timeOffset = 0;
        dynamic_reconfigure::Server<dmapping_refinement::calibParamsConfig> server;
        dynamic_reconfigure::Server<dmapping_refinement::calibParamsConfig>::CallbackType callback_function;
        dmapping_refinement::calibParamsConfig last_config;
        callback_function = boost::bind(&calib::callback, this, _1, _2);
        server.setCallback(callback_function);

        subLaserCloud = nh.subscribe("/velodyne_points", 100, &calib::velodyneHandler, this);
        subImu = nh.subscribe("/imu/data", 100, &calib::ImuCallback, this);
        pub = nh.advertise< pcl::PointCloud<PointType> >("/adjusted", 100);
        ros::spin();
    }


};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "calib");
    ros::NodeHandle nh("~");
    cout << "Startup" << endl;
    calib c(nh);


    return 0;
}


