

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
#include "lio_sam/generics.h"
#include "dmapping_refinement/utils.h"
#include "dmapping_refinement/fuser.h"
#include <omp.h>



bool keep_running = true;

class RefinementNode
{
public:

    ros::NodeHandle& nh;
    lidar::Lidar lidar_param;
    std::string directory;
    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    double output_downsample_size = 0.05;
    std::string loss_function = "Huber";
    bool save_Posegraph = false;
    bool save_BALM = true;
    bool save_odom = false;
    bool export_pcd = true;
    std::string constraint_graph_path;
    dmapping::Fuser::Parameters fuserpar;


    RefinementNode(ros::NodeHandle& handle) : nh(handle) {
        Initialize();
        Optimize();
    }
    bool ComputeSurfels(){
        if(graph->surfels_.empty()){
            //#pragma omp parallel for
            for(auto itr = graph->nodes.begin() ; itr != graph->nodes.end() ; itr++){
                const size_t idx = itr->first;
                cout <<"extract: "<< idx << endl; //", :" <<<<itr->second.segmented_scans.front()->size()<<endl;
                cout <<  fuserpar.lidar_param.num_lines << ", " << fuserpar.lidar_param.scan_period << endl;
                cout << itr->second.segmented_scans.front()->size() << endl;
                SurfelExtraction extr(itr->second.segmented_scans.front(), fuserpar.lidar_param.num_lines, fuserpar.lidar_param.scan_period);
                cout <<"extract: "<< idx << endl; //", :" <<<<itr->second.segmented_scans.front()->size()<<endl;
                SurfElCloud surfelcloud;
                extr.Extract(surfelcloud);
                //#pragma omp critical
                {
                    graph->surfels_[idx] = std::move(surfelcloud);
                }
            }
            return true;
        }
        else
            return false;
    }

    void Initialize(){

        fuserpar.GetParametersFromRos(nh);
        nh.getParam("/directory_output", directory);
        nh.getParam("/output_downsample_size", output_downsample_size);
        nh.getParam("/constraint_graph_path", constraint_graph_path);
        if(!(PoseGraph::LoadGraph(constraint_graph_path+"precompute", graph) || PoseGraph::LoadGraph(constraint_graph_path, graph)) ){
            exit(0);
        }
        if(ComputeSurfels()){
            std::cout << "Saving pose garph" << std::endl;
            PoseGraph::SaveGraph(constraint_graph_path+"precompute", graph);
        }
        else{
            cout << "Loaded precomputed surfels" << endl;
        }
    }
    void Optimize(){
        dmapping::Fuser fuser(fuserpar, graph, nh);
        fuser.Run();
    }
    boost::shared_ptr<PoseGraph> graph = nullptr;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "refinement");
    ros::NodeHandle nh("~");
    RefinementNode node(nh);
    ros::spin();

    return 0;
}


