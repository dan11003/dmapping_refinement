#include "dmapping_refinement/fuser.h"
namespace dmapping{

Fuser::Fuser(Parameters& par, boost::shared_ptr<PoseGraph> graph) : par_(par){
    //if(par.use_keyframe)
    graph_ = KeyFrameFilter(graph, par.keyframe_min_transl, par.keyframe_min_rot, par.tot_scans);
    /*else
        graph_ = graph;*/

    for (auto& [index, surfel]: graph->surfels_){
        surf_[index] = surfel.GetPointCloud();
        stamps_[index] = surfel.GetPointCloudTime();
    }
}


//n.segmented_scans = {pointcloud_surf_in, pointcloud_edge_in, pointcloud_less_edge_in};
void Fuser::Visualize(){
    static tf::TransformBroadcaster Tbr;
    std::vector<tf::StampedTransform> trans_vek;
    pcl::PointCloud<PointType>::Ptr merged_edge(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr merged_less_edge(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr merged_surf(new pcl::PointCloud<PointType>());
    const ros::Time t = ros::Time::now();
    for(auto itr = graph_->nodes.begin() ; itr != graph_->nodes.end() ; itr++){
        pcl::PointCloud<PointType> tmp_surf, tmp_edge, tmp_less_edge;
        pcl::transformPointCloud(*itr->second.segmented_scans[0], tmp_surf, itr->second.T.matrix());
        pcl::transformPointCloud(*itr->second.segmented_scans[1], tmp_edge, itr->second.T.matrix());
        pcl::transformPointCloud(*itr->second.segmented_scans[2], tmp_less_edge, itr->second.T.matrix());
        *merged_surf += tmp_surf;
        *merged_edge +=  tmp_edge;
        *merged_less_edge +=  tmp_less_edge;
        tf::Transform Tf;
        tf::transformEigenToTF(itr->second.T, Tf);
        trans_vek.push_back(tf::StampedTransform(Tf, t, "world", "node_"+std::to_string(itr->first)));
    }
    Tbr.sendTransform(trans_vek);
    PublishCloud("/surf", *merged_surf, "world", t);
    PublishCloud("/edge", *merged_edge, "world", t);
    PublishCloud("/less_edge", *merged_less_edge, "world", t);
    cout << "Publish size: " << merged_surf->size() + merged_edge->size() + merged_less_edge->size() << endl;
}

void Fuser::Run(){
    Visualize();
    usleep(1000*1000);
    Visualize();
    Optimize();
    ros::Rate r(0.2);
    while(ros::ok()){
        r.sleep();
        Visualize();
    }
}
void Fuser::Optimize(){
    std::map<int,NScanRefinement::Pose3d> parameters;
    GetParameters(parameters, graph_);
    NScanRefinement reg(par_.reg_par, parameters, surf_, stamps_);
    reg.Solve(parameters);
    cout << "solved" << endl;
    SetParameters(parameters, graph_);
}

void Fuser::GetParameters(std::map<int,NScanRefinement::Pose3d>& parameters, const boost::shared_ptr<PoseGraph> graph){
    parameters.clear();
    for (const auto& [index, node]: graph->nodes) {
        parameters[index].p = node.T.translation();
        parameters[index].q = Eigen::Quaterniond(node.T.linear());
        //cout << "Get parameter: " << node.T.matrix() << endl;
    }
}

void Fuser::SetParameters(const std::map<int,NScanRefinement::Pose3d>& parameters, boost::shared_ptr<PoseGraph> graph){
    for (const auto& [index, parameter]: parameters) {
        graph->nodes[index].T = EigenCombine(parameter.q, parameter.p);
        //cout << "Set parameter: " << graph->nodes[index].T.matrix() << endl;
    }
}

}
