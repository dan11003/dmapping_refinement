#include "dmapping_refinement/fuser.h"
namespace dmapping{

Fuser::Fuser(Parameters& par, boost::shared_ptr<PoseGraph> graph, ros::NodeHandle& nh) : par_(par), nh_(nh){

    graph_ = KeyFrameFilter(graph, par.keyframe_min_transl, par.keyframe_min_rot, par.tot_scans);
    for (auto& [index, surfel]: graph_->surfels_){
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
    ros::Time t = ros::Time::now();
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
    PublishCloud("/surf", *merged_surf, "world", t, nh_);
    PublishCloud("/edge", *merged_edge, "world", t,nh_);
    PublishCloud("/less_edge", *merged_less_edge, "world", t, nh_);
    cout << "Publish size: " << merged_surf->size() + merged_edge->size() + merged_less_edge->size() << endl;
    NormalCloud::Ptr merged_0(new NormalCloud());
    NormalCloud::Ptr merged_1(new NormalCloud());
    NormalCloud::Ptr merged_2(new NormalCloud());
    NormalCloud::Ptr merged_3(new NormalCloud());
    NormalCloud::Ptr merged_4(new NormalCloud());
    for(auto itr = graph_->nodes.begin() ; itr != graph_->nodes.end() ; itr++){
        const size_t idx = itr->first;
        NormalCloud::Ptr tmp_0(new NormalCloud());
        NormalCloud::Ptr tmp_1(new NormalCloud());
        NormalCloud::Ptr tmp_2(new NormalCloud());
        NormalCloud::Ptr tmp_3(new NormalCloud());
        NormalCloud::Ptr tmp_4(new NormalCloud());

        pcl::transformPointCloud(*graph_->surfels_[idx].GetPointCloud(0), *tmp_0, itr->second.T.matrix());
        pcl::transformPointCloud(*graph_->surfels_[idx].GetPointCloud(1), *tmp_1, itr->second.T.matrix());
        pcl::transformPointCloud(*graph_->surfels_[idx].GetPointCloud(2), *tmp_2, itr->second.T.matrix());
        pcl::transformPointCloud(*graph_->surfels_[idx].GetPointCloud(3), *tmp_3, itr->second.T.matrix());
        pcl::transformPointCloud(*graph_->surfels_[idx].GetPointCloud(4), *tmp_4, itr->second.T.matrix());

        *merged_0 += *tmp_0;
        *merged_1 += *tmp_1;
        *merged_2 += *tmp_2;
        *merged_3 += *tmp_3;
        *merged_4 += *tmp_4;
    }
    t = ros::Time::now();
    PublishCloud("/merged0", *merged_0, "world", t, nh_);
    PublishCloud("/merged1", *merged_1, "world", t, nh_);
    PublishCloud("/merged2", *merged_2, "world", t, nh_);
    PublishCloud("/merged3", *merged_3, "world", t, nh_);
    PublishCloud("/merged4", *merged_4, "world", t, nh_);

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
    GetParameters(parameters);
    NScanRefinement reg(par_.reg_par, parameters, surf_, stamps_, nh_);
    reg.Solve(parameters);
    //reg.Solve(parameters);
    cout << "solved" << endl;
    SetParameters(parameters);
}

void Fuser::GetParameters(std::map<int,NScanRefinement::Pose3d>& parameters){
    parameters.clear();
    for (const auto& [index, node]: graph_->nodes) {
        parameters[index].p = node.T.translation();
        parameters[index].q = Eigen::Quaterniond(node.T.linear());
        //cout << "Get parameter: " << node.T.matrix() << endl;
    }
}

void Fuser::SetParameters(const std::map<int,NScanRefinement::Pose3d>& parameters){
    for (const auto& [index, parameter]: parameters) {
        graph_->nodes[index].T = EigenCombine(parameter.q, parameter.p);
        //cout << "Set parameter: " << graph->nodes[index].T.matrix() << endl;
    }
}

}
