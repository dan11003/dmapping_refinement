#include "dmapping_refinement/fuser.h"
namespace dmapping{

Fuser::Fuser(Parameters& par, boost::shared_ptr<PoseGraph> graph, ros::NodeHandle& nh) : par_(par), nh_(nh){
    par_.submap_size = std::round(par_.submap_size/2)*2;

    graph_ = KeyFrameFilter(graph, par.keyframe_min_transl, par.keyframe_min_rot, par.tot_scans);
    for (auto& [index, surfel]: graph_->surfels_){
        surf_[index] = surfel.GetPointCloud();
        stamps_[index] = surfel.GetPointCloudTime();
        imu_[index] = graph_->nodes[index].imu;
    }
    // Set constraints
    for(auto itr = graph_->nodes.begin() ; std::next(itr) != graph_->nodes.end() ; itr++ ){
        const int idx = itr->first;
        const int nextIdx = std::next(itr)->first;
        const Eigen::Isometry3d Tdiff = graph_->nodes[idx].T.inverse()*graph_->nodes[nextIdx].T;
        graph_->AddConstraint(Tdiff, idx, nextIdx);
    }

    // Filter point clouds
    for (auto& [index, surf_cloud]: surf_){
        NormalCloud::Ptr filtered_normals(new NormalCloud());
        std::vector<pcl::Indices > k_indices;
        std::vector<std::vector<float> > k_sqr_distances;

        // Run search
        pcl::search::KdTree<pcl::PointXYZINormal> search;
        search.setInputCloud (surf_cloud);
        search.nearestKSearch (*surf_cloud, pcl::Indices(), 3, k_indices, k_sqr_distances);
        //cout << "k_ind: " << k_indices.size() << endl;
        pcl::NormalRefinement<pcl::PointXYZINormal> refinement(k_indices, k_sqr_distances);
        refinement.setInputCloud(surf_cloud);
        refinement.filter(*filtered_normals);
        surf_[index] = filtered_normals;
    }
}

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

void Fuser::RunDebugger(){
    int start = 0;
    int end = std::distance(graph_->nodes.begin(), graph_->nodes.end());

    std::string input = "";
    char *endp;

    while(ros::ok()){
        cout << "(r)egister, (s)tatus, (v)isualize, <number> set start" << endl;
        std::cin.clear();
        std::cin >> input;
        bool register_scans = false;
        long value = std::strtol(input.c_str(), &endp, 10);
        cout << "(r)egister, (v)isualize, <number> skip to" << endl;

        if(input =="r"){
            cout << "register" << endl;
            register_scans = true;
        }
        if(input =="s"){
            cout << "s" << endl;
            register_scans = true;
        }
        else if(input =="v"){
            cout << "visualize" << endl;
            Visualize();
        }
        else if (endp == input.c_str()) {
            /* conversion failed completely, value is 0, */
            /* endp points to start of given string */
            cout << "conversion failed completely, value is 0, endp points to start of given string " << endl;
        }
        else if (*endp != 0) {
            cout <<" got value, but entire string was not valid number, endp points to first invalid character " << endl;
        }
        else {
            start = value;
            cout << "skip to " << start << endl;
        }

        if(register_scans){
            std::map<int,bool> submap;
            auto itrFirst = std::advance(graph_->nodes.begin(), start);

            for(auto itr = itrFirst ;  std::distance(itrFirst,itr)  < graph_->nodes.size() && std::distance(itrFirst,itr) < par_.submap_size ; itr++ ){
                submap[itr->first] = (itr == itrFirst); // first true else false
            }

        }


    }
}


std::vector<std::map<int,bool>> Fuser::DivideSubmap(){
    const int halfStep = par_.submap_size/2;
    const int fullStep = par_.submap_size;
    Eigen::Isometry3d prev = graph_->nodes.begin()->second.T;
    std::vector<std::map<int,bool>> submaps;

    for(auto itrStart = graph_->nodes.begin() ;  std::distance(graph_->nodes.begin(),itrStart) + halfStep < graph_->nodes.size() ; std::advance(itrStart,halfStep) ){
        std::map<int,NScanRefinement::Pose3d> parameters;
        std::map<int,bool> submap;

        for(auto itr = itrStart
            ; std::distance(graph_->nodes.begin(),itr) < graph_->nodes.size() && std::distance(itrStart,itr) < fullStep
            ; std::advance(itr,1) )
        {
            const int count = std::distance(itrStart,itr);
            const bool first = (itrStart == graph_->nodes.begin());
            const bool lockParameter = (!first && count >= halfStep);
            const int idx = itr->first;
            //cout  << idx << ", ";
            submap[itr->first] = lockParameter;
        }
        submaps.push_back(submap);
    }
}
void Fuser::Optimize(){

    cout << "finished" << endl;
    std::vector<std::map<int,bool>> submaps = DivideSubmap();
    auto submap = submaps.front();
    std::map<int,NScanRefinement::Pose3d> parameters;
    for (auto itr = submap.begin() ; itr != submap.end() ; itr++) {
        const int currIdx = itr->first;
        if(itr->second){ // lock this parameter
            parameters[currIdx] = ToPose3d(graph_->nodes[currIdx].T); // this parameter was already optimized, keep parameter
        }
        else{
            const int prevIdx = std::prev(itr)->first;
            const Eigen::Isometry3d prev = ToIsometry3d(parameters[prevIdx]);
            const Eigen::Isometry3d inc = graph_->constraints[std::make_pair(prevIdx,currIdx)];
            parameters[currIdx] = ToPose3d(prev*inc); // this is important, otherwise there will be additional drift between submaps. This reuses the odometry prior relatively
        }
        //cout << "Get parameter: " << node.T.matrix() << endl;
    }



    /*std::map<int,NScanRefinement::Pose3d> parameters;
    GetParameters(parameters);
    NScanRefinement reg(par_.reg_par, parameters, surf_, stamps_, imu_, nh_);
    reg.Solve(parameters);
    //reg.Solve(parameters);
    cout << "solved" << endl;
    SetParameters(parameters);*/
}

void Fuser::GetParameters(std::map<int,NScanRefinement::Pose3d>& parameters){
    parameters.clear();
    for (const auto& [index, node]: graph_->nodes) {
        parameters[index] = ToPose3d(node.T);
        //cout << "Get parameter: " << node.T.matrix() << endl;
    }
}

void Fuser::SetParameters(const std::map<int,NScanRefinement::Pose3d>& parameters){
    for (const auto& [index, parameter]: parameters) {
        graph_->nodes[index].T = EigenCombine(parameter.q, parameter.p);
        //cout << "Set parameter: " << graph->nodes[index].T.matrix() << endl;
    }
}

Eigen::Isometry3d ToIsometry3d(const NScanRefinement::Pose3d& T){
    return EigenCombine(T.q, T.p);
}
NScanRefinement::Pose3d ToPose3d(const Eigen::Isometry3d& T){
    return NScanRefinement::Pose3d{T.translation(), Eigen::Quaterniond(T.linear()) };
}

}
