#include "dmapping_refinement/fuser.h"
namespace dmapping{

Fuser::Fuser(Parameters& par, boost::shared_ptr<PoseGraph> graph, ros::NodeHandle& nh) : par_(par), nh_(nh){
    par_.submap_size = std::round(par_.submap_size/2)*2;

    if(par.use_keyframe){
        cout << "Keyframe filtering" << endl;
        graph_ = KeyFrameFilter(graph, par.keyframe_min_transl, par.keyframe_min_rot, par.tot_scans);
    }
    else{
        graph_ = graph;
    }
    for(auto && node : graph_->nodes){
      Eigen::Quaterniond q(node.second.T.linear());
      q.normalize();
      node.second.T = EigenCombine(q,node.second.T.translation());
    }

    for (auto& [index, surfel]: graph_->surfels_){
        surf_[index] = surfel.GetPointCloud();
        imu_[index] = graph_->nodes[index].imu;
    }
    // Set constraints
    for(auto itr = graph_->nodes.begin() ; std::next(itr) != graph_->nodes.end() ; itr++ ){
        const int idx = itr->first;
        const int nextIdx = std::next(itr)->first;
        Eigen::Isometry3d Tdiff = graph_->nodes[idx].T.inverse()*graph_->nodes[nextIdx].T;
        Eigen::Quaterniond q(Tdiff.linear());
        q.normalize();
        Tdiff = EigenCombine(q, Tdiff.translation());
        graph_->AddConstraint(Tdiff, idx, nextIdx);
    }
    pub = nh_.advertise<visualization_msgs::Marker>("/submap_normals",100);
    //pubAfterRefinement = nh_.advertise<pcl::PointCloud<pcl::PointXYZINormal> >("/after refinement",100);
    pubDownsampled = nh_.advertise<visualization_msgs::Marker>("/submap_normals_downsampled",100);

    // Filter point clouds
    //for (auto itr = surf_.begin() ; itr != surf_.end() ; itr++){
    //for (auto& [index, surf_cloud]: surf_){
    cout << "Normal refinement started" << endl;
#pragma omp parallel
    {
#pragma omp single
        {
            for (auto itr = surf_.begin() ; itr != surf_.end() ; itr++){
#pragma omp task
                {
                    //do something

                    const int index = itr->first;
                    auto surf_cloud = itr->second;
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
#pragma omp critical
                    {
                        surf_[index] = filtered_normals;
                    }
                }
            }
        }
        cout << "Normal refinement finished" << endl;
    }
}
void Fuser::Align(){

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 12;
    ceres::Solver::Summary summary;
    ceres::Problem problem;
    ceres::LossFunction* loss = new ceres::CauchyLoss(0.01);

    NormalCloud::Ptr mergedSurf(new NormalCloud());
    NormalCloud::Ptr mergedLocal(new NormalCloud());
    NormalCloud::Ptr mergedAligned(new NormalCloud());
    for(auto itr = graph_->nodes.begin() ; itr != graph_->nodes.end() ; itr++){
        const int idx = itr->first;
        NormalCloud::Ptr tmp(new NormalCloud());
        pcl::transformPointCloud(*surf_[idx], *tmp, itr->second.T.matrix());
        *mergedSurf += *tmp;
    }

    const Eigen::Affine3d poseFirst(graph_->nodes.begin()->second.T.matrix());
    Eigen::AngleAxisd rotBefore(poseFirst.linear());
    cout << "pose: " << poseFirst.translation().transpose() << " - " << rotBefore.axis().transpose()*rotBefore.angle() << endl;
    pcl::transformPointCloud(*mergedSurf, *mergedLocal, poseFirst.inverse());

    NormalCloud::Ptr mergedLocalDownsampled(new NormalCloud());
    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    sor.setMinimumPointsNumberPerVoxel(2);
    sor.setInputCloud(mergedLocal);
    sor.setLeafSize (0.02, 0.1, 0.1);
    sor.filter (*mergedLocalDownsampled);

    PublishCloud("/AfterAlignment", *mergedLocalDownsampled, "world", "sensorBefore", poseFirst, ros::Time::now(), nh_);

    Eigen::Quaterniond q(poseFirst.linear());

    for(auto && p : mergedLocalDownsampled->points){
        const Eigen::Vector3d normal(p.normal_x, p.normal_y, p.normal_z);
        ceres::CostFunction* cost_function = planarCostFunction::Create(normal);
        problem.AddResidualBlock(cost_function, loss, q.coeffs().data());
    }
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization();
    problem.SetParameterization(q.coeffs().data(), quaternion_local_parameterization);

    ceres::Solve(options, &problem, &summary);
    Eigen::Affine3d poseAfter(EigenCombine(q, poseFirst.translation()).matrix());
    Eigen::AngleAxisd rotAfter(poseFirst.linear());



    PublishCloud("/AfterAlignment", *mergedLocalDownsampled, "world", "sensorAfter", poseAfter, ros::Time::now(), nh_);
    mergedLocalDownsampled->header.frame_id = "sensorAfter";
    std::map<int,NormalCloud::Ptr> cldMap = {{0,mergedLocalDownsampled}};
    VisualizePointCloudNormal(cldMap, "submapDownsampled", pubDownsampled);

    cout << "pose: " << poseFirst.translation().transpose() << " - " << rotAfter.axis().transpose()*rotAfter.angle() << endl;




    if(!summary.IsSolutionUsable()){
        cout << "ERROR NOT USABLE" << endl;
        exit(0);
    }else{
        cout << "Solved - score: " << summary.final_cost / summary.num_residuals << endl;
    }
    cout << summary.FullReport() << endl;


}

void Fuser::Visualize(){
    static tf::TransformBroadcaster Tbr;
    std::vector<tf::StampedTransform> trans_vek;
    pcl::PointCloud<PointType>::Ptr merged_edge(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr merged_less_edge(new pcl::PointCloud<PointType>());
    NormalCloud::Ptr merged_surf = NormalCloud().makeShared();
    ros::Time t = ros::Time::now();
    for(auto itr = graph_->nodes.begin() ; itr != graph_->nodes.end() ; itr++){
        //pcl::PointCloud<PointType> tmp_surf, tmp_edge, tmp_less_edge;
        NormalCloud tmp_surf;
        pcl::transformPointCloud(*surf_[itr->first], tmp_surf, itr->second.T.matrix()); // not good
        //pcl::transformPointCloud(*itr->second.segmented_scans[1], tmp_edge, itr->second.T.matrix());
        //pcl::transformPointCloud(*itr->second.segmented_scans[2], tmp_less_edge, itr->second.T.matrix());
        *merged_surf += tmp_surf;
        //*merged_edge +=  tmp_edge;
        //*merged_less_edge +=  tmp_less_edge;
        tf::Transform Tf;
        tf::transformEigenToTF(itr->second.T, Tf);
        trans_vek.push_back(tf::StampedTransform(Tf, t, "world", "node_"+std::to_string(itr->first)));
    }
    Tbr.sendTransform(trans_vek);
    PublishCloud("/surf", *merged_surf, "world", t, nh_);
    //PublishCloud("/edge", *merged_edge, "world", t,nh_);
    //PublishCloud("/less_edge", *merged_less_edge, "world", t, nh_);
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


void Fuser::Save(const std::string& directory, const std::string& prefix, const  double resolution){

    const std::string singleCloudsDir = directory + "/export/" + prefix;
    boost::filesystem::create_directories(singleCloudsDir);
    std::map<int,NScanRefinement::Pose3d> parameters;
    GetParameters(parameters);
    cout << "Saving: " << parameters.size() << " to " << directory << endl;
    NormalCloud::Ptr merged(new NormalCloud());

    std::vector<Eigen::Affine3d> poses;
    //std::vector<double> stamps;
    std::ofstream data_ofs(singleCloudsDir + "/odom.poses");
    for(auto itr = graph_->nodes.begin() ; itr != graph_->nodes.end(); itr++){

        Eigen::Matrix<double,4,4> mat = itr->second.T.matrix();
        data_ofs << itr->first << std::endl;
        data_ofs << mat(0,0) << " " << mat(0,1) << " " << mat(0,2) << " " << mat(0,3) << std::endl;
        data_ofs << mat(1,0) << " " << mat(1,1) << " " << mat(1,2) << " " << mat(1,3) << std::endl;
        data_ofs << mat(2,0) << " " << mat(2,1) << " " << mat(2,2) << " " << mat(2,3) << std::endl;
        data_ofs << mat(3,0) << " " << mat(3,1) << " " << mat(3,2) << " " << mat(3,3) << std::endl;
        //stamps.push_back(pcl_conversions::fromPCL(itr->second.segmented_scans.front()->header.stamp).toSec());
    }
    data_ofs.close();
    //SaveOdom(singleCloudsDir, poses, stamps, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>());


    NormalCloud::Ptr mergedDownsampled(new NormalCloud());
    for(auto itr = graph_->nodes.begin() ; itr != graph_->nodes.end(); itr++){
        const int idx = itr->first;
        NormalCloud::Ptr tmp(new NormalCloud());
        pcl::transformPointCloud(*surf_[idx], *tmp, itr->second.T.matrix());
        *merged += *tmp;
        const std::string filename =  singleCloudsDir + "/" + prefix +"_" + std::to_string(idx) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *tmp);

    }
    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    sor.setMinimumPointsNumberPerVoxel(2);
    sor.setInputCloud(merged);
    sor.setLeafSize (resolution, resolution, resolution);
    sor.filter (*mergedDownsampled);


    if(!merged->empty()){
        cout << "Saving full pointcloud" << endl;
        const std::string filename =  directory + prefix + ".pcd";
        pcl::io::savePCDFileBinary(filename, *merged);
    }
    if(!mergedDownsampled->empty()){
        cout << "Saving downsampled pointcloud" << endl;
        const std::string filename =  directory + prefix + "_downsample_"+ ".pcd";
        pcl::io::savePCDFileBinary(filename, *mergedDownsampled);
    }
    cout << "Saving finished" << endl;
}

void Fuser::RunDebugger(){
    int start = par_.skip_frames;
    int end = std::distance(graph_->nodes.begin(), graph_->nodes.end());

    std::string input = "";
    char *endp;

    while(ros::ok()){
        cout << "Index: " << start << endl;
        cout << "(r)egister, (s)tatus, (v)isualize, <number> set start, (e)xit" << endl;
        std::cin.clear();
        std::cin >> input;
        bool register_scans = false, align = false;

        long value = std::strtol(input.c_str(), &endp, 10);
        cout << "(a)lign, (r)egister, (v)isualize, <number> skip to" << endl;

        if(input =="r"){ cout << "(r)egister" << endl;       register_scans = true; }
        else if(input =="a"){ cout << "(a)lign" << endl; align = true; }
        else if(input =="e"){cout << "(e)xit" << endl; exit(0); }
        else if(input =="v"){ cout << "(v)isualize" << endl; Visualize(); }
        else if (endp == input.c_str()) { /*cout << "conversion failed completely, value is 0, endp points to start of given string " << endl; */}
        else if (*endp != 0) {/* cout <<" got value, but entire string was not valid number, endp points to first invalid character " << endl; */}
        else {
            if( graph_->nodes.find(value) != graph_->nodes.end()  ){
                start = value;
                cout << "skip to " << start << endl;
                //cout <<"node id: " <<  graph_->nodes.find(start)->first << endl;
            }
            else{
                cout << "index out of range " << endl;
            }
        }
        if(align){
            Align();
        }
        else if(register_scans){
            cout << "Register" << endl;
            std::map<int,bool> submapLock;
            auto itrFirst = graph_->nodes.find(start);
            for(auto itr = itrFirst ;  std::distance(itrFirst,itr)  < graph_->nodes.size() && std::distance(itrFirst,itr) < par_.submap_size ; itr++ ){
                submapLock[itr->first] = (itr == itrFirst); // first true else false
            }
            std::map<int,NScanRefinement::Pose3d> posePar, velPar;
            for(auto && [index,lock] : submapLock){
                posePar[index] = ToPose3d(graph_->nodes[index].T);
            }
            cout << "Coarse registration" << endl;
            NScanRefinement reg(par_.reg_par, posePar, surf_, imu_, nh_);
            reg.Solve(posePar, velPar, submapLock);

            std::map<int,NormalCloud::Ptr> output;
            NormalCloud::Ptr mergedPrev(new NormalCloud());
            NormalCloud::Ptr mergedDownsampled(new NormalCloud());
            reg.GetPointCloudsSurfTransformed(output);
            for(auto & [index,cloud] : output){
                *mergedPrev += *cloud;
            }
            PublishCloud("/refined_scans", *mergedPrev, "world", ros::Time::now(), nh_);
            PublishCloud("/refined_scans", *mergedPrev, "world", ros::Time::now(), nh_);
            /*pcl::VoxelGrid<pcl::PointXYZINormal> sor;
            sor.setMinimumPointsNumberPerVoxel(2);
            sor.setInputCloud(mergedPrev);
            sor.setLeafSize (0.05, 0.05, 0.05);
            sor.filter (*mergedDownsampled);*/
            std::map<int,NormalCloud::Ptr> submap = {{0,mergedPrev}};
            std::map<int,NormalCloud::Ptr> submapDownsampled = {{0,mergedDownsampled}};

            VisualizePointCloudNormal(submap, "submapRaw", pub);
            VisualizePointCloudNormal(submapDownsampled, "submapDownsampled", pubDownsampled);
            /*NScanRefinement::Parameters parFineGrained = {2, 5, par_.reg_par.max_dist_association*0.5, "cauchy"};
            cout << "Fine registration" << endl;
            reg.Solve(parameters, submapLock);*/
        }


    }
}



std::vector<std::map<int,bool>> Fuser::DivideSubmapNoOverlap(){
    const int fullStep = par_.submap_size;
    std::vector<std::map<int,bool> > submaps; // to hold the indices of the sub-vector
    auto it = graph_->nodes.begin();
    while (it != graph_->nodes.end()) {
        std::map<int,bool> subVector; // to hold the indices of the sub-vector
        for (int i = 0; i < fullStep && it != graph_->nodes.end(); ++i, ++it) {
            if(it == graph_->nodes.begin())
                subVector[it->first] = true;
            else
                subVector[it->first] = false;
        }
        submaps.push_back(subVector);
    }
    return submaps;
}


std::vector<std::map<int,bool>> Fuser::DivideSubmap(){
    const int halfStep = par_.submap_size/2;
    const int fullStep = par_.submap_size;
    Eigen::Isometry3d prev = graph_->nodes.begin()->second.T;
    std::vector<std::map<int,bool>> submaps;
    int nr = 0;

    for(auto itrStart = graph_->nodes.begin() ;  std::distance(graph_->nodes.begin(),itrStart) + halfStep < graph_->nodes.size() ; std::advance(itrStart,halfStep) ){
        std::map<int,NScanRefinement::Pose3d> parameters;
        std::map<int,bool> submap;
        cout  << "submap: ";

        for(auto itr = itrStart
            ; std::distance(graph_->nodes.begin(),itr) < graph_->nodes.size() && std::distance(itrStart,itr) < fullStep
            ; std::advance(itr,1) )
        {
            const int count = std::distance(itrStart,itr);
            const bool firstSubmap = (itrStart == graph_->nodes.begin());
            bool lockParameter = true;// Generally keep parameter locked
            if(count >= halfStep || (firstSubmap && nr++ > 0) ) { //Except for 2nd half of submap, or for index 1..halfStep of first submap
                lockParameter = false;
            }
            //const bool lockParameter = (!first && count >= halfStep); // first submap? always unlock
            const int idx = itr->first;
            cout  << idx << ", ";
            submap[itr->first] = lockParameter;
        }
        cout << endl;
        submaps.push_back(submap);
    }
    return submaps;
}
void Fuser::Run(){

    ros::Time t0 = ros::Time::now();
    cout << "submap partition" << endl;
    std::vector<std::map<int,bool>> submaps = DivideSubmap();
    for (auto&& submap : submaps) {
        std::map<int,NScanRefinement::Pose3d> posePar, velPar;

        for (auto itr = submap.begin() ; itr != submap.end() ; itr++) {
            const int currIdx = itr->first;
            const bool lockParameter = itr->second;
            //cout << currIdx <<", " << endl;

            if(lockParameter){ // lock this parameter
                //cout <<"lock: "<< currIdx <<", ";
                posePar[currIdx] = ToPose3d(graph_->nodes[currIdx].T); // copy directly!
            }
            else{
                //cout <<"unlock: "<< currIdx <<", ";
                const int prevIdx = std::prev(itr)->first;
                const Eigen::Isometry3d prev = ToIsometry3d(posePar[prevIdx]);
                const Eigen::Isometry3d inc = graph_->constraints[std::make_pair(prevIdx,currIdx)];
                posePar[currIdx] = ToPose3d(prev*inc); // this is important, otherwise there will be additional drift between submaps
            }
            //cout << "Get parameter: " << node.T.matrix() << endl;
            if(!ros::ok())
                break;
        }
        cout << endl;
        NScanRefinement reg(par_.reg_par, posePar, surf_, imu_, nh_);
        reg.Solve(posePar, velPar, submap);
        SetParameters(posePar);
        double timeElapsed = (ros::Time::now() - t0).toSec();
        cout << "Elapsed: " << timeElapsed << endl;
        if(par_.max_time > 0 && timeElapsed > par_.max_time){
            return;
        }
        /*NScanRefinement::Parameters parFineGrained = {2, 5, par_.reg_par.max_dist_association*0.5, "cauchy"};
        cout << "Fine grained" << endl;
        reg.Solve(parameters, submap);
        SetParameters(parameters);*/
    }
    cout << "Finish Fuser" << endl;

    /*std::map<int,NScanRefinement::Pose3d> parameters;
    GetParameters(parameters);
    NScanRefinement reg(par_.reg_par, parameters, surf_, imu_, nh_);
    reg.Solve(parameters);
    //reg.Solve(parameters);
    cout << "solved" << endl;
    SetParameters(parameters);*/
}
void Fuser::RunSubmapFuser(){

    ros::Time t0 = ros::Time::now();

    Eigen::Isometry3d poseScanLast = Eigen::Isometry3d::Identity();
    int idxScanLast;
    bool firstSubmap = true;
    std::vector<std::map<int,bool>> submaps = DivideSubmapNoOverlap();
    for (auto submapItr = submaps.begin() ; submapItr != submaps.end() ; submapItr++ ) {
        std::map<int,NScanRefinement::Pose3d> posePar, velPar;
        if(!ros::ok())
            return;

        if( !submapItr->empty() && submapItr->begin()->first < par_.skip_frames ){ // if there is something to process and index should not be skipped
            cout << "skip to " <<par_.skip_frames << endl;
            continue;
        }

        for (auto scanItr = submapItr->begin() ; scanItr != submapItr->end() ; scanItr++) {
            const int currIdx = scanItr->first;
            if(firstSubmap){ // lock the very first node only
              cout << "first submap" << endl;
                posePar[currIdx] = ToPose3d(graph_->nodes[currIdx].T); // copy directly from graph
                firstSubmap = false;
            }
            else if(scanItr == submapItr->begin()){  // if first in the submap but not in first node,
              cout << "first scan - not first submap" << endl;
                const Eigen::Isometry3d inc = graph_->constraints[std::make_pair(idxScanLast,currIdx)]; // we use odometry prediciton stored in the constraints...
                cout << "inc: " << inc.translation().transpose() << endl;
                cout << "last submap pos: " << inc.translation().transpose() << endl;
                posePar[currIdx] = ToPose3d(graph_->nodes[idxScanLast].T*inc); //... to predict the current pose
            }
            else{ //use the posePar vector as well as graph constraints
                cout << "Anywhere" << endl;
                const int prevIdx = std::prev(scanItr)->first;
                const Eigen::Isometry3d prev = ToIsometry3d(posePar[prevIdx]);
                const Eigen::Isometry3d inc = graph_->constraints[std::make_pair(prevIdx,currIdx)];
                posePar[currIdx] = ToPose3d(prev*inc); // this is important, otherwise there will be additional drift between submaps
            }
        }

        cout << "history: " << keyframesHistory.size() << endl;
        std::map<int,bool> submapWithKeyFrames = *submapItr;
        auto surfWithKeyFrame = surf_;
        for(auto && keyframe : keyframesHistory){
            surfWithKeyFrame[keyframe.idx] = keyframe.surf;
            posePar[keyframe.idx] = ToPose3d(keyframe.pose);
            submapWithKeyFrames[keyframe.idx]  = true;
        }

        NScanRefinement reg(par_.reg_par, posePar, surfWithKeyFrame, imu_, nh_);
        reg.Solve(posePar, velPar, submapWithKeyFrames);


        NormalCloud::Ptr aggregated = NormalCloud().makeShared();
        NormalCloud::Ptr aggregatedTransformed = NormalCloud().makeShared();
        idxScanLast = std::prev(submapItr->end())->first;
        poseScanLast =  ToIsometry3d(posePar[idxScanLast]);
        cout << "last pos: " << poseScanLast.translation().transpose() << endl;
        bool fuseKeyframeToHistory = false;
        if(keyframesHistory.empty()){
          fuseKeyframeToHistory = true;
        }
        else{
            const double distance = (keyframesHistory.back().pose.inverse()*poseScanLast).translation().norm();
            if(distance > par_.minDistanceKeyframe ){
              cout << "Sufficient movement - scan fused, movement:  " << distance << endl;
              fuseKeyframeToHistory = true;
            }
            else{
              cout << "Do not fuse" << endl;
            }
            //cout << "Sufficient movement - scan fused, movement:  " <<(keyframesHistory.back().pose.inverse()*poseScanLast).translation().norm() << endl;

        }


        for(auto && [idx,lock] : (*submapItr)){ // apply non-rigid transformation and concatinate scans into local frame, do only for scans, not previous keyframes
            NonRigidTransform(velPar[idx], {0,0,0}, NScanRefinement::Pose3d::Identity(), surf_[idx], surf_[idx]); // replace surf with deskewing - has no effect if velocity is not considered
            NormalCloud::Ptr tmp = NormalCloud().makeShared();
            pcl::transformPointCloudWithNormals(*surf_[idx], *tmp, (poseScanLast.inverse()*ToIsometry3d(posePar[idx])).matrix()); //Transform to local frame of SubmapLas
            *aggregated += *tmp;
            graph_->nodes[idx].T = ToIsometry3d(posePar[idx]);
            //cout << "Set: " << idx <<", at: " << graph_->nodes[idx].T.translation().transpose() << endl;
        }
        for(auto && p : aggregated->points){
            p.curvature = 0;
        }
        std::map<int,NormalCloud::Ptr> aggMap = {{-idxScanLast,aggregated}};
        pcl::transformPointCloudWithNormals(*aggregated, *aggregatedTransformed, poseScanLast.matrix()); //Transform to local frame of SubmapLas

        PublishCloud("/aggregatedTransformed", *aggregatedTransformed, "world", ros::Time::now(), nh_);
        //VisualizePointCloudNormal(aggMap, "aggregated", pub);
        if(fuseKeyframeToHistory){
            keyframesHistory.push_back(keyframes{poseScanLast, aggregated, -idxScanLast}); // make negative to avoid conflict - only submaps are negative
            if(keyframesHistory.size() > par_.submap_history)
                keyframesHistory.erase(keyframesHistory.begin());
        }


        /*
        double timeElapsed = (ros::Time::now() - t0).toSec();
        cout << "Elapsed: " << timeElapsed << endl;
        if(par_.max_time > 0 && timeElapsed > par_.max_time){
            return;
        }*/
        /*NScanRefinement::Parameters parFineGrained = {2, 5, par_.reg_par.max_dist_association*0.5, "cauchy"};
        cout << "Fine grained" << endl;
        reg.Solve(parameters, submap);
        SetParameters(parameters);*/
    }
    cout << "Finish Fuser" << endl;

}
void ApplyTransformation(std::map<int,NormalCloud::Ptr>, bool rigid, bool nonRigid, const std::map<int,bool>& locked){


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
