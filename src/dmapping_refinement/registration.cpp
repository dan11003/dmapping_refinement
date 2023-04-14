#include "dmapping_refinement/registration.h"

namespace dmapping {

NScanRefinement::NScanRefinement(Parameters& par, const std::map<int,Pose3d>& poses, std::map<int,NormalCloud::Ptr>& surf, std::map<int,std::vector<double> >& stamps, std::map<int,Eigen::Quaterniond>& imu, ros::NodeHandle& nh) : par_(par), poses_(poses), surf_(surf), stamps_(stamps), imu_(imu), nh_(nh){
    //loss_function = new ceres::HuberLoss(0.1); //= ceres::DENSE_QR;
    options.max_num_iterations = par_.inner_iterations;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 6;
    vis_pub = nh_.advertise<visualization_msgs::Marker>("/correspondances",100);
    normal_pub = nh_.advertise<visualization_msgs::Marker>("/normals",100);
    //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;


    // Initialize
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        velocities_[idx].p = Eigen::Vector3d(0,0,0);
        velocities_[idx].q = Eigen::Quaterniond::Identity();
    }
    // Filter
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        filtered_[idx] = NormalCloud().makeShared();
        pcl::VoxelGrid<pcl::PointXYZINormal> sor;
        sor.setMinimumPointsNumberPerVoxel(1);
        sor.setInputCloud(surf_[idx]);
        sor.setLeafSize (0.05f, 0.05f, 0.05f);
        sor.filter (*filtered_[idx]);
        cout <<"Downsampling rate: " << (double)filtered_[idx]->size() / surf_[idx]->size()  << endl;
    }
    //Create Eigen representation, is this needed?
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        NormalCloud::Ptr tmp_filtered = filtered_[idx];
        std::vector<Eigen::Vector3d> pntLocal(tmp_filtered->size());
        std::vector<Eigen::Vector3d> normalsLocal(tmp_filtered->size());
        for(size_t i = 0 ; i < tmp_filtered->size() ; i++){
            const auto& p = tmp_filtered->points[i];
            pntLocal[i] = Eigen::Vector3d(p.x, p.y, p.z);
            normalsLocal[i] = Eigen::Vector3d(p.normal_x, p.normal_y, p.normal_z);
        }
        means_local_[idx] = std::move(pntLocal);
        normals_local_[idx] = std::move(normalsLocal);
    }
}


void NScanRefinement::Visualize(const std::string& topic){
    static tf::TransformBroadcaster Tbr;
    std::vector<tf::StampedTransform> trans_vek;
    NormalCloud::Ptr surf_filtered(new NormalCloud());
    NormalCloud::Ptr surf_unfiltered(new NormalCloud());
    NormalCloud::Ptr surf_downsampled(new NormalCloud());
    const ros::Time t_cloud = ros::Time::now();
    //TransformCommonFrame(const std::map<int,NormalCloud::Ptr>& input, std::map<int,NormalCloud::Ptr>& output, const bool compute_kdtree){

    TransformCommonFrame(filtered_, transformed_, false);
    std::map<int,NormalCloud::Ptr> unfiltered_;
    TransformCommonFrame(surf_, unfiltered_, false);
    int nr = 0;
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        *surf_filtered += *transformed_[idx];
        *surf_unfiltered += *unfiltered_[idx];
        PublishCloud("registerd_"+std::to_string(nr++), *transformed_[idx], "world", t_cloud, nh_);
        tf::Transform Tf;
        const Eigen::Affine3d Tnode = EigenCombine(itr->second.q, itr->second.p);
        tf::transformEigenToTF(Tnode, Tf);
        trans_vek.push_back(tf::StampedTransform(Tf, t_cloud, "world", "reg_node_"+std::to_string(idx)));

        /*const Eigen::Affine3d Timu = EigenCombine(imu_[idx], itr->second.p);
    tf::transformEigenToTF(Timu, Tf);
    trans_vek.push_back(tf::StampedTransform(Tf, t_cloud, "world", "reg_imu_"+std::to_string(idx)));*/
    }
    Tbr.sendTransform(trans_vek);
    PublishCloud(topic, *surf_filtered, "world", t_cloud, nh_);
    PublishCloud(topic+"unfiltered", *surf_unfiltered, "world", t_cloud, nh_);

    PublishCloud(topic+"unfiltered", *surf_unfiltered, "world", t_cloud, nh_);
    pcl::VoxelGrid<pcl::PointXYZINormal> sor;
    sor.setMinimumPointsNumberPerVoxel(2);
    sor.setInputCloud(surf_unfiltered);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*surf_downsampled);
    PublishCloud(topic+"unfiltered_downsampled", *surf_downsampled, "world", t_cloud, nh_);
    //cout << "Publish unfiltered: " << surf_unfiltered->size() << " to " << topic << endl;
    //cout << "Publish filtered: " << surf_filtered->size() << " to " << topic << endl;
}


std::vector<std::pair<int,int> > NScanRefinement::AssociateScanPairsLogN(){
    std::vector<std::pair<int,int> > scan_pairs;
    for(auto itr_from = poses_.begin() ; itr_from != std::prev(poses_.end()) ; itr_from++){
        for(auto itr_to = std::next(itr_from) ; itr_to != poses_.end() ; itr_to++){
            //if(std::distance(itr_from,itr_to) % 2 ==0 || std::distance(itr_from,itr_to) == 1){ // 1 2 4 8 // optional - reduce from n^2 to log(n) where n is number of scans
            if( !(locked_[itr_from->first] && locked_[itr_to->first]) ){ // if not both locked
                scan_pairs.push_back(std::make_pair(itr_from->first, itr_to->first));
            }
        }
    }
    return scan_pairs;
    //cout << "scan pairs: " << scan_pairs.size() << endl;
}

std::vector<Correspondance> NScanRefinement::FindCorrespondences(const int scan_i, const int scan_j){
    std::vector<Correspondance> correspondances;
    const NormalCloud::Ptr cloud_i = transformed_[scan_i];
    const NormalCloud::Ptr cloud_j = transformed_[scan_j];
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtree = kdtree_[scan_j];
    const double max_dist = par_.max_dist_association;
    const double max_dist_squared = max_dist*max_dist;
    const double max_planar_distance = max_dist/2.0;

    const double sigma = 1.0 - std::cos(10*M_PI/180.0); // clear decay already at 10 degrees
    const double two_sigma_sqr = 2*sigma*sigma;

    int found = 0;
    int accept = 0;
    for(size_t i = 0 ; i < cloud_i->points.size() ; i++){
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        const pcl::PointXYZINormal& p_i = cloud_i->points[i];

        if(kdtree->nearestKSearch(p_i, 1, pointSearchInd, pointSearchSqDis) > 0){
            if(pointSearchSqDis.front() > max_dist_squared){ // control distance
                continue;
            }

            found++;
            const int j = pointSearchInd.front();
            const pcl::PointXYZINormal& p_j = cloud_j->points[j];
            const double dot(p_i.normal_x*p_j.normal_x + p_i.normal_y*p_j.normal_y + p_i.normal_z*p_j.normal_z);
            const double distance = 1 -(0.5 + 0.5*dot); // from 0 to 1 ( 0 similar, 1 big error
            if( distance < 0.2){ // ~30 deg dot(v1,v2)= 1 - 0.15 = 0.85, arccos(0.85) =~30deg
                const double d = -(distance*distance)/two_sigma_sqr;
                const double weight = exp(d);
                //std::cout << "d: " <<dot<< ", s: " << distance << ", w: " << weight << std::endl;
                const Eigen::Vector3d dst_normal(p_j.normal_x, p_j.normal_y, p_j.normal_z);
                const Eigen::Vector3d dst_pnt(p_j.x, p_j.y, p_j.z);
                const Eigen::Vector3d src_pnt(p_i.x, p_i.y, p_i.z);
                const double point_to_line = (src_pnt - dst_pnt).dot(dst_normal);
                if(point_to_line < max_planar_distance){
                    Correspondance c{scan_i, int(i), scan_j, int(j), weight};
                    /*if(i%1000==0){
                        cout << c.src_scan << ", " << c.src_idx << ", " <<c.target_scan << ", " <<c.target_idx << endl;
                    }*/
                    correspondances.push_back(std::move(c));
                    accept++;
                }
            }
        }
    }
    //cout <<"tot: " << cloud_i->size() << ", found: " << found << ", accept: " << accept << endl;
    return correspondances;
}
void NScanRefinement::TransformCommonFrame(const std::map<int,NormalCloud::Ptr>& input, std::map<int,NormalCloud::Ptr>& output, const bool compute_kdtree){
    const auto& input_set(input);
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        const auto h_transformed = NormalCloud().makeShared();
        const auto& h_vel = velocities_.at(idx);
        const auto& h_time = stamps_.at(idx);
        const auto& h_points = input_set.at(idx);
        const Eigen::Vector3d& t = poses_.at(idx).p;
        const Eigen::Quaterniond& q = poses_.at(idx).q;

        for(size_t i = 0 ; i < h_points->size() ; i++){
            const double time = h_time[i];
            const auto pnt = (h_points->points[i]);
            const Eigen::Vector3d p = Eigen::Vector3d(pnt.x, pnt.y, pnt.z);
            const Eigen::Vector3d v_comp = h_vel.p*time;
            const Eigen::Vector3d p_transformed = q*(p+v_comp) + t; // rigid transform
            const Eigen::Vector3d normal(pnt.normal_x, pnt.normal_y, pnt.normal_z);
            const Eigen::Vector3d normal_transformed = q*normal;
            h_transformed->push_back(EigToPnt(p_transformed, normal_transformed, h_points->points[i].intensity));
            //if(i%100==0)
            //    cout << p_transformed.transpose() << ", " << normal_transformed.transpose() << endl;
        }
        output[idx] = h_transformed;
        if(compute_kdtree){
            kdtree_[idx] = pcl::KdTreeFLANN<pcl::PointXYZINormal>().makeShared();
            kdtree_[idx]->setInputCloud(transformed_[idx]);
        }
    }
}

void NScanRefinement::addSurfCostFactor(const Correspondance& c, ceres::Problem& prob){

    //j destination, i source
    if(c.src_scan == c.target_scan){
        cerr << "PROBLEM SELF-ASSOCIATION" << endl;
    }
    const Eigen::Vector3d src_pnt = PntToEig(filtered_[c.src_scan]->points[c.src_idx]);     // // verify src and  target
    const Eigen::Vector3d dst_pnt = PntToEig(filtered_[c.target_scan]->points[c.target_idx]);
    const Eigen::Vector3d dst_normal = NormalToEig(filtered_[c.target_scan]->points[c.target_idx]);

    /*const Eigen::Quaterniond q_src(poses_[c.src_scan].q.coeffs().data());
    const Eigen::Quaterniond q_dst(poses_[c.target_scan].q.coeffs().data());
    const Eigen::Vector3d p_dst(poses_[c.target_scan].p.data());
    const Eigen::Vector3d p_src(poses_[c.src_scan].p.data());*/

    Eigen::Vector3d src_pnt_transf = PntToEig(transformed_[c.src_scan]->points[c.src_idx]);
    Eigen::Vector3d dst_pnt_transf = PntToEig(transformed_[c.target_scan]->points[c.target_idx]);
    Eigen::Vector3d dst_normal_transf = NormalToEig(transformed_[c.target_scan]->points[c.target_idx]);

    const double distance = (dst_pnt_transf - src_pnt_transf).dot(dst_normal_transf);
    if(isnan(distance)){
        cout << "Error - is nan" << distance << endl;
    }

    //cout <<"src: " << src_pnt.transpose() << ", dst_pnt: " << dst_pnt.transpose() <<", dst_pnt: " << dst_normal.transpose() << endl;
    const double t_src = stamps_[c.src_scan][c.src_idx];
    const double t_target = stamps_[c.target_scan][c.target_idx];

    ceres::CostFunction* cost_function = PointToPlaneErrorGlobalTime::Create(dst_pnt, src_pnt, dst_normal, t_src, t_target); // CHANGE THIS THIS IS WROOOOOOOOOOOOONG
    //ceres::CostFunction* cost_function = PointToPlaneErrorGlobal::Create(dst_pnt, src_pnt, dst_normal); // CHANGE THIS THIS IS WROOOOOOOOOOOOONG
    //ceres::CostFunction* cost_function = PointToPlaneErrorGlobal::Create(dst_pnt, src_pnt, dst_normal); // CHANGE THIS THIS IS WROOOOOOOOOOOOONG
    ceres::LossFunction* loss = nullptr;
    if(par_.loss == "huber")
        loss = new ceres::HuberLoss(0.1);
    else
        loss = new ceres::CauchyLoss(0.01);

    ceres::LossFunction* scaled_loss = new ceres::ScaledLoss(loss, c.weight, ceres::TAKE_OWNERSHIP);
    if(nr_residual++% 1000 == 0){
        //std::cout << "Add res: " << distance << std::endl;
        //cout <<"residual: " << c.src_scan <<" - "<< c.target_scan << endl;
        //cout <<"pose, q: " << poses_[c.src_scan].q.coeffs().transpose() << ", " << poses_[c.target_scan].p.transpose() << endl;
        //cout << "c.src_idx: " << c.src_idx << ", " << c.target_idx << endl;
        //cout << src_pnt.transpose() << ", " << dst_pnt.transpose() << ", " << dst_normal.transpose() << endl;
        //cout <<"block: "<< poses_[c.src_scan].q.coeffs().data()<<", " << poses_[c.src_scan].p.data() << ", " <<poses_[c.target_scan].q.coeffs().data() <<", " << poses_[c.target_scan].p.data() << endl;
    }
    prob.AddResidualBlock(cost_function,
                          scaled_loss,
                          poses_[c.src_scan].q.coeffs().data(),
            poses_[c.src_scan].p.data(),
            poses_[c.target_scan].q.coeffs().data(),
            poses_[c.target_scan].p.data(),
            velocities_[c.src_scan].p.data(),
            velocities_[c.target_scan].p.data());
    /*prob.AddResidualBlock(cost_function,
                          scaled_loss,
                          poses_[c.src_scan].q.coeffs().data(),
            poses_[c.src_scan].p.data(),
            poses_[c.target_scan].q.coeffs().data(),
            poses_[c.target_scan].p.data());
*/
}
void NScanRefinement::VisualizePointCloudNormal(std::map<int,NormalCloud::Ptr>& input, const std::string& name){
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.ns = name;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_LIST;

    line_strip.scale.x = 0.005;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
    for (const auto& [index, cloud] : input) {
        for(int i = 0 ; i < cloud->size() ; i++){
            const auto& pnt_1 = cloud->points[i];
            geometry_msgs::Point p1;
            p1.x = pnt_1.x; p1.y = pnt_1.y; p1.z = pnt_1.z;
            geometry_msgs::Point p2;
            p2.x = p1.x+pnt_1.normal_x*0.1;
            p2.y = p1.y+pnt_1.normal_y*0.1;
            p2.z = p1.z+pnt_1.normal_z*0.1;
            line_strip.points.push_back(p1);
            line_strip.points.push_back(p2);
        }
    }
    line_strip.header.stamp = ros::Time::now();
    normal_pub.publish(line_strip);
}

void NScanRefinement::VisualizeCorrespondance(std::vector<Correspondance>& corr){
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_LIST;

    line_strip.scale.x = 0.005;
    line_strip.color.b = 1.0;
    line_strip.color.a = 0.3;

    for(int i = 0 ; i < corr.size() ; i++){
        const auto pnt_1 = transformed_[corr[i].src_scan]->points[corr[i].src_idx];
        const auto pnt_2 = transformed_[corr[i].target_scan]->points[corr[i].target_idx];
        geometry_msgs::Point p1;
        p1.x = pnt_1.x; p1.y = pnt_1.y; p1.z = pnt_1.z;
        geometry_msgs::Point p2;
        p2.x = pnt_2.x; p2.y = pnt_2.y; p2.z = pnt_2.z;
        line_strip.points.push_back(p1);
        line_strip.points.push_back(p2);
    }
    line_strip.header.stamp = ros::Time::now();
    vis_pub.publish(line_strip);
}

/*void NScanRefinement::AddRotationTerm(int idx){
    const auto& imu_q(imu_[idx]);
    Eigen::Quaterniond& pose_q(poses_[idx].q);
    //Eigen::Vector3d variance(1.0/(0.001), 1.0/0.001, 1.0/10000.0);
    Eigen::Vector3d variance(0.00001, 0.00001, 0.000001);
    Eigen::Matrix3d sqrt_inf = variance.asDiagonal();
    ceres::CostFunction* cost_function = dmapping::RotErrorTerm::Create(imu_q, sqrt_inf);
    problem->AddResidualBlock(cost_function, nullptr, pose_q.coeffs().data());
}*/

void NScanRefinement::Solve(std::map<int,Pose3d>& solution, const std::map<int,bool>& locked){
    locked_ = locked;
    Solve(solution);
}
void NScanRefinement::Solve(std::map<int,Pose3d>& solution){

    if(locked_.size() != poses_.size()){
        locked_[poses_.begin()->first] = true; // lock first parameter block
        for(auto itr = std::next(poses_.begin()) ; itr != poses_.end() ; itr++)
            locked_[itr->first] = false; //unlock following parameter blocks
    }
    cout << "Solve - Visualize" << endl;
    Visualize("/after_reg");
    usleep(100*1000);
    Visualize("/after_reg");

    for (int i = 0; i < par_.outer_iterations ; i++) {
        ceres::Problem problem;
        cout << "Compute correspondance" << endl;
        TransformCommonFrame(filtered_, transformed_, true);
        std::vector<std::pair<int,int> > scan_pairs = AssociateScanPairsLogN(); //  {std::make_pair(poses_.begin()->first,std::next(poses_.begin())->first )};
        //cout <<"scan pairs: " << scan_pairs.size() << endl;
        std::vector<Correspondance> correspondances;
        #pragma omp parallel
        {
            #pragma omp single
            {
                //for (auto&& pair : scan_pairs) {
                for (int i = 0 ; i < scan_pairs.size() ; i++) {
                    #pragma omp task
                    {
                        const auto& pair = scan_pairs[i];
                        const int scan_i = pair.first;
                        const int scan_j = pair.second;

                        std::vector<Correspondance> tmp_corr = FindCorrespondences(scan_i, scan_j);
                        //cout << "FindCorrespondences: " << scan_i << "," << scan_j <<", size: " << tmp_corr.size() << endl;
                        #pragma omp critical
                        {
                            correspondances.insert(correspondances.end(), tmp_corr.begin(), tmp_corr.end());
                        }
                    }
                }
            }
        }
        VisualizeCorrespondance(correspondances);
        std::map<int,NormalCloud::Ptr> raw_transformed;
        TransformCommonFrame(surf_, raw_transformed, true);
        VisualizePointCloudNormal(raw_transformed, "normals");


        //cout <<"Total: " << correspondances.size() << endl;
        //cout << "Add residuals" << endl;
        for(auto && c : correspondances){
            addSurfCostFactor(c, problem);
        }
        /*for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
            AddRotationTerm(itr->first);
        }*/
        if(!correspondances.empty()){
            auto pose_first_iter = poses_.begin();

            ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization();
            for(auto itr = poses_.begin() ; itr != poses_.end(); itr++){
                problem.SetParameterization(itr->second.q.coeffs().data(), quaternion_local_parameterization);
                if(locked_[itr->first]){
                    problem.SetParameterBlockConstant(pose_first_iter->second.p.data());
                    problem.SetParameterBlockConstant(pose_first_iter->second.q.coeffs().data());
                    //cout << "locked ";
                }
                else{
                    //cout << "unlocked ";
                }
                //problem->SetParameterBlockConstant(velocities_[itr->first].p.data());
            }

            cout << "Minimize" << endl;
            ceres::Solve(options, &problem, &summary);
            cout << "solved" << endl;

            //cout << summary.FullReport() << endl;
            //cout << "score: " << summary.final_cost / summary.num_residuals << endl;
            //cout << "legit? - " << summary.IsSolutionUsable() << endl;

            //cout << "after solve: " << endl;
            for(auto itr = poses_.begin() ; itr != poses_.end(); itr++){
                const int idx = itr->first;
                //cout << poses_[idx].p.transpose() << " - " <<  poses_[idx].q.coeffs().transpose() << endl;  // velocities_[idx].p.transpose() << endl;;
                const std::string filename =  "/home/daniel/Documents/cloud/cloud_" + std::to_string(itr->first) + ".pcd";
                pcl::io::savePCDFileBinary(filename, *transformed_[itr->first]);
            }
        }

        Visualize("/after_reg");
        //cout << "" << endl;

    }
    solution = poses_;
}

}
