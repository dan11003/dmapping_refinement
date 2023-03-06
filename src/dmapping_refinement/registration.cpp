#include "dmapping_refinement/registration.h"

namespace dmapping {

NScanRefinement::NScanRefinement(Parameters& par, std::map<int,Pose3d>& poses, std::map<int,NormalCloud::Ptr>& surf) : par_(par), poses_(poses), surf_(surf){
    loss_function = new ceres::HuberLoss(0.1);
    //options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 10;//par_.inner_iterations;
    options.minimizer_progress_to_stdout = false;


    //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    // Filter
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        filtered_[idx] = NormalCloud().makeShared();
        pcl::VoxelGrid<pcl::PointXYZINormal> sor;
        sor.setInputCloud(surf_[idx]);
        sor.setLeafSize (0.05f, 0.05f, 0.05f);
        sor.filter (*filtered_[idx]);
        cout <<"Downsampleing rate: " << (double)filtered_[idx]->size() / surf_[idx]->size()  << endl;
    }
    //Create Eigen representation
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

void NScanRefinement::Visualize(){
    static tf::TransformBroadcaster Tbr;
    std::vector<tf::StampedTransform> trans_vek;
    NormalCloud::Ptr merged_surf(new NormalCloud());
    const ros::Time t = ros::Time::now();
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        NormalCloud tmp_surf;
        pcl::transformPointCloud(*surf_[idx], tmp_surf, itr->second.p, itr->second.q);
        *merged_surf += tmp_surf;
        tf::Transform Tf;
        const Eigen::Affine3d T = EigenCombine(itr->second.q, itr->second.p);
        tf::transformEigenToTF(T, Tf);
        trans_vek.push_back(tf::StampedTransform(Tf, t, "world", "reg_node_"+std::to_string(itr->first)));
    }
    Tbr.sendTransform(trans_vek);
    PublishCloud("/reg_surf", *merged_surf, "world", t);
    cout << "Publish size: " << merged_surf->size() << endl;
}


std::vector<std::pair<int,int> > NScanRefinement::AssociateScanPairsLogN(){
    std::vector<std::pair<int,int> > scan_pairs;
    for(auto itr_from = poses_.begin() ; itr_from != std::prev(poses_.end()) ; itr_from++){
        for(auto itr_to = std::next(itr_from) ; itr_to != poses_.end() ; itr_to++){
            //log n instead of n
            if(/*std::distance(itr_from,itr_to) % 2 ==0 ||*/ std::distance(itr_from,itr_to) == 1){ // 1 2 4 8
                scan_pairs.push_back(std::make_pair(itr_from->first, itr_to->first));
            }
        }
    }
    return scan_pairs;
    cout << "scan pairs: " << scan_pairs.size() << endl;
}

std::vector<Correspondance> NScanRefinement::FindCorrespondences(const int scan_i, const int scan_j){
    std::vector<Correspondance> correspondances;
    const NormalCloud::Ptr cloud_i = transformed_[scan_i];
    const NormalCloud::Ptr cloud_j = transformed_[scan_j];
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtree = kdtree_[scan_j];
    const double max_dist = 2;
    const double max_planar_distance = max_dist/2;
    const double max_dist_squared = max_dist*max_dist;
    const double sigma = 1.0 - std::cos(10*M_PI/180.0); // clear decay already at 10 degrees
    const double two_sigma_sqr = 2*sigma*sigma;
    int found = 0;
    int accept = 0;
    for(size_t i = 0 ; i < cloud_i->points.size() ; i++){
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        const pcl::PointXYZINormal& p_i = cloud_i->points[i];

        if(kdtree->nearestKSearch(p_i, max_dist_squared, pointSearchInd, pointSearchSqDis)){
            if(pointSearchSqDis.front() > max_dist_squared){ // control distance
                continue;
            }
            found++;
            const int j = pointSearchInd.front();
            const pcl::PointXYZINormal& p_j = cloud_j->points[j];
            const double dot(p_i.normal_x*p_j.normal_x + p_i.normal_y*p_j.normal_y + p_i.normal_z*p_j.normal_z);
            const double distance = 1 -(0.5 + 0.5*dot); // from 0 to 1 ( 0 similar, 1 big error
            if( distance < 0.3){ // ~30 deg dot(v1,v2)= 1 - 0.15 = 0.85, arccos(0.85) =~30deg
                const double d = -(distance*distance)/two_sigma_sqr;
                //const double weight = exp(d);
                //std::cout << "d: " <<dot<< ", s: " << distance << ", w: " << weight << std::endl;
                const Eigen::Vector3d dst_normal(p_j.normal_x, p_j.normal_y, p_j.normal_z);
                const Eigen::Vector3d dst_pnt(p_j.x, p_j.y, p_j.z);
                const Eigen::Vector3d src_pnt(p_i.x, p_i.y, p_i.z);
                const double point_to_line = (src_pnt - dst_pnt).dot(dst_normal);
                if(point_to_line < max_planar_distance){
                    //cout <<point_to_line <<", ";

                    Correspondance c{scan_i, int(i), scan_j, int(j), 1.0/*weight*/};
                    if(i%1000==0){
                        cout << c.src_scan << ", " << c.src_idx << ", " <<c.target_scan << ", " <<c.target_idx << endl;
                    }

                    correspondances.push_back(std::move(c));
                    accept++;
                }
            }
        }
    }
    cout <<"tot: " << cloud_i->size() << ", found: " << found << ", accept: " << accept << endl;
    return correspondances;
}
void NScanRefinement::TransformCommonFrame(){
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        transformed_[idx] = NormalCloud().makeShared();
        pcl::transformPointCloudWithNormals(*filtered_[idx], *transformed_[idx], EigenCombine(poses_[idx].q,poses_[idx].p).matrix());
        kdtree_[idx] = pcl::KdTreeFLANN<pcl::PointXYZINormal>().makeShared();
        kdtree_[idx]->setInputCloud(transformed_[idx]);
    }
}

void NScanRefinement::addSurfCostFactor(const Correspondance& c, ceres::Problem& problem){

    //j destination, i source
    if(c.src_scan == c.target_scan){
        cerr << "PROBLEM SELF-ASSOCIATION" << endl;
    }
    const Eigen::Vector3d src_pnt = PntToEig(filtered_[c.src_scan]->points[c.src_idx]);     // // verify src and  target
    const Eigen::Vector3d dst_pnt = PntToEig(filtered_[c.target_scan]->points[c.target_idx]);
    const Eigen::Vector3d dst_normal = NormalToEig(filtered_[c.target_scan]->points[c.target_idx]);
    //cout <<"src: " << src_pnt.transpose() << ", dst_pnt: " << dst_pnt.transpose() <<", dst_pnt: " << dst_normal.transpose() << endl;

    ceres::CostFunction* cost_function = PointToPlaneErrorGlobal::Create(dst_pnt, src_pnt, dst_normal); // CHANGE THIS THIS IS WROOOOOOOOOOOOONG
    //problem.AddResidualBlock(cost_function, loss, srcQ.coeffs().data(),srcT.data(),dstQ.coeffs().data(), dstT.data());
    //ceres::LossFunction* scaled_loss = new ceres::ScaledLoss(loss_function, c.weight, ceres::TAKE_OWNERSHIP);
    if(nr_residual++% 1000 == 0){
        cout <<"residual: " << c.src_scan <<" - "<< c.target_scan << endl;
        cout << src_pnt.transpose() << ", " << dst_pnt.transpose() << ", " << dst_normal.transpose() << endl;
        cout <<"block: "<< poses_[c.src_scan].q.coeffs().data()<<", " << poses_[c.src_scan].p.data() << ", " <<poses_[c.target_scan].q.coeffs().data() <<", " << poses_[c.target_scan].p.data() << endl;
    }


    problem.AddResidualBlock(cost_function,
                             new ceres::HuberLoss(0.1),
                             poses_[c.src_scan].q.coeffs().data(),
            poses_[c.src_scan].p.data(),
            poses_[c.target_scan].q.coeffs().data(),
            poses_[c.target_scan].p.data() );

    //eigen_quaternion::EigenQuaternionParameterization *quaternion_parameterization = new eigen_quaternion::EigenQuaternionParameterization;
    // problem.SetParameterization(&cameras[i*7],quaternion_parameterization);
    //problem.AddResidualBlock(cost_function, loss_function, parameters);
}


void NScanRefinement::Solve(){

    for (int i = 0; i < par_.inner_iterations ; i++) {
        Visualize();

        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        cout << "Transform common frame" << endl;
        TransformCommonFrame();
        cout << "Which scans to match?" << endl;
        std::vector<std::pair<int,int> > scan_pairs = AssociateScanPairsLogN();
        cout << scan_pairs.size() << endl;
        std::vector<Correspondance> correspondances;
        for (auto&& pair : scan_pairs) {
            const int scan_i = pair.first;
            const int scan_j = pair.second;

            std::vector<Correspondance> tmp_corr = FindCorrespondences(scan_i, scan_j);
            cout << "FindCorrespondences: " << scan_i << "," << scan_j <<", size: " << tmp_corr.size() << endl;
            correspondances.insert(correspondances.end(), tmp_corr.begin(), tmp_corr.end());
        }
        cout <<"Total: " << correspondances.size() << endl;
        cout << "Add residuals" << endl;
        for(auto && c : correspondances){
            addSurfCostFactor(c, problem);
        }
        auto pose_first_iter = poses_.begin();
        problem.SetParameterBlockConstant(pose_first_iter->second.p.data());
        problem.SetParameterBlockConstant(pose_first_iter->second.q.coeffs().data());
        ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

        for(auto itr = poses_.begin() ; itr != poses_.end(); itr++){
            problem.SetParameterization(itr->second.q.coeffs().data(), quaternion_local_parameterization);
        }
        cout <<"Solve" << endl;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        cout << summary.FullReport() << endl;
        cout << "legit? - " << summary.IsSolutionUsable() << endl;
        Visualize();
    }
}

}
