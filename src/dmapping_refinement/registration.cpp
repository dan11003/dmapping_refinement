#include "dmapping_refinement/registration.h"

namespace dmapping {

NScanRefinement::NScanRefinement(Parameters& par, const std::map<int,Pose3d>& poses, std::map<int,NormalCloud::Ptr>& surf, std::map<int,std::vector<double> >& stamps, ros::NodeHandle& nh) : par_(par), poses_(poses), surf_(surf), stamps_(stamps), nh_(nh){
    loss_function = new ceres::CauchyLoss(0.1); // HuberLoss(0.1);
    //options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = par_.inner_iterations;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 6;


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
    NormalCloud::Ptr merged_surf(new NormalCloud());
    const ros::Time t_cloud = ros::Time::now();

    TransformCommonFrame();
    int nr = 0;
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        *merged_surf += *transformed_[idx];
        PublishCloud("registerd_"+std::to_string(nr++), *transformed_[idx], "world", t_cloud, nh_);
        tf::Transform Tf;
        const Eigen::Affine3d T = EigenCombine(itr->second.q, itr->second.p);
        tf::transformEigenToTF(T, Tf);
        trans_vek.push_back(tf::StampedTransform(Tf, t_cloud, "world", "reg_node_"+std::to_string(idx)));
    }
    Tbr.sendTransform(trans_vek);
    PublishCloud(topic, *merged_surf, "world", t_cloud, nh_);
    cout << "Publish size: " << merged_surf->size() << " to " << topic << endl;
}


std::vector<std::pair<int,int> > NScanRefinement::AssociateScanPairsLogN(){
    std::vector<std::pair<int,int> > scan_pairs;
    for(auto itr_from = poses_.begin() ; itr_from != std::prev(poses_.end()) ; itr_from++){
        for(auto itr_to = std::next(itr_from) ; itr_to != poses_.end() ; itr_to++){
            //log n instead of n
            if(std::distance(itr_from,itr_to) % 2 ==0 || std::distance(itr_from,itr_to) == 1){ // 1 2 4 8
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
    const double max_planar_distance = par_.max_dist_association/2;
    const double max_dist_squared = par_.max_dist_association*par_.max_dist_association;
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
void NScanRefinement::TransformCommonFrame(){
    for(auto itr = poses_.begin() ; itr != poses_.end() ; itr++){
        const int idx = itr->first;
        auto h_transformed = NormalCloud().makeShared();
        auto& h_vel = velocities_[idx];
        auto& h_time = stamps_[idx];
        auto& h_points = filtered_[idx];
        Eigen::Vector3d& t = poses_[idx].p;
        Eigen::Quaterniond& q = poses_[idx].q;

        for(size_t i = 0 ; i < h_points->size() ; i++){
            //pcl::transformPointCloudWithNormals(*filtered_[idx], *transformed_[idx], EigenCombine(poses_[idx].q,poses_[idx].p).matrix());
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
        transformed_[idx] = h_transformed;
        kdtree_[idx] = pcl::KdTreeFLANN<pcl::PointXYZINormal>().makeShared();
        kdtree_[idx]->setInputCloud(transformed_[idx]);
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
        cout << "omg is nan" << distance << endl;
    }

    //cout <<"src: " << src_pnt.transpose() << ", dst_pnt: " << dst_pnt.transpose() <<", dst_pnt: " << dst_normal.transpose() << endl;
    const double t_src = stamps_[c.src_scan][c.src_idx];
    const double t_target = stamps_[c.target_scan][c.target_idx];

    //ceres::CostFunction* cost_function = PointToPlaneErrorGlobalTime::Create(dst_pnt, src_pnt, dst_normal, t_src, t_target); // CHANGE THIS THIS IS WROOOOOOOOOOOOONG
    //ceres::CostFunction* cost_function = PointToPlaneErrorGlobal::Create(dst_pnt, src_pnt, dst_normal); // CHANGE THIS THIS IS WROOOOOOOOOOOOONG
    ceres::CostFunction* cost_function = PointToPlaneErrorGlobal::Create(dst_pnt, src_pnt, dst_normal); // CHANGE THIS THIS IS WROOOOOOOOOOOOONG
    ceres::LossFunction* scaled_loss = new ceres::ScaledLoss(loss_function, c.weight, ceres::TAKE_OWNERSHIP);
    if(nr_residual++% 1000 == 0){
        //std::cout << "Add res: " << distance << std::endl;
        //cout <<"residual: " << c.src_scan <<" - "<< c.target_scan << endl;
        //cout <<"pose, q: " << poses_[c.src_scan].q.coeffs().transpose() << ", " << poses_[c.target_scan].p.transpose() << endl;
        //cout << "c.src_idx: " << c.src_idx << ", " << c.target_idx << endl;
        //cout << src_pnt.transpose() << ", " << dst_pnt.transpose() << ", " << dst_normal.transpose() << endl;
        //cout <<"block: "<< poses_[c.src_scan].q.coeffs().data()<<", " << poses_[c.src_scan].p.data() << ", " <<poses_[c.target_scan].q.coeffs().data() <<", " << poses_[c.target_scan].p.data() << endl;
    }
    /*prob.AddResidualBlock(cost_function,
                          scaled_loss,
                          poses_[c.src_scan].q.coeffs().data(),
            poses_[c.src_scan].p.data(),
            poses_[c.target_scan].q.coeffs().data(),
            poses_[c.target_scan].p.data(),
            velocities_[c.src_scan].p.data(),
            velocities_[c.target_scan].p.data());*/
    prob.AddResidualBlock(cost_function,
                          scaled_loss,
                          poses_[c.src_scan].q.coeffs().data(),
            poses_[c.src_scan].p.data(),
            poses_[c.target_scan].q.coeffs().data(),
            poses_[c.target_scan].p.data());

}



void NScanRefinement::Solve(std::map<int,Pose3d>& solution){
    Visualize("/after_reg");
    usleep(100*1000);
    Visualize("/after_reg");

    for (int i = 0; i < par_.outer_iterations ; i++) {
        problem = new ceres::Problem();
        TransformCommonFrame();
        std::vector<std::pair<int,int> > scan_pairs = AssociateScanPairsLogN(); //  {std::make_pair(poses_.begin()->first,std::next(poses_.begin())->first )};
        cout <<"scan pairs: " << scan_pairs.size() << endl;
        std::vector<Correspondance> correspondances;
        for (auto&& pair : scan_pairs) {
            const int scan_i = pair.first;
            const int scan_j = pair.second;

            std::vector<Correspondance> tmp_corr = FindCorrespondences(scan_i, scan_j);
            //cout << "FindCorrespondences: " << scan_i << "," << scan_j <<", size: " << tmp_corr.size() << endl;
            correspondances.insert(correspondances.end(), tmp_corr.begin(), tmp_corr.end());
        }
        cout <<"Total: " << correspondances.size() << endl;
        cout << "Add residuals" << endl;
        for(auto && c : correspondances){
            addSurfCostFactor(c, *problem);
        }
        auto pose_first_iter = poses_.begin();
        problem->SetParameterBlockConstant(pose_first_iter->second.p.data());
        problem->SetParameterBlockConstant(pose_first_iter->second.q.coeffs().data());

        ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization();
        for(auto itr = poses_.begin() ; itr != poses_.end(); itr++){
            problem->SetParameterization(itr->second.q.coeffs().data(), quaternion_local_parameterization);
            //problem->SetParameterBlockConstant(velocities_[itr->first].p.data());
        }

        //problemos.Evaluate(opt, &cost, &residuals, nullptr, nullptr);
        //cout << "cost: " << cost <<", res size: " << residuals.size() << endl;
        cout << "before solve: " << endl;
        for(auto itr = poses_.begin() ; itr != poses_.end(); itr++){
            const int idx = itr->first;
            cout <<"P: " << poses_[idx].p.transpose() << " - q: " <<  poses_[idx].q.coeffs().transpose() << " - v: " << velocities_[idx].p.transpose() << endl;;
        }
        ceres::Solve(options, problem, &summary);

        cout << summary.FullReport() << endl;
        cout << "score: " << summary.final_cost / summary.num_residuals << endl;
        cout << "legit? - " << summary.IsSolutionUsable() << endl;

        cout << "after solve: " << endl;
        for(auto itr = poses_.begin() ; itr != poses_.end(); itr++){
            const int idx = itr->first;
            //cout << poses_[idx].p.transpose() << " - " <<  poses_[idx].q.coeffs().transpose() << endl;  // velocities_[idx].p.transpose() << endl;;
            const std::string filename =  "/home/daniel/Documents/cloud/cloud_" + std::to_string(itr->first) + ".pcd";
            pcl::io::savePCDFileBinary(filename, *transformed_[itr->first]);
        }

        Visualize("/after_reg");
        cout << "finish" << endl;

    }
    solution = poses_;
}

}
