
#include "dmapping_refinement/d_utils.h"

namespace dmapping {

bool KeyFrameUpdate(const Eigen::Isometry3d& delta, const double keyframe_min_transl, const double keyframe_min_rot){
    const double delta_movement = delta.translation().norm();
    const Eigen::Vector3d ea = delta.linear().eulerAngles(0,1,2);//Eigen::AngleAxisd(delta.linear()).angle();
    const double delta_rot = std::max(ea(0),ea(1)); // exclude yaw
    if(delta_movement > keyframe_min_transl || delta_rot > keyframe_min_rot) {
        return true;
    }else{
        return false;
    }
}

boost::shared_ptr<PoseGraph> KeyFrameFilter(boost::shared_ptr<PoseGraph> input, const double keyframe_min_transl, const double keyframe_min_rot, const int max_size){
    boost::shared_ptr<PoseGraph> output = boost::make_shared<PoseGraph>();
    Eigen::Isometry3d Tprev;
    //cout <<"keyframe_min_transl: "<< keyframe_min_transl << ", keyframe_min_rot:" << keyframe_min_rot << ", max_size: " << max_size << endl;
    for(auto itr = std::next(input->nodes.begin()) ; itr != input->nodes.end() ; itr++){
        const Eigen::Isometry3d Tnow = itr->second.T;
        const size_t idx = itr->first;

        if(itr == std::next(input->nodes.begin())){ // if first iteration
            Tprev = Tnow;
            output->nodes[idx] = itr->second;
            output->surfels_[idx] = input->surfels_[idx];
            continue;
        }else{ // if another iteration
            const Eigen::Isometry3d delta =  Tprev.inverse()*Tnow;
            if(KeyFrameUpdate(delta, keyframe_min_transl, keyframe_min_rot)){
                Tprev = Tnow;
                output->nodes[idx] = itr->second;
                output->surfels_[idx] = input->surfels_[idx];
            }
        }
        if(output->nodes.size() == max_size){
            break;
        }
    }
    output->constraints = input->constraints;
    cout << "keyframe_min_transl: " << keyframe_min_transl << endl;
    cout << "keyframe_min_rot: " << keyframe_min_rot << endl;
    cout << "Before filter graph: " <<input->ToString() << endl;
    cout << "After filter graph: " <<output->ToString() << endl;
    return output;
}

NormalCloud::Ptr TransformNonRigid(NormalCloud::Ptr cloud,
                                   const Eigen::Vector3d& t,
                                   const Eigen::Quaterniond& q,
                                   const Eigen::Vector3d& t_prim,
                                   const Eigen::Quaterniond& q_prim){
    NormalCloud::Ptr transformed(new NormalCloud());
    for(int i = 0 ; i < cloud->size() ; i++){
        const double  time = cloud->points[i].curvature;
        const Eigen::Vector3d pnt = PntToEig(cloud->points[i]);
        const Eigen::Vector3d normal = NormalToEig(cloud->points[i]);
        const Eigen::Vector3d v_comp = t_prim*time;
        //const Eigen::Vector3d q_comp = velocities_[idx].q*time;
        const Eigen::Quaterniond qIdentity = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond qNlerp;
        if( time > 0){
            qNlerp = qIdentity.slerp(time, q_prim);
        }else{
            Eigen::Quaterniond q_prim_inv = q_prim.inverse();
            qNlerp = qIdentity.slerp(-time, q_prim_inv);
        }
        //qNlerp.normalize();
        const Eigen::Vector3d pnt_comp = qNlerp*pnt + v_comp;
        const Eigen::Vector3d normal_comp = qNlerp*normal; // add motion compensation


        const Eigen::Vector3d p_transformed = q*pnt_comp + t; // rigid transform
        const Eigen::Vector3d n_transformed = q*normal_comp;// rigid transform
        transformed->push_back(EigToPnt(p_transformed, n_transformed, cloud->points[i].intensity, cloud->points[i].curvature));
    }
    return transformed;
}


void SortTime(pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr cloud){

    std::sort(cloud->begin(),cloud->end(), lessThanKey());

}

SurfelExtraction::SurfelExtraction(pcl::PointCloud<PointType>::Ptr& surf_in, int n_scan_lines, float scan_period) : n_scan_lines_(n_scan_lines), scan_period_(scan_period){
    surf_in_ = surf_in;
    Initialize();
}

void SurfelExtraction::Initialize(){

    ringClouds_.resize(n_scan_lines_);
    times_.resize(n_scan_lines_);
    for(int i = 0 ; i < ringClouds_.size() ; i++){ // initiaize clouds
        ringClouds_[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }
    for(auto&& pnt : surf_in_->points){ // Fill up
        ringClouds_[pnt.ring]->push_back(pnt);
    }
    for(auto && cloud : ringClouds_){ // Sort
        SortTime(cloud);
    }
    for(auto&& cloud : ringClouds_){ // And create the same structure for doubles
        for(auto && pnt : cloud->points){
            times_[pnt.ring].push_back(pnt.time); // Please preallocate next time daniel!!!!
        }
    }

}


std::vector<int> NNSearchArray::findClosestElements(std::vector<double>& arr, int k, float resolution, float query) {

    auto itr_begin = arr.begin();
    auto itr_end = arr.end();
    const int nr_neigbours = (k-1)/2;
    if(nr_neigbours == 0){
        cout << "incorrect usage, k= "<< k <<  endl;
        exit(0);
    }

    std::vector<int> indicies;
    auto it = std::lower_bound(itr_begin, itr_end, query);

    if(it == itr_begin){
        for(auto itr = itr_begin ; itr != itr_begin+k ; itr++){
            indicies.push_back(std::distance(itr_begin,itr));
        }
        return indicies; // return vector<int>(itr_begin(), itr_begin()+k);
    }else if(it == itr_end){
        for(auto itr = itr_end - k ; itr != itr_end ; itr++){
            indicies.push_back(std::distance(itr_begin,itr));
        }
        return indicies; //return vector<int>(itr_end() - k, itr_end());

    }


    const int n = arr.size();
    int idx = it - itr_begin;
    int left, right;
    left = idx - k/2;
    right = left + k;

    // cout << left << ", " << right << " -> ";

    if(left < 0){
        left = 0;
        right = left +k;
    }

    if(right > n){
        right = n;
        left = right - k;
    }

    while(left > 0 && query - arr[left-1] <= arr[right-1] - query){
        right--;
        left--;
    }

    while(right < n && arr[(right-1)+1] - query  < query - arr[left]){
        right++;
        left++;
    }

    for(auto itr = itr_begin + left  ; itr != itr_begin + right ; itr++){
        //cout <<"r: " << ((*itr-query)/resolution) << ", ";
        if(std::round((*itr-query)/resolution)  <= nr_neigbours){
            indicies.push_back(std::distance(itr_begin,itr));
        }
    }
    //cout << nr_neigbours << ", ";
    //cout << indicies.size() << endl;
    return indicies;
}

void SurfelExtraction::Extract(SurfElCloud& surfelCloud){
    cout << "nr_neighbours: " << nr_neighbours << endl;

    for(auto && pnt : surf_in_->points){
        SurfelPointInfo pntSurfEl;
        if(EstimateNormal(pnt, pntSurfEl)){
            surfelCloud.cloud.push_back(std::move(pntSurfEl));
        }
    }
}

void SurfelExtraction::LineNNSearch( const int ring, const double query, int &row, Eigen::MatrixXd& neighbours){

    NNSearchArray NNSearch;
    const int scans_par_line = 1800;
    float hor_time_res = scan_period_/scans_par_line; // max 6 points from center
    //cout << "max tdiff - not implemented yet: " << scans_par_line << std::endl;

    std::vector<int> indicies = NNSearch.findClosestElements(times_[ring], 2*nr_neighbours+1, hor_time_res, query);
    //cout << "nearby: " << indicies.size() << endl;
    if(indicies.size() > 0){
        for(int first_row = row; row< first_row + indicies.size() ; row++){ // i is row in matrix
            //cout << i - first_row << endl;
            const int idx = indicies[row - first_row]; // zero index
            //cout << "idx" << idx << endl;
            const Eigen::Vector3d  pntNear(ringClouds_[ring]->points[idx].x, ringClouds_[ring]->points[idx].y, ringClouds_[ring]->points[idx].z);
            neighbours.block<1,3>(row,0) = pntNear;
            //cout << "neigbour " << neighbours.block<1,3>(i,0).transpose() << endl;
        }
    }
}
bool SurfelExtraction::GetNeighbours(const vel_point::PointXYZIRTC& pnt, Eigen::MatrixXd& neighbours){
    const int ring = pnt.ring;
    const double time = pnt.time;
    std::vector<int> search;
    // not last ring
    neighbours = Eigen::MatrixXd((2*nr_neighbours+1)*3,3);
    int first = 0;

    ///cout << "ring: "<< ring << endl;
    //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;
    LineNNSearch(ring, time,first, neighbours);
    //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;

    if(ring == n_scan_lines_ -1){ // top ring
        LineNNSearch(ring-1, time, first, neighbours);
        LineNNSearch(ring-2, time, first, neighbours);
    }
    else if(ring == 0 ){ // bot ring
        LineNNSearch(ring+1, time, first, neighbours);
        LineNNSearch(ring+2, time, first, neighbours);
    }else{ // all other cases
        LineNNSearch(ring-1, time, first, neighbours);
        LineNNSearch(ring+1, time, first, neighbours);
    }
    neighbours.conservativeResize(first,3);
    //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;

    return true;


}
bool SurfelExtraction::EstimateNormal(const vel_point::PointXYZIRTC& pnt, SurfelPointInfo& surfel){

    //cout << "EstimateNormal" << endl;
    Eigen::MatrixXd X; //neighbours
    const bool statusOK = GetNeighbours(pnt, X); // 3 x Nsamples
    if(!statusOK){
        return false;
    }
    /*pcl::PointCloud<pcl::PointXYZ> cloud, cloud_pnt;

  for(int i = 0 ; i <X.rows() ; i++){
    pcl::PointXYZ p(X(i,0), X(i,1), X(i,2));
    cloud.push_back(p);
  }
  cout << X << endl;
  cout <<"rows: " <<  X.rows() << endl;
  pcl::PointXYZ pnt_xyz(pnt.x, pnt.y, pnt.z);
  cloud_pnt.push_back(pnt_xyz);
  PublishCloud("surf", *surf_in_, "base_link", ros::Time::now() );
  PublishCloud("center", cloud_pnt, "base_link", ros::Time::now() );
  PublishCloud("neighbours", cloud, "base_link", ros::Time::now() );
  */

    //PublishCloud(const std::string& topic, Cloud& cloud, const std::string& frame_id, const ros::Time& t);
    const int Nsamples = X.rows();
    Eigen::Matrix<double,1,3> mean(0,0,0);  // 3 x 1

    for(Eigen::Index i=0 ; i<Nsamples ; i++)
        mean += X.block<1,3>(i,0); // compute sum
    mean/=Nsamples;

    for(Eigen::Index i=0 ; i<Nsamples ; i++) // subtract mean
        X.block<1,3>(i,0) = X.block<1,3>(i,0) - mean;

    const Eigen::Matrix3d cov = 1.0/(Nsamples - 1.0)*X.transpose()*X;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);


    const float l1 = std::sqrt(es.eigenvalues()[0]);  // l1 < l2 < l3
    const float l2 = std::sqrt(es.eigenvalues()[1]);
    const float l3 = std::sqrt(es.eigenvalues()[2]); // remove sqrt!!!
    const float planarity = 1 - (l2 - l1)/(l3); // this should be it when l1 -> 0  & l2/l3 is high  planarity -> 1 if l3 >> l1+l2

    Eigen::Vector3d normal = es.eigenvectors().col(0);

    Eigen::Matrix <double, 3, 1> vp (pnt.x, pnt.y, pnt.z);
    if(vp.dot(normal)> 0) // when looking at point from origin, the normal should not look back at you
        normal *=-1;
    if( fabs(vp.dot(normal)) < 0.1 ) // 0.1 ~= cos(p5
        return false;

    surfel.centerPoint = Eigen::Vector3d(pnt.x, pnt.y, pnt.z);
    //surfel.mean = mean;
    surfel.l3 = l3;
    //surfel.cov = cov;
    surfel.nSamples = Nsamples;
    surfel.planarity = planarity; // this should be it when l1 -> 0  & l2/l3 is high  planarity -> 1 if l3 >> l1+l2
    surfel.normal = normal;
    surfel.entropy = 0.5*log(1 + 2*M_PI*M_E*cov.determinant());
    surfel.intensity = pnt.intensity;
    surfel.time = pnt.time;
    surfel.curvature = pnt.curvature;
    //surfel.intensity = planarity;
    return true;
}

void lineRefinement::Filter(pcl::PointCloud<PointType>::Ptr output, int nr_neigh, double expected_noise){
    output = pcl::PointCloud<PointType>().makeShared();
    for(int i = nr_neigh ; i = 0 < input_->size() - nr_neigh ; i++){// all points

        Eigen::MatrixXd X((2*nr_neigh+1),3); // build matrix
        int rows = 0;
        for(int j = i - nr_neigh ; j < i + nr_neigh ; j++){
            if(j==i)
                continue;
            const Eigen::Vector3d  pntNear(input_->points[j].x, input_->points[j].y, input_->points[j].z);
            X.block<1,3>(rows,0) = pntNear;
        }
        X.conservativeResize(rows,3);

        // subtract mean
        Eigen::Matrix<double,1,3> mean(0,0,0);  // 3 x 1
        for(Eigen::Index i=0 ; i<rows ; i++) // compute mean
            mean += X.block<1,3>(i,0);
        mean/=rows;
        for(Eigen::Index i=0 ; i<rows ; i++) // subtract mean
            X.block<1,3>(i,0) = X.block<1,3>(i,0) - mean;

        Eigen::Matrix<double,3,1> p0;
        Eigen::Matrix<double,3,1> v;
        if(Fit(X, p0, v)){
            cout << "Fit: " << X << ", p0: " << p0 << ", " << v << endl;
        }
    }
}
bool lineRefinement::Fit(const Eigen::Matrix<double,3,1>& X, Eigen::Matrix<double,3,1>& p0, Eigen::Matrix<double,3,1>& v){
    p0 << 0, 0, 0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    ceres::Problem problem;
    for(int i = 0 ; i < X.rows() ; i++){
        const Eigen::Matrix<double,3,1> p = X.block<1,3>(i,0);
    //    ceres::CostFunction* cost_function = LineCostFunctor::Create(p);
    //    problem.AddResidualBlock(cost_function, nullptr, q.coeffs().data(), p0.data());
    }
    //ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization();
    //problem.SetParameterization(q.coeffs().data(), quaternion_local_parameterization);
    // Solve the problem
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    v = q*Eigen::Vector3d(1,0,0);
}
void VisualizePointCloudNormal(std::map<int,NormalCloud::Ptr>& input, const std::string& name, ros::Publisher& pub){
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
    pub.publish(line_strip);
}
Eigen::Isometry3d EigenCombine(const Eigen::AngleAxisd& q, const Eigen::Vector3d& transl){
    Eigen::Matrix4d prediction = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d pred_mat;
    pred_mat.setIdentity();
    pred_mat.block<3,3>(0,0) = q.toRotationMatrix();
    pred_mat.block<3,1>(0,3) = transl;
    return Eigen::Isometry3d(pred_mat);
}
Eigen::Isometry3d EigenCombine(const Eigen::Quaterniond& q, const Eigen::Vector3d& transl){
    Eigen::Matrix4d prediction = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d pred_mat;
    pred_mat.setIdentity();
    pred_mat.block<3,3>(0,0) = q.toRotationMatrix();
    pred_mat.block<3,1>(0,3) = transl;
    return Eigen::Isometry3d(pred_mat);
}

}
