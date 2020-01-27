#include "stiffness_commanding/StiffnessCommanding.h"

// issues:
//  When data_matrix has not yet reaches the size of the window_length
// The eigenvalues and vectors of the covariance matrix seems wrong?


StiffnessCommanding::StiffnessCommanding():
nh_("~")
{
    getParameters();
    initializeSubscribers();
    initializePublishers();
    covariance_matrix_MA_ = initialize2DMultiArray(3, 3, 0, ee_frame_name_);
    stiffness_matrix_MA_ = initialize2DMultiArray(3, 3, 0, ee_frame_name_);
}  


void StiffnessCommanding::getTF(tf2_ros::Buffer& buffer)
{
    // Read as buffer.lookupTransform(target_frame, source_frame, when?, wait x amount of sec untill available):
    // Where the transform can mean two things: 
    //1) find a transform that maps a pose(point/rot) defined in source frame to the target frame
    // OR:
    //2) find the pose(point/rot) of the source_frame as seen from the target frame
    try{
        marker_in_ee_frame_ = buffer.lookupTransform(ee_frame_name_,virtual_marker_name_ ,  
                                ros::Time(0),ros::Duration(0.25) ); 
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
        } 
}


void StiffnessCommanding::run()
{
    
    if (lockstate_msg_.lock_white == true){
        covariance_matrix_MA_ = prev_covariance_matrix_MA_;
        eigen_message_ = prev_eigen_message_;
        stiffness_matrix_MA_ = prev_stiffness_matrix_MA_;
        covariance_pub_.publish(covariance_matrix_MA_);
        eigen_pair_pub_.publish(eigen_message_);
        stiffness_pub_.publish(stiffness_matrix_MA_);
    }
    else{
    
    // get error signal from tf_tree
    std::vector<float> error_signal(3);
    error_signal = getErrorSignal(); // x,y,z,qx,qy,qz,qw
    // ROS_INFO_STREAM("error_signal: \n"<< error_signal[0]
    // << "\n "<< error_signal[1]<< "\n "<< error_signal[2]);

    // data_matrix_ grows by adding the errorsignal until window length
    // in the function only the first three x,y,z translations are used
    populateDataMatrix(error_signal,data_matrix_); 

    // covariance matrix is found from data matrix
    Eigen::Matrix3f covariance_matrix;
    getCovarianceMatrix(data_matrix_, covariance_matrix); 
    // ROS_INFO_STREAM("covariance_matrix: \n"<< covariance_matrix);

    std::pair<Eigen::Matrix3f, Eigen::Vector3f> eigen_vector_and_values =
                                        computeEigenVectorsAndValues(covariance_matrix);
    // ROS_INFO_STREAM("eigenvectors: \n"<< eigen_vector_and_values.first);
    // ROS_INFO_STREAM("EIGENVALUES: \n"<< eigen_vector_and_values.second);

    Eigen::Vector3f stiffness_diagonal = getStiffnessEig(eigen_vector_and_values);
    // ROS_INFO_STREAM("stiffness_diagonal: "<< stiffness_diagonal[0]
    //                                     << stiffness_diagonal[1]<< stiffness_diagonal[2]);

    Eigen::Matrix3f K_matrix = getStiffnessMatrix(eigen_vector_and_values,stiffness_diagonal);
    // ROS_INFO_STREAM("K_matrix: "<< "\n" << K_matrix);

    fill2DMultiArray(covariance_matrix,covariance_matrix_MA_);
    eigen_message_ = setEigenPairMessage(eigen_vector_and_values);
    fill2DMultiArray(K_matrix,stiffness_matrix_MA_);

    covariance_pub_.publish(covariance_matrix_MA_);
    eigen_pair_pub_.publish(eigen_message_);
    stiffness_pub_.publish(stiffness_matrix_MA_);
    
    // ROS_INFO_STREAM("---------------------------------------------------");
    }
    prev_covariance_matrix_MA_ = covariance_matrix_MA_;
    prev_eigen_message_ = eigen_message_;
    prev_stiffness_matrix_MA_ = stiffness_matrix_MA_;
}



std::vector<float> StiffnessCommanding::getErrorSignal()
{
    std::vector<float> error_signal;
    error_signal.push_back(marker_in_ee_frame_.transform.translation.x);
    error_signal.push_back(marker_in_ee_frame_.transform.translation.y);
    error_signal.push_back(marker_in_ee_frame_.transform.translation.z);
    error_signal.push_back(marker_in_ee_frame_.transform.rotation.x);
    error_signal.push_back(marker_in_ee_frame_.transform.rotation.y);
    error_signal.push_back(marker_in_ee_frame_.transform.rotation.z);
    error_signal.push_back(marker_in_ee_frame_.transform.rotation.w);   
    return error_signal;
}


// Test this function for observations > window length
void StiffnessCommanding::populateDataMatrix(std::vector<float>& complete_error_signal, 
                                            std::vector< std::vector<float> >& data_matrix)
{
    // only consider the first 3, x,y,z translations AND NOT THE XYZW QUATERNIONS
    int length_error = 3;
    std::vector<float> error_signal;
    for(int i=0; i<length_error; ++i){
        error_signal.push_back(complete_error_signal[i]);
    }
    
    // check size error_signal
    assert(error_signal.size() == length_error);

    // check size of previous error signal (skip first loop)
    if(data_matrix.empty() == false)//data_matrix != NULL)
    {
        std::vector<float> last_vector = data_matrix.back();
        assert(last_vector.size()== length_error);
    }

    // check if infinite in signal
    if( (std::isinf( error_signal[0]) == true) || (std::isinf(error_signal[1]) == true) 
                                            || (std::isinf(error_signal[2]) == true)) {
        return ;
    }

    int observations = data_matrix.size();

    if(observations < window_length_){
        data_matrix.push_back(error_signal); //+1
    }
    else if(observations == window_length_){
        data_matrix.erase(data_matrix.begin()); //-1
        data_matrix.push_back(error_signal);   //+1
    }
    else if(observations > window_length_){
        int to_remove = (observations-window_length_+1); // remove 1 extra (-1) for pushback later 
        data_matrix.erase( data_matrix.begin(), data_matrix.begin()+to_remove); // remove first part of vector
        data_matrix.push_back(error_signal); // +1
    }
    else{
        ROS_INFO_STREAM("Dit zou niet moeten gebeuren");
    }
}


// Need to check the covariance matrix on singularities otherwise loose rank not good blabla
void StiffnessCommanding::getCovarianceMatrix(std::vector< std::vector<float> >& data_matrix, 
                                                Eigen::Matrix3f& covariance_matrix)
{
    if(data_matrix.empty()){
        return;
    }

    const int height = data_matrix[0].size(); 
    const int length = data_matrix.size();

    Eigen::MatrixXf data_mat_eigen(height,length);

    assert(height==data_mat_eigen.rows());
    assert(length==data_mat_eigen.cols());
    
    // optimize this loop!
    for(int i=0; i<length;++i){
        for(int j=0; j<height; ++j){
            data_mat_eigen(j,i) = data_matrix[i][j];
        }
    }
    
    // find covariance matrix 
    Eigen::MatrixXf centered = data_mat_eigen.colwise() - data_mat_eigen.rowwise().mean();
    // prevent division by zero! by setting n = 0 for observations = 1
    float n = 1;
    if(length == 1){
        n = 0;
    }
    //Find covariance matrix
    covariance_matrix = centered*centered.transpose() / float(data_mat_eigen.cols() - n); 
}


std::pair<Eigen::Matrix3f, Eigen::Vector3f> 
                StiffnessCommanding::computeEigenVectorsAndValues(Eigen::Matrix3f &matrix)
{
    // get eigenvalues and eigenvectors via eigen_solver(finds upon initialization)
    // because we are dealing with real symmetric matrix(A=A^T) matrix==selfadjointmatrix
    Eigen::Matrix3f eigen_vectors = Eigen::Matrix3f::Zero();
    Eigen::Vector3f eigen_values  = Eigen::Vector3f::Identity();
    
    // Compute eigen values and eigen vectors.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(matrix);
    if (eigensolver.info() == Eigen::Success)
    {
        eigen_vectors = eigensolver.eigenvectors();
        eigen_values  = eigensolver.eigenvalues();
        // ROS_INFO_STREAM("Before shuffle eigenvectors: \n"<< eigen_vectors);
        // ROS_INFO_STREAM("Before shuffle EIGENVALUES: \n"<< eigen_values);
    }
    else
    {
        ROS_WARN_THROTTLE(1, "Failed to compute eigen vectors/values. Is the stiffness matrix correct?");
    }

    /* The eigensolver returns random order of vector value pairs
    Assuming that the eigenvectors represent a 3x3 rotation matrix, 
    a total of six orders exist (e.g. 1 (of 6) = [V1,V3,V2][E1,E3,E2])
    The rotation matrix is either a reflection (determinant is -1) which reverses "handedness" 
    or a rotation (determinant is 1) which preserves "handedness".
    Therefore, only 3 are correct and associated with a rotation.*/
    bool right_handed = checkRightHandednessMatrix(eigen_vectors);

    if(right_handed == false)  { // then Shuffle
        std::pair<Eigen::Matrix3f, Eigen::Vector3f> pair = shuffelEigenPairs(eigen_vectors,eigen_values);

        bool check = checkRightHandednessMatrix(pair.first);
        if(check == false){ // shuffle succeeded?
            ROS_INFO_STREAM("shuffle of vectors did not succeed");
        }

        return pair; // return shuffled pair
    }
    else // do nothing
    {
        return std::make_pair(eigen_vectors, eigen_values); // return old pair
    }    
}


bool StiffnessCommanding::checkRightHandednessMatrix(Eigen::Matrix3f &eigen_vectors)
{
    bool right_handed;

    float determinant = eigen_vectors.determinant();
    // ROS_INFO_STREAM("determinant= "<<determinant);

    if(round(determinant) == -1){
        right_handed = false;
    }
    else if(round(determinant) == 1){
        right_handed = true;
    }
    else{
        ROS_WARN_THROTTLE(1, "determinant of eigen_vector matrix is not -1 or 1. Is the matrix correct?");
    }


    // alternative method:
    // Only use this method if matrix is smaller than 4x4! otherwise, 
    // use dot(cros(V1,V2),V3) to determine if right or left handed!
    // Eigen::Vector3f V1 = eigen_vectors.col(0);
    // Eigen::Vector3f V2 = eigen_vectors.col(1);
    // Eigen::Vector3f V3 = eigen_vectors.col(2);
    // Eigen::Vector3f V12_cross = V1.cross(V2);
    // float hand = V12_cross.dot(V3); 
    // if(round(hand) == -1) {
    //     right_handed = false;
    //     ROS_INFO_STREAM("DIRECTION: -1"<<right_handed);
    // }
    // else if(round(hand) == 1){
    //     right_handed = true;
    //     ROS_INFO_STREAM("DIRECTION: 1"<<right_handed);
    // }
    // else{
    //     ROS_WARN_THROTTLE(1, "not 1 or -1 one normalize vector?, should not happen");
    // }

    // ROS_INFO_STREAM("BEFORE RETURN: \n"<<right_handed);
    return right_handed;
}


std::pair<Eigen::Matrix3f, Eigen::Vector3f> StiffnessCommanding::shuffelEigenPairs(
                                                            Eigen::Matrix3f &eigen_vectors, 
                                                            Eigen::Vector3f &eigen_values)
{   
    Eigen::Matrix3f shuffled_vectors;
    Eigen::Vector3f shuffled_values;

    // Shuffle sequence [0=0,1=2,2=1]
    shuffled_vectors.col(0) = eigen_vectors.col(0);
    shuffled_vectors.col(1) = eigen_vectors.col(2);
    shuffled_vectors.col(2) = eigen_vectors.col(1);
    shuffled_values << eigen_values(0),eigen_values(2),eigen_values(1);

    return std::make_pair(shuffled_vectors,shuffled_values);
}


Eigen::Vector3f StiffnessCommanding::getStiffnessEig(std::pair<Eigen::Matrix3f, Eigen::Vector3f>& vector_value_pair)
{
    Eigen::Vector3f eigen_values = vector_value_pair.second;

    Eigen::Vector3f stiffness_diagonal  = Eigen::Vector3f::Zero();
    
    // Set eignevalues of covariance matrix inversely proportional to stiffness
    for(int i=0; i<eigen_values.size(); ++i)
    {
        float E = eigen_values(i);
        // need to check if negative and close to zero
        float upper = std::pow(10,-6);
        float lower = -1*upper;
        if( E > lower && E < upper ){ 
            E = 0;
        }

        // square root for standard deviation
        float lambda = sqrt(E);
        float k;

        if(lambda<=lambda_min_){
            k = stiffness_max_;
        }
        else if(lambda > lambda_min_ && lambda < lambda_max_){
            k = stiffness_max_ - (stiffness_max_ - stiffness_min_)/(lambda_max_ - lambda_min_) * (lambda - lambda_min_);
        }
        else if(lambda>=lambda_max_){
            k = stiffness_min_;
        }
        else{
            ROS_INFO_STREAM("Something went wrong, no idea what");
        }
        stiffness_diagonal(i) = k;
    }
    return stiffness_diagonal;
}


Eigen::Matrix3f StiffnessCommanding::getStiffnessMatrix(
                    std::pair<Eigen::Matrix3f, Eigen::Vector3f>& vector_value_pair, 
                                            Eigen::Vector3f &stiffness_diagonal)
{
    Eigen::Matrix3f K_matrix, k_diag, V, Vt;

    k_diag = Eigen::Matrix3f(stiffness_diagonal.asDiagonal());
    V = vector_value_pair.first;
    Vt = V.transpose();

    K_matrix = V * k_diag * Vt;

    return K_matrix;
}


void StiffnessCommanding::fill2DMultiArray(Eigen::Matrix3f matrix,
                                                            stiffness_commanding::HeaderFloat32MultiArray& multi_array)
{
    int height = multi_array.F32MA.layout.dim[0].size;
    int width = multi_array.F32MA.layout.dim[1].size;

    std::vector<float> vec(width*height, 0);
    for (int i=0; i<height; i++){
        for (int j=0; j<width; j++){
            vec[i*width + j] = matrix(i,j);
            }
        }
    multi_array.F32MA.data = vec;
}


stiffness_commanding::EigenPairs StiffnessCommanding::setEigenPairMessage(std::pair<Eigen::Matrix3f, 
													        Eigen::Vector3f>& vector_value_pair)
{
    Eigen::Matrix3f eigen_vectors = vector_value_pair.first;
    Eigen::Vector3f eigen_values = vector_value_pair.second;
    
    stiffness_commanding::EigenPairs message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = ee_frame_name_; 

    for (int i=0; i<3; i++)
    {
        stiffness_commanding::VectorValue single_pair;

        std::vector<float> Vi;
        Eigen::Vector3f eigen_vector_i = eigen_vectors.col(i);

        Vi.resize(eigen_vector_i.size());
        Eigen::Vector3f::Map(&Vi[0], eigen_vector_i.size()) = eigen_vector_i;

        single_pair.eigen_value = eigen_values(i);
        single_pair.eigen_vector = Vi;

        message.pairs.push_back(single_pair);
    }

    return message;
}


void StiffnessCommanding::getParameters()
{
    // input output topic names
    nh_.param<std::string>("covariance_topic_name", covariance_command_topic_name_, "/covariance_matrix"); 
    nh_.param<std::string>("eigen_pair_topic_name", eigen_pair_topic_name_, "/eigen_pair"); 
    nh_.param<std::string>("stiffness_topic_name", stiffness_command_topic_name_, "/stiffness_command");  
    nh_.param<std::string>("lock_state_topic_name", lock_state_topic_name_, "/omni1_lock_state");
    // TF frame names
    nh_.param<std::string>("ee_frame_name", ee_frame_name_, "end_effector"); 
    nh_.param<std::string>("virtual_marker_name", virtual_marker_name_, "virtual_marker_transform");
    // Tune parameters
    nh_.param<float>("stiffness_min", stiffness_min_, 0); 
    nh_.param<float>("stiffness_max", stiffness_max_, 1000); 
    nh_.param<float>("lambda_min", lambda_min_, 0.01); 
    nh_.param<float>("lambda_max", lambda_max_, 0.45); 
    nh_.param<float>("window_length", window_length_, 100); 
}

void StiffnessCommanding::initializeSubscribers()
{
    lock_state_sub_ = nh_.subscribe(lock_state_topic_name_, 1, &StiffnessCommanding::CB_getLockState, this);
}

void StiffnessCommanding::CB_getLockState(const phantom_omni::LockState& lockstate_message)
{
    lockstate_msg_ = lockstate_message;
}

void StiffnessCommanding::initializePublishers()
{
    covariance_pub_ = nh_.advertise<stiffness_commanding::HeaderFloat32MultiArray>(covariance_command_topic_name_,1);
    eigen_pair_pub_ = nh_.advertise<stiffness_commanding::EigenPairs>(eigen_pair_topic_name_,1);
    stiffness_pub_ = nh_.advertise<stiffness_commanding::HeaderFloat32MultiArray>(stiffness_command_topic_name_,1);
}


stiffness_commanding::HeaderFloat32MultiArray StiffnessCommanding::initialize2DMultiArray(int height, int width, int offset, std::string frame_id )
{
    stiffness_commanding::HeaderFloat32MultiArray message;

    message.header.stamp = ros::Time::now();
    message.header.frame_id = frame_id; 

    std_msgs::Float32MultiArray multi_array;
    multi_array.layout.dim.push_back(std_msgs::MultiArrayDimension()); // height
    multi_array.layout.dim.push_back(std_msgs::MultiArrayDimension()); // width
    multi_array.layout.dim[0].label = "height";
    multi_array.layout.dim[1].label = "width";
    multi_array.layout.dim[0].size = height; 
    multi_array.layout.dim[1].size = width;
    multi_array.layout.dim[0].stride = height*width;
    multi_array.layout.dim[1].stride = width;
    multi_array.layout.data_offset = 0;

    message.F32MA = multi_array;

    return message;
}



int main( int argc, char** argv )
{
	ros::init(argc, argv, "stiffness_commanding");

	StiffnessCommanding node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener TF_listener(tfBuffer);
  
    while (ros::ok())
    {
        ros::Rate loop_rate(30);
        node.getTF(tfBuffer);
        node.run();

        ros::spinOnce();
        loop_rate.sleep();
    }
}