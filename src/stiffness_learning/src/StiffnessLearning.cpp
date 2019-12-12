#include "stiffness_learning/StiffnessLearning.h"

// issues:
//  When data_matrix has not yet reaches the size of the window_length
// The eigenvalues and vectors of the covariance matrix seems wrong?


StiffnessLearning::StiffnessLearning():
nh_("~")
{
    getParameters();
    initializePublishers();
    covariance_matrix_MA_ = initialize2DMultiArray(3, 3, 0);
    stiffness_matrix_MA_ = initialize2DMultiArray(3, 3, 0);
    // clear certain vectors?
}  

void StiffnessLearning::getTF(tf2_ros::Buffer& buffer)
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

void StiffnessLearning::run()
{
    // get error signal from tf_tree
    std::vector<float> error_signal(3);
    error_signal = getErrorSignal(); // x,y,z,qx,qy,qz,qw
    ROS_INFO_STREAM("error_signal: \n"<< error_signal[0]
    << "\n "<< error_signal[1]<< "\n "<< error_signal[2]);

    // data_matrix_ grows by adding the errorsignal until window length
    // in the function only the first three x,y,z translations are used
    populateDataMatrix(error_signal,data_matrix_); 

    // covariance matrix is found from data matrix
    Eigen::Matrix3f covariance_matrix;
    getCovarianceMatrix(data_matrix_, covariance_matrix); 
    ROS_INFO_STREAM("covariance_matrix: \n"<< covariance_matrix);

    // get eigenvalues and eigenvectors via eigen_solver(finds upon initialization)
    // USE Eigen::SelfAdjointEigenSolver 
    // because we are dealing with real symmetric matrix(A=A^T) matrix==selfadjointmatrix
    Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix,true); 
    ROS_INFO_STREAM("EIGENVALUES: \n"<< eigen_solver.eigenvalues());
    ROS_INFO_STREAM("eigenvectors: \n"<< eigen_solver.eigenvectors());


    Eigen::Vector3f stiffness_diagonal;
    getStiffnessEig(eigen_solver,stiffness_diagonal);
    // ROS_INFO_STREAM("stiffness_diagonal: "<< stiffness_diagonal[0]
    //                                     << stiffness_diagonal[1]<< stiffness_diagonal[2]);

    Eigen::Matrix3f K_matrix;
    setStiffnessMatrix(eigen_solver,stiffness_diagonal,K_matrix);
    ROS_INFO_STREAM("K_matrix: "<< "\n" << K_matrix);
    // ROS_INFO_STREAM("K_matrix: "<< "\n" << K_matrix);

    fill2DMultiArray(covariance_matrix,covariance_matrix_MA_);
    fill2DMultiArray(K_matrix,stiffness_matrix_MA_);

    covariance_pub_.publish(covariance_matrix_MA_);
    stiffness_pub_.publish(stiffness_matrix_MA_);
    
    ROS_INFO_STREAM("---------------------------------------------------");
}

std::vector<float> StiffnessLearning::getErrorSignal()
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
void StiffnessLearning::populateDataMatrix(std::vector<float>& complete_error_signal, 
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
void StiffnessLearning::getCovarianceMatrix(std::vector< std::vector<float> >& data_matrix, 
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

void StiffnessLearning::getStiffnessEig(Eigen::EigenSolver<Eigen::Matrix3f> &eigen_solver,
                                         Eigen::Vector3f &stiffness_diagonal)
{
    std::complex<float> E;
    float k;
    
    // Set eignevalues of covariance matrix inversely proportional to stiffness
    for(int i=0; i<stiffness_diagonal.size(); ++i)
    {
        E = eigen_solver.eigenvalues().col(0)[i];

        if( E.imag() != 0){
            ROS_INFO_STREAM("IMAGINAIR!?!?!?! O.o");
        }

        // need to check if negative and close to zero
        float upper = std::pow(10,-6);
        float lower = -1*upper;
        if( E.real() > lower && E.real() < upper ){ 
            E.real() = 0;
        }

        // Check wheter or not I need to take square root? in klas kronander(2012/2014) does
        float lambda = sqrt(E.real());

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
            ROS_INFO_STREAM("Lambda is no real number check eigenvalues covariance matrix");
        }
        stiffness_diagonal(i) = k;
    }
}

void StiffnessLearning::setStiffnessMatrix(Eigen::EigenSolver<Eigen::Matrix3f> &eigen_solver, 
                                            Eigen::Vector3f &stiffness_diagonal,
                                            Eigen::Matrix3f &K_matrix)
{
    Eigen::Matrix3f k_diag, V, Vt;
    k_diag.setIdentity();
        
        for(int i=0; i<3;++i){
            for(int j=0; j<3; ++j){
                V(i,j) = eigen_solver.eigenvectors()(i,j).real();
                Vt(i,j) = eigen_solver.eigenvectors().transpose()(i,j).real();
            }
            k_diag(i,i) = stiffness_diagonal(i);
        }
        

     K_matrix = V * k_diag * Vt;
}
void StiffnessLearning::fill2DMultiArray(Eigen::Matrix3f matrix,
                                                            std_msgs::Float32MultiArray& multi_array)
{
    int height = multi_array.layout.dim[0].size;
    int width = multi_array.layout.dim[1].size;

    std::vector<float> vec(width*height, 0);
    for (int i=0; i<height; i++){
        for (int j=0; j<width; j++){
            vec[i*width + j] = matrix(i,j);
            }
        }
    multi_array.data = vec;
}


void StiffnessLearning::getParameters()
{
    // input output topic names
    nh_.param<std::string>("covariance_topic_name", covariance_command_topic_name_, "/covariance_matrix"); 
    nh_.param<std::string>("stiffness_topic_name", stiffness_command_topic_name_, "/stiffness_command");  
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

void StiffnessLearning::initializePublishers()
{
    covariance_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(covariance_command_topic_name_,1);
    stiffness_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(stiffness_command_topic_name_,1);
}


std_msgs::Float32MultiArray StiffnessLearning::initialize2DMultiArray(int height, int width, int offset)
{
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
    return multi_array;
}



int main( int argc, char** argv )
{
	ros::init(argc, argv, "stiffness_learning");

	StiffnessLearning node;

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