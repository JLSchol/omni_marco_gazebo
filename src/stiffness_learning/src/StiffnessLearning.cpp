#include "stiffness_learning/StiffnessLearning.h"

// typedef Eigen CUSTOM_TYPE;


StiffnessLearning::StiffnessLearning():
nh_("~")
{
    ROS_INFO_STREAM("----------------------------------");
    getParameters();
    initializeSubscribers();
    initializePublishers();
    initializeStiffnessMsg();
}  


void StiffnessLearning::run()
{
  ROS_INFO_STREAM("shit is aan het runnen");
  
  std::vector<float> error_signal(3);
  getErrorSignal(error_signal); 

  populateDataMatrix(error_signal,data_matrix_); 
  
  Eigen::Matrix3f covariance_matrix;
  getCovarianceMatrix(data_matrix_, covariance_matrix);

  // get eigenvalues and eigenvectors
  Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix,true);
  ROS_INFO_STREAM("EIGENVALUES:");
  ROS_INFO_STREAM(eigen_solver.eigenvalues());
  ROS_INFO_STREAM("eigenvectors:");
  ROS_INFO_STREAM(eigen_solver.eigenvectors().col(0));
  ROS_INFO_STREAM(eigen_solver.eigenvectors().col(1));
  ROS_INFO_STREAM(eigen_solver.eigenvectors().col(2));
  ROS_INFO_STREAM("---------------------------------------------------");

//   getEigenValues()

//   covariance_matrix.compu

//   getEigenVectors();

//   getStiffnessEig();

//   setStiffnessMatrix();

  fillStiffnessMsg();

  stiffness_pub_.publish(stiffness_matrix_);
}


void StiffnessLearning::getErrorSignal(std::vector<float>& error_signal)
{
    getTF();
    std::vector<float> X_actual;
    X_actual.push_back(base_to_ee_.getOrigin().x());
    X_actual.push_back(base_to_ee_.getOrigin().y());
    X_actual.push_back(base_to_ee_.getOrigin().z());

    std::vector<float> X_marker; 
    X_marker.push_back(marker_transform_.transform.translation.x);
    X_marker.push_back(marker_transform_.transform.translation.y);
    X_marker.push_back(marker_transform_.transform.translation.z);

    int size = error_signal.size();
    for(int i=0; i<size; ++i){
        error_signal[i] = X_marker[i] - X_actual[i];
    }
}


void StiffnessLearning::populateDataMatrix(std::vector<float>& error_signal, std::vector< std::vector<float> >& data_matrix)
{
    // check size of data matrix
    if( (std::isinf( error_signal[0]) == true) || (std::isinf(error_signal[2]) == true) || (std::isinf(error_signal[2]) == true)) {
        // ROS_INFO_STREAM("inif: "<<error_signal[0]<<error_signal[1]<<error_signal[2]);
        return ;
    }
    // ROS_INFO_STREAM("Not in if: "<<error_signal[0]);

    int observations = data_matrix.size();

    if(observations < window_length_){
        data_matrix.push_back(error_signal); //+1
        ROS_INFO_STREAM("IF<<<");
    }
    else if(observations == window_length_){
        data_matrix.erase(data_matrix.begin()); //-1
        data_matrix.push_back(error_signal);   //+1
        ROS_INFO_STREAM("IF======");
    }
    else if(observations > window_length_){
        int to_remove = (observations-window_length_+1); // remove 1 extra (-1) for pushback later 
        data_matrix.erase( data_matrix.begin(), data_matrix.begin()+to_remove); // remove first part of vector
        data_matrix.push_back(error_signal); // +1
        ROS_INFO_STREAM("IF>>>>>>>>>");
    }
    else{
        ROS_INFO_STREAM("Dit zou niet moeten gebeuren");
    }
}

void StiffnessLearning::getTF()
{
    if (TF_listener_.waitForTransform(ee_frame_name_, base_frame_name_, ros::Time(0), ros::Duration(0.25)))
    {
        TF_listener_.lookupTransform(base_frame_name_, ee_frame_name_,ros::Time(0), base_to_ee_);
    }
}

void StiffnessLearning::getCovarianceMatrix(std::vector< std::vector<float> >& data_matrix, Eigen::Matrix3f& covariance_matrix)
{
    if(data_matrix.empty()){
        ROS_INFO_STREAM("hoihoi");
        return;
    }

    const int height = data_matrix[0].size(); 
    const int length = data_matrix.size();

    Eigen::MatrixXf data_mat_eigen(height,length);

    assert(height==data_mat_eigen.rows());
    assert(length==data_mat_eigen.cols());
    
    ROS_INFO_STREAM("LENGTH:"<< length);
    for(int i=0; i<length;++i){
        for(int j=0; j<height; ++j){
            data_mat_eigen(j,i) = data_matrix[i][j];
        }
    }
    
    Eigen::MatrixXf centered = data_mat_eigen.colwise() - data_mat_eigen.rowwise().mean();
    ROS_INFO_STREAM("SIZE:"<< centered.rows()<<centered.cols());

    float n = 1;
    if(length == 1){
        n = 0;
    }

    covariance_matrix = centered*centered.transpose() / float(data_mat_eigen.cols() - n); 

    ROS_INFO_STREAM("SIZE" << covariance_matrix.rows()<<covariance_matrix.cols());   
}

// void StiffnessLearning::getEigenValues(Eigen::Matrix3f& covariance_matrix,)
// {

// }

// void StiffnessLearning::getEigenVectors(Eigen::Matrix3f& covariance_matrix,)
// {
    
// }

void StiffnessLearning::fillStiffnessMsg()
{
    int height = stiffness_matrix_.layout.dim[0].size;//no matiching functon
    int width = stiffness_matrix_.layout.dim[1].size;
    // ROS_INFO_STREAM("HEIGHT:" << height);
    
    //Create eigen matrix 3x3
    Eigen::Matrix3f stiffness;
    float counter = 0;
    for (int i=0; i<height; ++i){
        for (int j=0; j<width; ++j){
            stiffness(i,j) = counter;
            counter++;
        }
    }

    // use 3x3 eigenmatrix to fill data from msg
    std::vector<float> vec(width*height, 0);
    for (int i=0; i<height; i++){
        for (int j=0; j<width; j++){
            vec[i*width + j] = stiffness(i,j);
        }
    }
    stiffness_matrix_.data = vec;
}

void StiffnessLearning::getParameters()
{
    // input output topic names
    nh_.param<std::string>("marker_topic_name", marker_trans_topic_name_, "/marker_transform");
    nh_.param<std::string>("stiffness_topic_name", stiffness_command_topic_name_, "/stiffness_command");  
    // TF frame names
    nh_.param<std::string>("base_frame_name", base_frame_name_, "base_marco"); 
    nh_.param<std::string>("ee_frame_name", ee_frame_name_, "end_effector"); 
    // Tune parameters
    nh_.param<float>("stiffness_min", stiffness_min_, 100); 
    nh_.param<float>("stiffness_max", stiffness_max_, 500); 
    nh_.param<float>("lambda_min", lambda_min_, 5); 
    nh_.param<float>("lambda_max", lambda_max_, 25); 
    nh_.param<float>("window_length", window_length_, 200); 
}


void StiffnessLearning::initializeSubscribers()
{
    marker_sub_ = nh_.subscribe(marker_trans_topic_name_, 1, &StiffnessLearning::CB_getMarkerTransform, this);
}

void StiffnessLearning::initializePublishers()
{
    stiffness_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(stiffness_command_topic_name_,1);
}

void StiffnessLearning::initializeStiffnessMsg()
{
    int height = 3;
    int width = 3;
    stiffness_matrix_.layout.dim.push_back(std_msgs::MultiArrayDimension()); // height
    stiffness_matrix_.layout.dim.push_back(std_msgs::MultiArrayDimension()); // width
    stiffness_matrix_.layout.dim[0].label = "height";
    stiffness_matrix_.layout.dim[1].label = "width";
    stiffness_matrix_.layout.dim[0].size = height; 
    stiffness_matrix_.layout.dim[1].size = width;
    stiffness_matrix_.layout.dim[0].stride = height*width;
    stiffness_matrix_.layout.dim[1].stride = width;
    stiffness_matrix_.layout.data_offset = 0;
}


void StiffnessLearning::CB_getMarkerTransform(const geometry_msgs::TransformStamped& marker_transform_message)
{
    marker_transform_ = marker_transform_message;
}

int main( int argc, char** argv )
{

	ros::init(argc, argv, "stiffness_learning");

	StiffnessLearning node;

  
  while (ros::ok())
  {

    ros::Rate loop_rate(30);

    node.run();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

}