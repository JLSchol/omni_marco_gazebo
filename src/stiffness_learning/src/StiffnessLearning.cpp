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
    // data_matrix_.reserve(1);
    // data_matrix_.[0].reserve(3);

}  


void StiffnessLearning::run()
{
  ROS_INFO_STREAM("shit is aan het runnen");
  
  std::vector<float> error_signal(3);
  getErrorSignal(error_signal); // werkt
  
  populateDataMatrix(error_signal,data_matrix_); // 
  
  Eigen::Matrix3f covariance_matrix;
  getCovarianceMatrix(data_matrix_, covariance_matrix);

//   getEigenValues();

//   getEigenVector();

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

void StiffnessLearning::getTF()
{
    if (TF_listener_.waitForTransform(ee_frame_name_, base_frame_name_, ros::Time(0), ros::Duration(0.25)))
    {
        TF_listener_.lookupTransform(base_frame_name_, ee_frame_name_,ros::Time(0), base_to_ee_);
    }
}

void StiffnessLearning::getCovarianceMatrix(std::vector< std::vector<float> >& data_matrix, Eigen::Matrix3f& covariance_matrix)
{
    
    int height = data_matrix[0].size();
    int length = data_matrix.size();
    // ROS_INFO_STREAM("height: "<< height << " length: "<< length);
    // convert to data_matrix to eigen format
    std::vector<float> test_vector(3);
    test_vector.push_back(1); test_vector.push_back(2); test_vector.push_back(3);

    Eigen::MatrixXf test = Eigen::Map<Eigen::Matrix<float, 3, 1> >(test_vector.data());
    
    ROS_INFO_STREAM("eigen:" << test(0)<< test(1)<< test(2));

    // do cov calculations
    
}


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