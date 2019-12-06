#include "stiffness_learning/StiffnessLearning.h"

// issues:
//  When data_matrix has not yet reaches the size of the window_length
// The eigenvalues and vectors of the covariance matrix seems wrong?


StiffnessLearning::StiffnessLearning():
nh_("~")
{
    getParameters();
    initializeSubscribers();
    initializePublishers();
    initializeStiffnessMsg();
    initializeCovarianceMsg();
    // clear
}  


void StiffnessLearning::run()
{
//   ROS_INFO_STREAM("shit is aan het runnen");
  
  std::vector<float> error_signal(3);
  getErrorSignal(error_signal); // new signal every loop
  ROS_INFO_STREAM("error_signal: \n"<< error_signal[0]
  << "\n "<< error_signal[1]<< "\n "<< error_signal[2]);

//   data_matrix_ grows by adding the errorsignal until window length
  populateDataMatrix(error_signal,data_matrix_); 

  // covariance matrix is found from data matrix
  Eigen::Matrix3f covariance_matrix;
  getCovarianceMatrix(data_matrix_, covariance_matrix); 
  ROS_INFO_STREAM("covariance_matrix: \n"<< covariance_matrix);

  // get eigenvalues and eigenvectors
  Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix,true); // finds upon initialization
  ROS_INFO_STREAM("EIGENVALUES:"<< eigen_solver.eigenvalues());
  ROS_INFO_STREAM("eigenvectors:"<< eigen_solver.eigenvectors());
  

  Eigen::Vector3f stiffness_diagonal;
  getStiffnessEig(eigen_solver,stiffness_diagonal);
//   ROS_INFO_STREAM("stiffness_diagonal: "<< stiffness_diagonal[0]
//                                         << stiffness_diagonal[1]<< stiffness_diagonal[2]);
  
  Eigen::Matrix3f K_matrix;
  setStiffnessMatrix(eigen_solver,stiffness_diagonal,K_matrix);
  ROS_INFO_STREAM("K_matrix: "<< "\n" << K_matrix);
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(K_matrix);
    // Eigen::Vector3f eigen_values  = eigensolver.eigenvalues();
    // Eigen::Matrix3f eigen_vectors = eigensolver.eigenvectors();
    // ROS_INFO_STREAM("K_matrix: "<< "\n" << K_matrix);
    // ROS_INFO_STREAM("vector: \n" << eigen_vectors);
    // ROS_INFO_STREAM("values: \n" << eigen_values);

  fillStiffnessMsg(K_matrix);
  
  fillCovarianceMsg(covariance_matrix);
  

  stiffness_pub_.publish(stiffness_matrix_);
  covariance_pub_.publish(covariance_matrix_);
  ROS_INFO_STREAM("---------------------------------------------------");
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

void StiffnessLearning::getTF()
{
    if (TF_listener_.waitForTransform(ee_frame_name_, base_frame_name_, ros::Time(0), ros::Duration(0.25)))
    {
        TF_listener_.lookupTransform(base_frame_name_, ee_frame_name_,ros::Time(0), base_to_ee_);
    }
}

// Test this function for observations > window length
void StiffnessLearning::populateDataMatrix(std::vector<float>& error_signal, 
                                            std::vector< std::vector<float> >& data_matrix)
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
        // ROS_INFO_STREAM("IF<<<");
    }
    else if(observations == window_length_){
        data_matrix.erase(data_matrix.begin()); //-1
        data_matrix.push_back(error_signal);   //+1
        // ROS_INFO_STREAM("IF======");
    }
    else if(observations > window_length_){
        int to_remove = (observations-window_length_+1); // remove 1 extra (-1) for pushback later 
        data_matrix.erase( data_matrix.begin(), data_matrix.begin()+to_remove); // remove first part of vector
        data_matrix.push_back(error_signal); // +1
        // ROS_INFO_STREAM("IF>>>>>>>>>");
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
    // ROS_INFO_STREAM("DATAMATRIXEIGEN: "<< data_mat_eigen);  
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
        if( E.real() > lower && E.real() < upper ){ // && (E.real() > std::pow(-10,-6)) << waarom werkt dit niet?
            E.real() = 0;
            // ROS_INFO_STREAM("IN Ereal IF ROUNDOF to 0 "<< E.real());
        }

        // Check wheter or not I need to take square root? in klas kronander(2012/2014) 
        float lambda = sqrt(E.real());

        // if(lambda <= std::pow(10,-4) && lambda>= std::pow(-10,-4)){
        //     lambda = 0;
        //     ROS_INFO_STREAM("IN Lambda IF ROUNDOF=0 "<< lambda);
        // }

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
        // ROS_INFO_STREAM("Ereal"<< E.real());
        // ROS_INFO_STREAM("Lambda "<< lambda);
        // ROS_INFO_STREAM("--------------------------------------");
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

void StiffnessLearning::fillStiffnessMsg(Eigen::Matrix3f k_matrix)
{
    int height = stiffness_matrix_.layout.dim[0].size;//no matiching functon
    int width = stiffness_matrix_.layout.dim[1].size;
    
    std::vector<float> vec(width*height, 0);
    for (int i=0; i<height; i++){
        for (int j=0; j<width; j++){
            vec[i*width + j] = k_matrix(i,j);
        }
    }
    stiffness_matrix_.data = vec;
}

void StiffnessLearning::fillCovarianceMsg(Eigen::Matrix3f covariance)
{
    int height = covariance_matrix_.layout.dim[0].size;//no matiching functon
    int width = covariance_matrix_.layout.dim[1].size;
    
    std::vector<float> vec(width*height, 0);
    for (int i=0; i<height; i++){
        for (int j=0; j<width; j++){
            vec[i*width + j] = covariance(i,j);
        }
    }
    covariance_matrix_.data = vec;
}

void StiffnessLearning::getParameters()
{
    // input output topic names
    nh_.param<std::string>("marker_topic_name", marker_trans_topic_name_, "/marker_transform");
    nh_.param<std::string>("stiffness_topic_name", stiffness_command_topic_name_, "/stiffness_command");  
    nh_.param<std::string>("covariance_topic_name", covariance_command_topic_name_, "/covariance_matrix");  
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
    covariance_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(covariance_command_topic_name_,1);
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

void StiffnessLearning::initializeCovarianceMsg()
{
    int height = 3;
    int width = 3;
    covariance_matrix_.layout.dim.push_back(std_msgs::MultiArrayDimension()); // height
    covariance_matrix_.layout.dim.push_back(std_msgs::MultiArrayDimension()); // width
    covariance_matrix_.layout.dim[0].label = "height";
    covariance_matrix_.layout.dim[1].label = "width";
    covariance_matrix_.layout.dim[0].size = height; 
    covariance_matrix_.layout.dim[1].size = width;
    covariance_matrix_.layout.dim[0].stride = height*width;
    covariance_matrix_.layout.dim[1].stride = width;
    covariance_matrix_.layout.data_offset = 0;
}

void StiffnessLearning::CB_getMarkerTransform(const geometry_msgs::TransformStamped& 
                                                        marker_transform_message)
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