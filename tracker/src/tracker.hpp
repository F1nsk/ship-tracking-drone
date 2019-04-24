
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Bool.h> 
#include <sensor_msgs/NavSatFix.h> 
#include <ros/ros.h>
#include <vector> 
#include <eigen3/Eigen/Dense> 

const double pi = 3.14159;


struct coordinate
{
    double x;
    double y;
    double z;
    double yaw; 
};


struct gpsCoordinate
{
    double latitude;
    double longitude;
    double altitude;

     
};



class tracker
{

    public:
    Eigen::MatrixXd poseMatrix; 
    tracker();
    void trackStartCallBck(const std_msgs::Bool::ConstPtr& msg); 
    void darknetCallBck(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg); 
    void gpsCallBck(const sensor_msgs::NavSatFix::ConstPtr& msg); 
    coordinate calcCenterPoint(coordinate min, coordinate max); 
    coordinate GPStoCartisian(coordinate GPS); 
    Eigen::MatrixXd getYawMatrix( double yawAngle_); 
    Eigen::MatrixXd getPitchMatrix(double pitchAngle_); 
    Eigen::MatrixXd getRollMatrix(double rollAngle_);
    Eigen::MatrixXd getAMatrix(coordinate pixel_);
    void getposeMatrix(double yawAngle_, double pitchAngle_, double rollAngle_ ); 
    Eigen::MatrixXd getLMatrix(coordinate carmeraInWorld_, coordinate pixel_);
    coordinate doMinSquare( Eigen::MatrixXd matrixA_ , Eigen::MatrixXd matrixL_   );   
    // std::vector < std::vector < double >> multiplyMatrix(std::vector<std::vector< double>> matrixA_, std::vector<std::vector< double>> matrixB_);
    // std::vector < std::vector < double >> transpose(std::vector< std::vector< double>> matrix_); 
    // double getDet(std::vector < std::vector < double>> matrix_); 
    // std::vector< std::vector < double >> getInverse( std::vector < std::vector < double >> matrix_  );  
    // void printMatrix(std::vector<std::vector<double>> matrix_);
    void trackingPath(coordinate center, int radius); 
    ros::NodeHandle n;
    void publistPath(std::vector<coordinate> somePath); 

       














    ~tracker(); 

    private:

        double f = 4651.16; //forcal lenght?  
        double sensorW = 5456/2; // sensor width in pixels
        double sensorH = 3064/2; // sensor height in pixels
        
        ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("/position_controller/drone",50 , true);
        double altitude =100; 

        
        // gpsCoordinate    





};