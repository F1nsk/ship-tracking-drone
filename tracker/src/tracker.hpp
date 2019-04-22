
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Bool.h> 
#include <sensor_msgs/NavSatFix.h> 
#include <ros/ros.h>
#include <vector> 

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
    tracker();
    void trackStartCallBck(const std_msgs::Bool::ConstPtr& msg); 
    void darknetCallBck(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg); 
    void gpsCallBck(const sensor_msgs::NavSatFix::ConstPtr& msg); 
    coordinate calcCenterPoint(coordinate min, coordinate max); 
    coordinate GPStoCartisian(coordinate GPS); 
    std::vector< std::vector<double> > getYawMatrix( int yawAngle_); 
    std::vector <std::vector <double>> getPitchMatrix(int pitchAngle_); 
    std::vector <std::vector <double>> getRollMatrix(int rollAngle_);
    std::vector < std::vector < double>> getA( std::vector <std::vector <double>> rL, coordinate point1, coordinate point2, int focalLenght);
    std::vector < std::vector < double>> getL( std::vector <std::vector <double>> rL, coordinate point1, coordinate point2, int focalLenght);
    std::vector < std::vector < double >> multiplyMatrix(std::vector<std::vector< double>> matrixA_, std::vector<std::vector< double>> matrixB_);
    std::vector < std::vector < double >> transpose(std::vector< std::vector< double>> matrix_); 
    void printMatrix(std::vector<std::vector<double>> matrix_);
    void trackingPath(coordinate center, int radius); 
    ros::NodeHandle n;
    void publistPath(std::vector<coordinate> somePath); 

      












    ~tracker(); 

    private:

        double f = 4651.16; //forcal lenght?  
        double sensorW = 5456/2; // sensor width in pixels
        double sensorH = 3064/2; // sensor height in pixels
        ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("/position_controller/drone",50 , true);


        
        // gpsCoordinate    





};