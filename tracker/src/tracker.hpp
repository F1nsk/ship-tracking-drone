
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Bool.h> 
#include <sensor_msgs/NavSatFix.h> 
#include <ros/ros.h>




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

    void trackStartCallBck(const std_msgs::Bool::ConstPtr& msg); 
    void darknetCallBck(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg); 
    void gpsCallBck(const sensor_msgs::NavSatFix::ConstPtr& msg); 
    coordinate calcCenterPoint(coordinate min, coordinate max); 
    coordinate GPStoCartisian(coordinate GPS); 
    void trackingPath(coordinate, int boatLenght); 









    tracker();

    ~tracker(); 

    private:
        // gpsCoordinate    





};