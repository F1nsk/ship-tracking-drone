
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Bool.h> 
#include <sensor_msgs/NavSatFix.h> 








class tracker
{

    public:

    void trackStartCallBck(const std_msgs::Bool::ConstPtr& msg); 
    void darknetCallBck(const darknet_ros_msgs::BoundingBoxes& msg); 
    void gpsCallBck(const sensor_msgs::NavSatFix& msg); 
    







    tracker();

    ~tracker(); 

    private:

    





};