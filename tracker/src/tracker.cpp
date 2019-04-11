
#include "tracker.hpp"


tracker::tracker()
{


}




void tracker::darknetCallBck(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    coordinate temp;
    coordinate temp1;     
    temp.x =  msg->bounding_boxes[0].xmin; 
    temp.y = msg->bounding_boxes[0].ymin;
    temp1.x = msg->bounding_boxes[0].xmax;
    temp1.y = msg->bounding_boxes[0].ymax; 

    calcCenterPoint(temp, temp1);

}

coordinate tracker::calcCenterPoint(coordinate min, coordinate max)
{
        coordinate centerPoint; 

        centerPoint.x = (min.x + max.x) / 2; 
        centerPoint.y = (min.y + max.y) /2; 


        return centerPoint;  




}


void  tracker::gpsCallBck(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gpsCoordinate temp; 


    temp.altitude = msg->altitude;
    temp.latitude = msg->latitude;
    temp.longitude = msg ->longitude; 

    


}








tracker::~tracker()
{

}