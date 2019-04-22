#include <math.h>
#include <iostream>
#include <stdio.h>
#include <array>
#include <iomanip>
#include <vector>
#include <cmath>
#include <string> 
#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Bool.h> 
#include <sstream> 

const double pi = 3.14159;


using namespace std;


struct coordinate
{
    double x;
    double y;
    double z;
    double yaw; 
};



class boatcontroller
{

public:

    
    boatcontroller();
  
    vector<coordinate> elipsiodPath(coordinate center); 
    
    
    void publishPath(vector<coordinate> somePath);  

    ros::NodeHandle n;
   
    ~boatcontroller();



private:
 
  
    ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("/position_controller/boat",50 , true);
    int interator = 1; 
    int takeOffiterator = 1; 







};
