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



class globalPathPlanner
{

public:

    bool run;
    bool takeOffCMD; 
    globalPathPlanner();
    void getPoint(vector<double> input);
    bool isSquare(coordinate a, coordinate b, coordinate c, coordinate d);
    vector<coordinate> flyincirkel(coordinate center, int radius, int numberOfPoints, bool show);
    vector<coordinate> generateMap(coordinate a, coordinate b, coordinate c, coordinate d, double gridResolution, double colDist);
    vector<coordinate> elipsiodPath(coordinate center, int r1, int r2,  int numberOfPoints); 
    vector<coordinate> randomwElip(coordinate center); 

    int calcDist(coordinate a, coordinate b); 
    void printPath(vector<coordinate> somePath, bool showZ);
    void printCoordinate(coordinate someCoordinate, std::string someString); 
    
    void msgCallback(const std_msgs::Bool::ConstPtr& msg); 
    void takeOffCMDCallBck(const std_msgs::Bool::ConstPtr& msg); 
    void publishPath(vector<coordinate> somePath);  
    void takeOff();

    void subscriber(); 
    ros::NodeHandle n;
   
    ~globalPathPlanner();



private:
 
   
    coordinate pointOne;
    coordinate pointTwo;
    coordinate pointThree;
    coordinate pointFour;
    vector<coordinate> gridCoordinates; 
    ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("/position_controller/drone",50 , true);
    int interator = 1; 
    int takeOffiterator = 1; 







};
