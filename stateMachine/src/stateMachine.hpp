#include <math.h>
#include <iostream>
#include <stdio.h>
#include <array>
#include <iomanip>
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include <std_msgs/Bool.h> 
#include <std_msgs/Int8.h>



class stateMachine
{

    public: 
    bool run = false; 
    bool lowLevelDetection = false;  
    bool highLevelDection = false; 

    stateMachine(); 

    void stateChanger();
    void stateSearching(); 
    void stateClassify(); 
    void stateTrack(); 
    void stateTakeOff(); 
    void publish( ros::Publisher somePUB, bool someBoolean); 
    void lowLevelDetectorCallBck(const std_msgs::Bool::ConstPtr& msg); 
    void ClassifierCallBck(const std_msgs::Int8ConstPtr& msg);  




    ros::NodeHandle n; 
    ~stateMachine(); 



    private:  
        bool detector; 
        bool classifer; 
        bool takeOff;  

        ros::Publisher areaSearchPUB = n.advertise<std_msgs::Bool>("/stateMachine/areaSearcher" ,2 , true);
        ros::Publisher classifierPUB = n.advertise<std_msgs::Bool>("/stateMachine/classifier", 2 , true); 
        ros::Publisher lowLevelDetectorPUB  = n.advertise<std_msgs::Bool>("/stateMachine/lowLevelDetector",2 , true); 
        ros::Publisher trackerDetectorPUB  = n.advertise<std_msgs::Bool>("/stateMachine/track", 2, true );
        





};