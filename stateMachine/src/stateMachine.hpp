#include <math.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdio.h>
#include <array>
#include <iomanip>
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include <std_msgs/Bool.h> 
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h> 
#include <sensor_msgs/Image.h> 

using namespace cv;


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
    void takeOffCMDCallBck(const std_msgs::Bool::ConstPtr& msg); 
    void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg);
    void imagPublisher(cv::Mat img);  






    ros::NodeHandle nH;

    // image_transport::ImageTransport it; 

    
    ~stateMachine(); 



    private:  
        bool detector = false; 
        bool classifer = false; 
        bool takeOffCMD = false;  
        int StateNumber = 0; 

        ros::Publisher takeOffPUB =  nH.advertise<std_msgs::Bool>("/stateMachine/takeOff/trigger", 2, true); 
        ros::Publisher areaSearchPUB = nH.advertise<std_msgs::Bool>("/stateMachine/areaSearcher/trigger" ,2 , true);
        ros::Publisher classifierPUB = nH.advertise<std_msgs::Bool>("/stateMachine/classifier/trigger", 2 , true); 
        ros::Publisher lowLevelDetectorPUB  = nH.advertise<std_msgs::Bool>("/stateMachine/lowLevelDetector/trigger",2 , true); 
        ros::Publisher trackerDetectorPUB  = nH.advertise<std_msgs::Bool>("/stateMachine/track/trigger", 2, true );
        // image_transport::Publisher imagePub = it.advertise("/yoloDetectFeed/image_raw", 1);


        





};