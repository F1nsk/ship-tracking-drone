#include <ros/ros.h>
#include "cvFunctions.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h> 
#include <sensor_msgs/Image.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h> 
#include <opencv2/opencv.hpp>
#include <sstream> 
using namespace cv; 
cvFunctions func; 

std_msgs::String test; 
cv::Mat imgRGB; 


	



void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{

   cv_bridge::CvImagePtr img_ptr;  
 
  try
  {
     
   

    img_ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8); 
    imgRGB = img_ptr->image;
    func.notWater(imgRGB);	

    
    //cv::imshow("view", imgRGB);
    //cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgMsg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{

 	
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  ros::spinOnce();
  ///cv::destroyWindow("view");

   
   std::cout << "here" << std::endl;  
  ros::Publisher pub =nh.advertise<std_msgs::Bool>("low_lvl_detector", 500); 
  std::cout << "here2" << std::endl;  
 
  ros::Rate loop_rate(1); 
  ros::spinOnce();  

  
  int count = 0;
  
  while(nh.ok())
	{

	

	 pub.publish(func.boolMsg);  
         loop_rate.sleep(); 
	 ros::spinOnce();
         loop_rate.sleep(); 
         ++count; 
	  
	}
  	
 
 return 0; 
	
}





















































