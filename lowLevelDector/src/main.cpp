#include <ros/ros.h>
#include "cvFunctions.h"
 



int main(int argc, char **argv)
{

 	

  ros::init(argc, argv, "lowleveldetector");
   cvFunctions cvfun; 

   image_transport::ImageTransport it(cvfun.nH);

  image_transport::Subscriber subImg = it.subscribe("/drone/camera/image_raw", 1, &cvFunctions:: imageCallback, &cvfun);
  ros::Subscriber subTrigger =cvfun.nH.subscribe("/stateMachine/areaSearcher/trigger", 1, &cvFunctions::msgCallback, &cvfun ); 

 
  ros::Rate loop_rate(10); 

  
	ros::spin();   
	  	
 
 return 0; 
	
}





















































