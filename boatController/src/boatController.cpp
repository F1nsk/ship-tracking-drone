
#include "boatController.hpp"






boatcontroller::boatcontroller()
{


};



vector<coordinate> boatcontroller::elipsiodPath(coordinate center)
{          
    int r1 = std::rand() % 50 + 450; 
    int r2 = std::rand() % 50 +  450;  


    coordinate temp;
    vector<coordinate> path;
         for(double angle=0; angle<=2*pi; angle+=2*pi/500)
            {
                temp.x = center.x + r1 * cos(angle); 
                temp.y = center.y + r2 * sin(angle);
                path.push_back(temp);

            }
    
    return path; 

}




void boatcontroller::publishPath(vector<coordinate> somePath)
{
    ros::Rate my_loop_rate(10);
             
    std::cout  << " running " << std::endl; 

    geometry_msgs::PoseStamped poseStamped; 
    poseStamped.header.frame_id="droneWayPoint"; 
    poseStamped.header.stamp= ros::Time::now(); 
   
    //std::cout << "path size" << somePath.size() << std::endl;
    //printPath(somePath, true);
    
    // std::cout <<" debug here" << std::endl; 
    poseStamped.pose.position.x = somePath.at(interator).x; 
    poseStamped.pose.position.y = somePath.at(interator).y; 
    poseStamped.pose.position.z = -3; 
    poseStamped.pose.orientation.z =pi + atan2(somePath.at(interator).y - somePath.at(interator -1).y, somePath.at(interator).x - somePath.at(interator -1).x);

     
    
     poseStampedPub.publish(poseStamped); 

     interator +=1; 
     if (interator >= somePath.size())
     {
         interator = 1; 
     }
        
            
  
   

    




}




boatcontroller::~boatcontroller()
{

};


























































