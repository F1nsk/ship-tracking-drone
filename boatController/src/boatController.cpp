
#include "boatController.hpp"






boatcontroller::boatcontroller()
{


};



vector<coordinate> boatcontroller::elipsiodPath(coordinate center, int r1 , int r2)
{          
    


    coordinate temp;
    vector<coordinate> path;
         for(double angle=0; angle<=2*pi; angle+=2*pi/50)
            {
                temp.x = static_cast<int>( center.x + r1 * cos(angle)); 
                temp.y = static_cast<int> (center.y + r2 * sin(angle));
                path.push_back(temp);

            }
    
    return path; 

}




void boatcontroller::publishPathBoat(vector<coordinate> somePath)
{
    ros::Rate my_loop_rate(6);
             
    std::cout  << " running " << std::endl; 

    geometry_msgs::PoseStamped poseStamped; 
    poseStamped.header.frame_id="BoatwayPoint"; 
    poseStamped.header.stamp= ros::Time::now(); 
   
    //std::cout << "path size" << somePath.size() << std::endl;
    //printPath(somePath, true);
    
    // std::cout <<" debug here" << std::endl; 
    poseStamped.pose.position.x = somePath.at(interator).x; 
    poseStamped.pose.position.y = somePath.at(interator).y; 
    poseStamped.pose.position.z = -1; 
    poseStamped.pose.orientation.z =atan2(somePath.at(interator).y - somePath.at(interator -1).y, somePath.at(interator).x - somePath.at(interator -1).x);

     
    
     poseStampedPubBoat.publish(poseStamped); 

     interator +=1; 
     if (interator >= somePath.size())
     {
         interator = 1; 
     }
        
  
   

    




}




boatcontroller::~boatcontroller()
{

};


























































