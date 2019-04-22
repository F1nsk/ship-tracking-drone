#include "boatController.hpp"





int main(int argc, char **argv)
{


    

    
    
    
    ros::init( argc, argv, "pathplanner");
    
    coordinate center = {500, 500};
    boatcontroller boatC;

    vector<coordinate> path = boatC.elipsiodPath(center); 

    ros::Rate loop_rate(10);

    int i = 1;  
    

    while(ros::ok())
    {
     
            
            

            boatC.publishPath(path);




            loop_rate.sleep();
            
            ros::spinOnce(); 

        

    }
    
    return 0; 
    
    

}

 