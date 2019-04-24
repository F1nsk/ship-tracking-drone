#include "boatController.hpp"





int main(int argc, char **argv)
{


    

    
    
    int r1 = std::rand() % 50 + 450; 
    int r2 = std::rand() % 50 +  450;  
    ros::init( argc, argv, "boatController");
    
    coordinate center = {500, 500};
    boatcontroller boatC;

    vector<coordinate> path = boatC.elipsiodPath(center, 100, 500); 
    ros::Rate loop_rate(10);

    

    while(ros::ok())
    {
        
            
                    

            boatC.publishPathBoat(path);




            loop_rate.sleep();
            
            ros::spinOnce(); 

        

    }
    
    return 0; 
    
    

}

 