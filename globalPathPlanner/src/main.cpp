#include "globalPathPlanner.hpp"





int main(int argc, char **argv)
{
    
//     std::cout << "init" << std::endl;

//     std::vector<double> input= {1.00, 1.00, 1.00, 2.000, 2.00, 2.000, 3.00, 3.00, 3.00, 4.0, 4.0, 4.00}; // formate is point xyz, followed by the next point

//    // gp.getPoint(input);

     
    coordinate one = {1000, 0, 100};
    coordinate two = {1000, 1000, 0};
    coordinate three = {0, 0, 1000};
    coordinate four = {0, 1000, 0};
    vector <coordinate> squarePath = {one, two, three, four}; 

//     gp.flyincirkel(center, 100, 5, false);
//     gp.isSquare(one, two, three, four);
    
    
    
    ros::init( argc, argv, "pathplanner");
    coordinate center = {120, -50,50};
    globalPathPlanner gp;

    
    int i = 1;  
    
    vector<coordinate>  path = gp.flyincirkel(center, 30, 500, false);
    vector<coordinate> pathElip = gp.elipsiodPath(center, 50, 100, 500 );
    vector<coordinate>  pathGrid = gp.generateMap(one, two, three, four, 2); 
    ros::Subscriber sub = gp.n.subscribe("/stateMachine/areaSearcher/trigger", 1, &globalPathPlanner::msgCallback, &gp ); 
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
            
            if( gp.run == false)
            {
           
            std::cout << "waiting" << std::endl; 
            } 
            else if (gp.run == true)
            {
             gp.takeOff(); 
             gp.publishPath(pathGrid);
             
            }
            else 
            {

                std::cout  << " still waiting" << std::endl; 
            }
            
            






            loop_rate.sleep();
            
            ros::spinOnce(); 

        

    }
    
    return 0; 
    
    

}

 