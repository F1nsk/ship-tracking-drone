#include "globalPathPlanner.hpp"






int main(int argc, char **argv)
{
    
//     std::cout << "init" << std::endl;

//     std::vector<double> input= {1.00, 1.00, 1.00, 2.000, 2.00, 2.000, 3.00, 3.00, 3.00, 4.0, 4.0, 4.00}; // formate is point xyz, followed by the next point

//    // gp.getPoint(input);

     
    coordinate one = {0, 100, 100};
    coordinate two = {100, 100, 0};
    coordinate three = {0, 0, 100};
    coordinate four = {0, 100, 0};
    vector <coordinate> squarePath = {one, two, three, four}; 

//     gp.flyincirkel(center, 100, 5, false);
//     gp.isSquare(one, two, three, four);
    
    
    
    ros::init( argc, argv, "talker");
    coordinate center = {0,0,50};
    globalPathPlanner gp;

    
    int i = 1;  
    
    vector<coordinate>  path = gp.flyincirkel(center, 30, 500, true);
    
    ros::Rate loop_rate(10);
    ros::Subscriber sub = gp.n.subscribe("/stateMachine/areaSearcher", 1, &globalPathPlanner::msgCallback, &gp ); 
    while(ros::ok())
    {
            
            if( gp.run != true)
            {
            std::cout << "waiting" << std::endl; 
            
           
            } 
            else
            {
             std::cout  << " running " << std::endl; 
             gp.publishPath(path); 
            }
            
            






            loop_rate.sleep();     
            
            ros::spinOnce(); 

        

    }
    
    return 0; 
    
    

}

 