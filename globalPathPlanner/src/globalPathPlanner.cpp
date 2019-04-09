
#include "globalPathPlanner.hpp"






globalPathPlanner::globalPathPlanner()
{


};



void globalPathPlanner::getPoint(vector<double> input)
{
    /* Simply takes an input vector of points and check if enought points have en entered
     * and puts them into a coordinate
     */

    float temp;



   if ( input.size() != 12)

       {
       std::cout << "Not correct amount of elements \n" << "only " << input.size() << " entered " << std::endl;
       }


       else

       {
        std::cout << "Number of elements " << input.size() << " entered " << std::endl;



                           pointOne.x = input.at(0);
                           pointOne.y = input.at(1);
                           pointOne.z = input.at(2);
                           pointTwo.x = input.at(3);
                           pointTwo.y = input.at(4);
                           pointTwo.z = input.at(5);
                           pointThree.x = input.at(6);
                           pointThree.y = input.at(7);
                           pointThree.z = input.at(8);
                           pointFour.x = input.at(9);
                           pointFour.y = input.at(10);
                           pointFour.z = input.at(11);



        }



};


int  globalPathPlanner::calcDist(coordinate a, coordinate b)
{

    int dist = sqrt(pow((a.x - b.x),2 ) + pow((a.y - b.y),2)); 

    return dist; 

}


bool globalPathPlanner::isSquare(coordinate a, coordinate b, coordinate c, coordinate d)
{
        /* This function checks if the entered coordinates form a rectangle, this is done
         * by check if the squard distances from the center of mass is equal for all 4 points
         */
        double xCenter;
        double yCenter;
        double dist1, dist2, dist3, dist4;

        xCenter = (a.x + b.x + c.x + d.x)/4;
        yCenter = (a.y + b.y + c.x + d.x)/4;

        dist1 = sqrt(xCenter - a.x)+sqrt(yCenter - a.y);
        dist2 = sqrt(xCenter - b.x)+sqrt(yCenter - b.y);
        dist3 = sqrt(xCenter - c.x)+sqrt(yCenter - c.y);
        dist4 = sqrt(xCenter - d.x)+sqrt(yCenter - d.y);


        return dist1 == dist2 && dist1 == dist3 && dist1 == dist4;


};


void globalPathPlanner::printPath( vector<coordinate> somePath, bool showZ)
{
    if( showZ == true )
    {
        for(int i =0; i<somePath.size() ; i++)
        {
            std::cout << somePath.at(i).x << " "  << somePath.at(i).y << " " << somePath.at(i).z << std::endl;
        }
    }
    else 
    {

    for(int i =0; i<somePath.size() ; i++)
        {
            std::cout << somePath.at(i).x << " "  << somePath.at(i).y << std::endl;
        }
    } 

}

void globalPathPlanner::printCoordinate(coordinate someCoordinate, std::string someString)
{
 
 std::cout << "someString" << someCoordinate.x << someCoordinate.y << someCoordinate.z << std::endl;  
 
}


vector<coordinate> globalPathPlanner::elipsiodPath(coordinate center, int r1, int r2, int numberOfPoints)
{   
            coordinate temp;
             vector<coordinate> path;
         for(double angle=0; angle<=2*pi; angle+=2*pi/numberOfPoints)
            {
                temp.x = center.x + r1 * cos(angle); 
                temp.y = center.y + r2 * sin(angle);
                path.push_back(temp);

            }
    
    return path; 

}



vector<coordinate> globalPathPlanner::flyincirkel(coordinate center, int radius, int numberOfPoints, bool show)
{
    /* This function creates a circular flight path for a plane from center point and a radius
     *
     */

    coordinate temp;
    vector<coordinate> path;
   

    for(double angle=0; angle<=2*pi; angle+=2*pi/numberOfPoints)
    {

       temp.x = center.x+ radius*cos(angle);
       temp.y = center.y + radius*sin(angle);
       temp.z = center.z;
       path.push_back(temp);
       

    }
     if(show == true)
     {
        for(int i =0; i<path.size() ; i++)
        {
            std::cout << path.at(i).x << " "  << path.at(i).y << std::endl;
        }
     }
     return path;

};



vector<coordinate> globalPathPlanner::generateMap(coordinate a, coordinate b, coordinate c, coordinate d, double gridResolution)
{
     coordinate temp; 
     vector<coordinate> coordList;
     vector<coordinate> removeMe; 
     coordinate tempC =  c;
     double gridRes = gridResolution; 

     
     
        // unsigned int distOne = calcDist(a, b); 
        // unsigned int distTwo = calcDist(c, d); 
        // unsigned int distThree = calcDist(c, a); 

        // int numberOfPtsLongSide = distOne/gridRes;  
        // int numberOfPtsShortSide = distThree/gridRes; 
     

        for(double i = 0; i <= b.x; i+= gridRes)
        {
        for(double j= 0; j <= b.y; j+= gridRes)
                {
                temp.x = c.x + i ;
                temp.y = c.y + j;
                coordList.push_back(temp); 
                } 
        }


        // for(int i = gridRes; i=b.x; i+gridRes)
        // {   
        //      temp.x = a.x + gridRes;
        //      temp.y = a.y;
        //      gridCoordinates.push_back(temp);

        // }
        printPath(coordList, true); 

        std::cout <<  " done "  << std::endl; 
         

        return(coordList); 
     

    



};



void globalPathPlanner::msgCallback(const std_msgs::Bool::ConstPtr& msg)
{
   run =  msg->data; 
  
}





void globalPathPlanner::publishPath(vector<coordinate> somePath)
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
    poseStamped.pose.position.z = 100; 
    poseStamped.pose.orientation.z =pi + atan2(somePath.at(interator).y - somePath.at(interator -1).y, somePath.at(interator).x - somePath.at(interator -1).x);

     
    

     poseStampedPub.publish(poseStamped); 

     interator +=1; 
     if (interator >= somePath.size())
     {
         interator = 1; 
     }
        
            
  
   

    




}




globalPathPlanner::~globalPathPlanner()
{

};


























































