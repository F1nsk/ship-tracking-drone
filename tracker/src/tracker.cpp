
#include "tracker.hpp"


tracker::tracker()
{


}




void tracker::darknetCallBck(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    coordinate temp;
    coordinate temp1;     
    temp.x =  msg->bounding_boxes[0].xmin; 
    temp.y = msg->bounding_boxes[0].ymin;
    temp1.x = msg->bounding_boxes[0].xmax;
    temp1.y = msg->bounding_boxes[0].ymax; 

    calcCenterPoint(temp, temp1);

}

coordinate tracker::calcCenterPoint(coordinate min, coordinate max)
{
        coordinate centerPoint; 

        centerPoint.x = (min.x + max.x) / 2; 
        centerPoint.y = (min.y + max.y) /2; 


        return centerPoint;  




}


void  tracker::gpsCallBck(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gpsCoordinate temp; 


    temp.altitude = msg->altitude;
    temp.latitude = msg->latitude;
    temp.longitude = msg ->longitude; 

    


}




Eigen::MatrixXd tracker::getYawMatrix(double yawAngle_)
{
    double yawAngle = yawAngle_;


    Eigen::MatrixXd yawMatrix(3,3);
    yawMatrix << cos(yawAngle), sin(yawAngle), 0, - sin(yawAngle), cos(yawAngle), 0, 0, 0,1;


    return yawMatrix; 
}


Eigen::MatrixXd tracker::getPitchMatrix(double pitchAngle_)
{

    double pitchAngle = pitchAngle_; 

    Eigen::MatrixXd pitchMatrix(3,3); 
    pitchMatrix << 1, 0, 0, 0, cos(pitchAngle), -sin(pitchAngle), 0, sin(pitchAngle), cos(pitchAngle);

    return pitchMatrix;             
}



Eigen::MatrixXd tracker::getRollMatrix(double rollAngle_ )
{

    double rollAngle = rollAngle_ ; 

  Eigen::MatrixXd rollMatrix(3,3);
  rollMatrix << cos(rollAngle), 0, sin(rollAngle), 0,1 ,0 ,-sin(rollAngle), 0, cos(rollAngle);            

  return rollMatrix;  
}




void tracker::getposeMatrix(double yawAngle_, double pitchAngle_, double rollAngle_)
{
    double yawAngle = yawAngle_; 
    double pitchAngle = pitchAngle_; 
    double rollAngel = rollAngle_; 

    Eigen::MatrixXd tempPoseMatrix; 
    Eigen::MatrixXd yawMatrix = getYawMatrix(yawAngle); 
    Eigen::MatrixXd pitchMatix = getPitchMatrix(pitchAngle); 
    Eigen::MatrixXd rollMatrix = getRollMatrix(rollAngel); 


    tempPoseMatrix = yawMatrix * (pitchMatix * rollMatrix); 

    poseMatrix = tempPoseMatrix; 









}




void tracker::trackingPath(coordinate center, int radius)
{
    /* This function creates a circular flight path for a plane from center point and a radius
     *
     */

    coordinate temp;
    std::vector<coordinate> path;
    int numberOfPoints = 300; 
   

    for(double angle=0; angle<=2*pi; angle+=2*pi/numberOfPoints)
    {

       temp.x = center.x+ radius*cos(angle);
       temp.y = center.y + radius*sin(angle);
       temp.z = center.z;
       path.push_back(temp);
       

    }

    publistPath(path); 
    

};

Eigen::MatrixXd tracker::getAMatrix(coordinate pixel_)
{

    coordinate pixel = pixel_;  
    double el11 = (pixel.x * poseMatrix.coeff(1,3) + f * poseMatrix.coeff(1,1));
    double el12 = (pixel.x * poseMatrix.coeff(2,3) + f * poseMatrix.coeff(2,1)); 
    double el21 = (pixel.y * poseMatrix.coeff(1,2) + f * poseMatrix.coeff(1,2));
    double el22 = (pixel.y * poseMatrix.coeff(2,3) + f * poseMatrix.coeff(2,2)); 
    Eigen::MatrixXd AMatrix(2,2); 
    AMatrix << el11 , el12  , el21 , el22; 

    return AMatrix;                                                      




}


Eigen::MatrixXd tracker::getLMatrix(coordinate cameraInWorld_ , coordinate pixel_ )
{

    double xW = cameraInWorld_.x;
    double yW = cameraInWorld_.y;
    double zW = cameraInWorld_.z; 
    double xP = pixel_.x; 
    double yP = pixel_.y; 
    double z = altitude; 


    Eigen::MatrixXd LMatrix(2,1);
    LMatrix <<   xW * (f*poseMatrix.coeff(1,1)+ xP * poseMatrix.coeff(1,3))  + yW * (f * poseMatrix.coeff(2,1) + xP * poseMatrix.coeff(2,3)) + zW * (f * poseMatrix.coeff(3,1) + xW * poseMatrix.coeff(3,3) ) - z * (f * poseMatrix.coeff(3,1) + xW * poseMatrix.coeff(3,3))
    ,xW * (f*poseMatrix.coeff(1,2)+ yP * poseMatrix.coeff(1,3)) + yW * (f * poseMatrix.coeff(2,2) + yP * poseMatrix.coeff(2,3)) + zW * (f * poseMatrix.coeff(3,2) + yW * poseMatrix.coeff(3,3)) - z * (f * poseMatrix.coeff(3,2) + yW * poseMatrix.coeff(3,3));

    return LMatrix;  

  
}

coordinate tracker::doMinSquare(Eigen::MatrixXd matrixA_ , Eigen::MatrixXd matrixL_ )
{
    Eigen::MatrixXd matrixA = matrixA_;
    Eigen::MatrixXd matrixL = matrixL_; 
    Eigen::MatrixXd result(2,1); 
    coordinate location; 

    result = matrixA.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(matrixL); //least squares solving 

    location.x = result.coeff(1,1);
    location.y = result.coeff(2,1); 
    
    std::cout << "result is \n"  << result << std::endl;   
    return location; 





}

void tracker::publistPath(std::vector<coordinate> somePath)
{
    ros::Rate my_loop_rate(10);
             
    std::cout  << " running " << std::endl; 
    int interator; 
    geometry_msgs::PoseStamped poseStamped; 
    poseStamped.header.frame_id="droneWayPoint"; 
    poseStamped.header.stamp= ros::Time::now(); 
   
    //std::cout << "path size" << somePath.size() << std::endl;
    //printPath(somePath, true);
    
    // std::cout <<" debug here" << std::endl; 
    poseStamped.pose.position.x = somePath.at(interator).x; 
    poseStamped.pose.position.y = somePath.at(interator).y; 
    poseStamped.pose.position.z = somePath.at(interator).z; 
    poseStamped.pose.orientation.z =pi + atan2(somePath.at(interator).y - somePath.at(interator -1).y, somePath.at(interator).x - somePath.at(interator -1).x);

     
    

     poseStampedPub.publish(poseStamped); 

     interator +=1; 
     if (interator >= somePath.size())
     {
         interator = 1; 
     }
        
   


}


tracker::~tracker()
{

}



// std::vector<std::vector<double> >tracker::transpose(std::vector<std::vector<double>> matrix_)
// {   
    
//     std::vector<std::vector< double >> matrix = matrix_; 
//     int rows = matrix.size(); 
//     int cols  = matrix[0].size(); 
//     std::vector<std::vector<double>> tranposeMatrix(rows, std::vector<double> (cols));

//     for (int i = 0; i < cols; i ++)
//         {
//             for(int j = 0; j  < rows; j++)
//                 {
//                     tranposeMatrix[i][j] = matrix[j][i]; 
//                 }
//         } 
    
    
//     return tranposeMatrix;  




// }

// std::vector<std::vector < double>> tracker::multiplyMatrix(std::vector<std::vector<double>> matrixA_, std::vector<std::vector<double>> matrixb_ )
// {   

//     std::vector<std::vector<double>> matrixA = matrixA_; 
//     std::vector<std::vector<double>> matrixB = matrixb_;
//     int rows =  matrixA.size();
//     int cols = matrixA[0].size();
//     std::vector<std::vector<double>> multiMatrix(rows, std::vector<double> (cols));




//     if( matrixA[0].size() != matrixB.size())
//         {
//             std::cout << " matrix dimensions does not match \n" <<  std::endl; 
     
//         }
//     else
//         {
//             for(int i =0;  i < matrixA.size(); i++)
//                 {
//                     for(int j =0; j < matrixB[0].size(); j++)
//                         {
//                             multiMatrix[i][j] = 0;
//                             for(int k =0; k < matrixB.size(); k++)
//                                 {

//                                     multiMatrix[i][j] += matrixA[i][k] * matrixB[k][j]; 
//                                 }
//                         }
//                 }
//         }
//     return multiMatrix; 

// }



// double getDet(std::vector < std::vector < double>> matrix_ )
// {

//     double det = 0; 
//     std::vector < std::vector < double >> matrix = matrix_; 

//     for( int i =0; i < matrix[0].size() ; i++ )
//         {
//             for( int j = 0; j < matrix.size() ; j ++)
//                 {
//                     det =  det + (matrix[j][i])
//                 }
//         }
                          



// }

