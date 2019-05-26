
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


void tracker::testPoseMatrix(Eigen::MatrixXd TESTposeMatrix)
{
    poseMatrix = TESTposeMatrix; 


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




Eigen::MatrixXd tracker::getYawMatrix(double yawAngle_) // computes the yaw ration matrix based on the inputted angle
{
    double yawAngle = yawAngle_;


    Eigen::MatrixXd yawMatrix(3,3);
    yawMatrix << cos(yawAngle), sin(yawAngle), 0, - sin(yawAngle), cos(yawAngle), 0, 0, 0,1;


    return yawMatrix; 
}


Eigen::MatrixXd tracker::getPitchMatrix(double pitchAngle_)  // computes the pitch ration matrix based on the inputted angle
{

    double pitchAngle = pitchAngle_; 

    Eigen::MatrixXd pitchMatrix(3,3); 
    pitchMatrix << 1, 0, 0, 0, cos(pitchAngle), -sin(pitchAngle), 0, sin(pitchAngle), cos(pitchAngle);

    return pitchMatrix;             
}



Eigen::MatrixXd tracker::getRollMatrix(double rollAngle_ )  // computes the roll ration matrix based on the inputted angle
{

    double rollAngle = rollAngle_ ; 

  Eigen::MatrixXd rollMatrix(3,3);
  rollMatrix << cos(rollAngle), 0, sin(rollAngle), 0,1 ,0 ,-sin(rollAngle), 0, cos(rollAngle);            

  return rollMatrix;  
}




void tracker::getposeMatrix(double yawAngle_, double pitchAngle_, double rollAngle_)  // computes the complete ration matrix based on the inputted angle 
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

Eigen::MatrixXd tracker::getAMatrix(coordinate pixel_, bool debug_) 
{
    bool debug = debug_;
    coordinate pixel = pixel_;  
    long double el11 = (pixel.x * poseMatrix.coeff(0,2) + f * poseMatrix.coeff(0,0));
    long double el12 = (pixel.x * poseMatrix.coeff(1,2) + f * poseMatrix.coeff(1,0)); 
    long double el21 = (pixel.y * poseMatrix.coeff(0,2) + f * poseMatrix.coeff(0,1));
    long double el22 = (pixel.y * poseMatrix.coeff(1,2) + f * poseMatrix.coeff(1,1)); 
    Eigen::MatrixXd AMatrix(2,2); 
    AMatrix << el11 , el12  , el21 , el22; 

    if(debug == true)
    {
        std::cout << "A" << std::endl; 
        std::cout << AMatrix << std::endl;  
    }


    return AMatrix;                                                      




}


Eigen::MatrixXd tracker::getLMatrix(coordinate cameraInWorld_ , coordinate pixel_ , bool bebug_) // computes the the L matrix needed for min square decompostion 
{
    bool debug = bebug_; 
    long double x0 = cameraInWorld_.x;
    long double y0 = cameraInWorld_.y;
    long double z0 = altitude; 
    long double x = pixel_.x; 
    long double y = pixel_.y; 
    long double focal = f;
    long double Z = planeheight; //point height, ref height of the plane 
    long double r11 = poseMatrix.coeff(0,0);
    long double r12 = poseMatrix.coeff(0,1);
    long double r13 = poseMatrix.coeff(0,2);
    long double r21 = poseMatrix.coeff(1,0);
    long double r22 = poseMatrix.coeff(1,1);
    long double r23 = poseMatrix.coeff(1,2);
    long double r31 = poseMatrix.coeff(2,0);
    long double r32 = poseMatrix.coeff(2,1);
    long double r33 = poseMatrix.coeff(2,2);
    long double el11 = (focal * r11 * x0 + focal * r21 * y0 - focal * r31 * Z + focal * r31 * z0 + x * r13 * x0 + x * r23 * y0 - x * r33 * Z + x * r33 * z0);
    long double el21 = (focal * r12 * x0 + focal * r22 * y0 - focal * r32 * Z + focal * r32 * z0 + y * r13 * x0 + y * r23 * y0 - y * r33 * Z + y * r33 * z0);


    Eigen::MatrixXd LMatrix(2,1);
    LMatrix <<  el11 , el21; 
    if(debug == true)
    {
    std::cout << "debug"<< std::endl;

    std::cout << r11 << std::endl; 
    std::cout << r12 << std::endl; 
    std::cout << r13 << std::endl; 
    std::cout << r21 << std::endl; 
    std::cout << r22 << std::endl; 
    std::cout << r23 << std::endl; 
    std::cout << r31 << std::endl; 
    std::cout << r32 << std::endl; 
    std::cout << r33 << std::endl; 
  

    std::cout << "L" << std::endl;
    std::cout << LMatrix << std::endl; 
    }
    return LMatrix;  

  
}
void tracker::findPointPosImage(long double focalLenght_,  std::vector<long double> cameraWorldPostion_, std::vector<long double> pointPositionInWorld_)
{
        long double focalLenght = focalLenght_; 
        long double camWorldX = cameraWorldPostion_[0];
        long double camWorldY = cameraWorldPostion_[1];
        long double camWorldZ = cameraWorldPostion_[2]; 
        long double pointWorldX = pointPositionInWorld_[0]; 
        long double pointWorldY = pointPositionInWorld_[1];
        long double pointWorldZ = pointPositionInWorld_[2]; 

        long double yP = 0;
        long double xP = 0; 

        xP = -focalLenght * ( ( (poseMatrix.coeff(0,0)*(pointWorldX - camWorldX)) + (poseMatrix.coeff(1,0) * (pointWorldY - camWorldY))  + (poseMatrix.coeff(2,0) * (pointWorldZ - camWorldZ)) )/
                              ( (poseMatrix.coeff(0,2)*(pointWorldX - camWorldX)) + (poseMatrix.coeff(1,2) * (pointWorldY - camWorldY))  + (poseMatrix.coeff(2,2) * (pointWorldZ - camWorldZ)) )); 

        yP   =-focalLenght * (( ( poseMatrix.coeff(0,1)*(pointWorldX - camWorldX) + poseMatrix.coeff(1,1) * (pointWorldY - camWorldY)  + poseMatrix.coeff(2,1) * (pointWorldZ - camWorldZ) ))/(( poseMatrix.coeff(1,2)*(pointWorldX - camWorldX) + poseMatrix.coeff(1,2) * (pointWorldY - camWorldY)  + poseMatrix.coeff(2,2) * (pointWorldZ - camWorldZ) ))); 


        std::cout << "xP = " << std::endl;
        std::cout <<  xP <<  std::endl;
        std::cout <<  "\n" << std::endl; 
        std::cout << "yP = " << std::endl; 
        std::cout << yP << std::endl; 
        std::cout <<"\n" << std::endl; 

}

coordinate tracker::doMinSquare(Eigen::MatrixXd matrixA_ , Eigen::MatrixXd matrixL_ ) //does the min square decompotion using the eigen library 
{
    Eigen::MatrixXd matrixA = matrixA_;
    Eigen::MatrixXd matrixL = matrixL_; 
    Eigen::MatrixXd result(2,1); 
    Eigen::MatrixXd temp; 
    coordinate location; 
    // std::cout << "here" << std::endl; 
    // std::cout << matrixA << std::endl; 
    // std::cout << matrixL << std::endl; 
//    (A.transpose() * A).ldlt().solve(A.transpose() * b) 
    // result = matrixA.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(matrixL); //least squares solving 
    //result = matrixA.ldlt().solve(matrixL);

    // result = (matrixA.transpose() * matrixA).ldlt().solve(matrixA.transpose() * matrixL);
    location.x = result.coeff(0,0);
    location.y = result.coeff(1,0); 
    temp = matrixA.transpose() * matrixA; 
   

    result = temp.inverse() *  matrixA.transpose() * matrixL;
    printf(" x result is %f \n", result.coeff(0,0 ));   
    printf(" Y result is %f \n", result.coeff(1,0 ));   

    location.x = result.coeff(0,0);
    location.y = result.coeff(1,0); 
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

void tracker::test()
{


        std::vector<Eigen::MatrixXd> rotationMatrices; 
        std::vector<coordinate  > GPSKeeper;
        std::vector<long double> planeheightKeeper;  
        std::vector <  coordinate > pixelKeeper; 
        coordinate result; 
        coordinate groundTruth = {241753.220826,	6160294.860516,	5.732144};

        /* point on which the testing is  done each matrix represents a rotation matrix   */ 

        Eigen::MatrixXd img35Matrix(3, 3);
        img35Matrix << 0.87178906, -0.47651387, -0.11365898,0.46938413,0.87892968, -0.08462365,0.14022259,0.02042425,0.98990933;
        coordinate img35GPS =  {241737.9833458330831490,	6160289.3355145351961255,	110.7603127174832451};  // x, y ,z 
        long double img35PlaneHeigth =5.732144; 
        coordinate img35Pixel = {40, -158}; 
        rotationMatrices.push_back(img35Matrix); 
        GPSKeeper.push_back(img35GPS); 
        planeheightKeeper.push_back(img35PlaneHeigth);
        pixelKeeper.push_back(img35Pixel); 
        

        Eigen::MatrixXd img36Matrix(3,3);
        img36Matrix << 0.7397178,-0.66367138,	-0.111166,0.66527495,0.74609534,-0.02740405,0.10112772,-0.05368469,0.99342395;
        coordinate img36GPS = {241710.6285522790276445,	6160303.0373303480446339,	113.3814467622704854}; 
        long double img36PlaneHeigth =5.732144; 
        coordinate img36Pixel = {503.6472411207838, -948.5236402396183};
        rotationMatrices.push_back(img36Matrix); 
        GPSKeeper.push_back(img36GPS); 
        planeheightKeeper.push_back(img36PlaneHeigth); 
        pixelKeeper.push_back(img36Pixel);
        


        Eigen::MatrixXd img37Matrix(3,3); 
        img37Matrix << 7.76978654e-01, -6.23715520e-01,-8.53412068e-02,6.29526438e-01,7.69976976e-01,1.04076510e-01,7.96629660e-04,-1.34589773e-01,9.90901084e-01;
        coordinate img37GPS = {241680.6798939024738502,	6160315.2465446069836617,	113.9804108923304256}; 
        long double img37PlaneHeigth =5.732144;
        coordinate img37Pixel = {1389.9298413006206, -1483.6601730397513};
        rotationMatrices.push_back(img37Matrix); 
        GPSKeeper.push_back(img37GPS); 
        planeheightKeeper.push_back(img37PlaneHeigth);
        pixelKeeper.push_back(img37Pixel);  
        

        Eigen::MatrixXd img55Matrix(3,3); 
        img55Matrix <<0.1534163,0.98763462,-0.03226912,-0.98816051,0.15338387,-0.00349262,0.00150013,0.0324229,0.99947311 ;
        coordinate img55GPS = {241718.5752740695024841,	6160345.4928991990163922,	110.1268070192413973};
        long double img55PlaneHeigth =5.732144;
        coordinate img55Pixel = {1938.4881078918133, 810.152025038569};
        rotationMatrices.push_back(img55Matrix); 
        GPSKeeper.push_back(img55GPS); 
        planeheightKeeper.push_back(img55PlaneHeigth); 
        pixelKeeper.push_back(img55Pixel); 
        

        Eigen::MatrixXd img56Matrix(3,3); 
        img56Matrix << 0.24265478,0.96977073,-0.02575655,-0.96982543,0.24314348,0.01788507,0.02360695,0.02063946,0.99950824;
        coordinate img56GPS = {241745.2931913038482890,	6160333.6742490641772747,	111.4323529299558828};
        long double img56PlaneHeigth =5.732144;
        coordinate img56Pixel = {1286.5726756857573, -136.4250031942299};
        rotationMatrices.push_back(img56Matrix); 
        GPSKeeper.push_back(img56GPS); 
        planeheightKeeper.push_back(img56PlaneHeigth); 
        pixelKeeper.push_back(img56Pixel); 
    
        

        Eigen::MatrixXd img57Matrix(3,3); 
        img57Matrix << 0.28759024,0.9572824,-0.03003756,-0.9576957,0.28777609,0.00196592,0.01052604,0.02820146,0.99954684;
        coordinate img57GPS = {241772.7440308686345816,	6160322.5347383925691247,	110.1915097883977239}; 
        long double img57PlaneHeigth =5.732144; 
        coordinate img57Pixel = {704.4296041259586, -1053.6265464797905};
        rotationMatrices.push_back(img57Matrix); 
        GPSKeeper.push_back(img57GPS); 
        planeheightKeeper.push_back(img57PlaneHeigth);
        pixelKeeper.push_back(img57Pixel); 
        


        Eigen::MatrixXd img58Matrix(3,3); 
        img58Matrix << 0.30626497,0.95107523,0.04071469,-0.95097731,0.30374396,0.05815288,0.04294092,-0.05652893,0.9974771; 
        coordinate img58GPS = {241801.3945700247422792,	6160310.8569446457549930,	112.0742519670662887}; 
        long double img58PlaneHeigth =5.732144;   
        coordinate img58Pixel = { -139.41008526816532, -1515.6939423784052};
        rotationMatrices.push_back(img58Matrix); 
        GPSKeeper.push_back(img58GPS); 
        planeheightKeeper.push_back(img58PlaneHeigth); 
        pixelKeeper.push_back(img58Pixel); 

        for(int i =0; i < 7; i++)
            {
                altitude = GPSKeeper[i].z;
                planeheight = planeheightKeeper[i];
                testPoseMatrix(rotationMatrices[i]);
                //getAMatrix({ -139, -1515}, true);
                //getLMatrix(GPSKeeper[i], pixelKeeper[i], true);
                std::cout << " image " << i << std::endl; 
                result =  doMinSquare(getAMatrix(pixelKeeper[i], false) , getLMatrix(GPSKeeper[i], pixelKeeper[i], false) );
                std::cout << "error in x " << std::endl; 
                std::cout << result.x - groundTruth.x << std::endl; 
                std::cout << "errpr in y" << std::endl; 
                std::cout << result.y - groundTruth.y << std::endl; 

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

