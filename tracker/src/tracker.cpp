
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

std::vector<std::vector < double>> tracker::multiplyMatrix(std::vector<std::vector<double>> matrixA_, std::vector<std::vector<double>> matrixb_ )
{   

    std::vector<std::vector<double>> matrixA = matrixA_; 
    std::vector<std::vector<double>> matrixB = matrixb_;
    int rows =  matrixA.size();
    int cols = matrixA[0].size();
    std::vector<std::vector<double>> multiMatrix(rows, std::vector<double> (cols));




    if( matrixA[0].size() != matrixB.size())
        {
            std::cout << " matrix dimensions does not match \n" <<  std::endl; 
     
        }
    else
        {
            for(int i =0;  i < matrixA.size(); i++)
                {
                    for(int j =0; j < matrixB[0].size(); j++)
                        {
                            multiMatrix[i][j] = 0;
                            for(int k =0; k < matrixB.size(); k++)
                                {

                                    multiMatrix[i][j] += matrixA[i][k] * matrixB[k][j]; 
                                }
                        }
                }
        }
    return multiMatrix; 

}






std::vector<std::vector<double>> tracker::getYawMatrix(int yawAngle_)
{
  int yawAngle = yawAngle_;


  std::vector< std::vector<double>> yawMatrix =  { {cos(yawAngle), sin(yawAngle), 0},
                                                    {- sin(yawAngle), cos(yawAngle), 0},
                                                    {0, 0,1}

                                                  };


    return yawMatrix; 
}



std::vector<std::vector<double>> tracker::getPitchMatrix(int pitchAngle_)
{

    int pitchAngle = pitchAngle_; 

    std::vector< std::vector < double>> pitchMatrix = { {1, 0, 0},
                                                        { 0, cos(pitchAngle), -sin(pitchAngle)},
                                                        {0, sin(pitchAngle), cos(pitchAngle)}

                                                        };

    return pitchMatrix;             
}



std::vector< std::vector< double >> tracker::getRollMatrix(int rollAngle_ )
{

    int rollAngle = rollAngle_ ; 

    std::vector< std::vector < double>> rollMatric = { { cos(rollAngle), 0, sin(rollAngle)},
                                                        {0,1 ,0},
                                                        {-sin(rollAngle), 0, cos(rollAngle)} 
                                                        }; 

    return rollMatric;  
}


void tracker::printMatrix(std::vector<std::vector<double >> matrix_)
{

        std::vector< std::vector< double >> matrix = matrix_; 

        for (int i = 0; i < matrix.size(); i++ )
        {
            for(int j = 0; j < matrix[0].size(); j++)
            {
                std::cout << matrix[i][j] << " " ; 
              
            }
            std::cout << "\n"; 


        }


} 

std::vector<std::vector<double> >tracker::transpose(std::vector<std::vector<double>> matrix_)
{   
    
    std::vector<std::vector< double >> matrix = matrix_; 
    int rows = matrix.size(); 
    int cols  = matrix[0].size(); 
    std::vector<std::vector<double>> tranposeMatrix(rows, std::vector<double> (cols));

    for (int i = 0; i < cols; i ++)
        {
            for(int j = 0; j  < rows; j++)
                {
                    tranposeMatrix[i][j] = matrix[j][i]; 
                }
        } 
    
    
    return tranposeMatrix;  




}




tracker::~tracker()
{

}