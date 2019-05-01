
#include "tracker.hpp"


int main(int argc, char **argv)
{
    
        ros::init( argc, argv, "pathplanner");
        std::vector< std::vector<double>> a = { {1,2},
                                                {3,4}};
        std::vector<std::vector<double>> b = { {4,3},
                                                {2,1}};

        Eigen::MatrixXd img35Matrix(3, 3);
        img35Matrix << 0.87178906, -0.47651387, -0.11365898, 0.46938413,0.87892968 ,-0.08462365,0.14022259,0.02042425,0.98990933;
        // Eigen::MatrixXd img57Matrix(3,3); 
        // img57Matrix << 0.287590235,-0.957695703,0.010526037,0.957282404,0.287776087,0.028201463,-0.030037562,0.001966,0.999547;

        Eigen::MatrixXd test;  
       
        tracker tr; 
        tr.altitude =110.760312717483; 
        tr.planeheight = 5.732144;
        //tr.getposeMatrix(0, 1.5708, 0); // double yawAngle_, double pitchAngle_, double rollAngle_ 
        std::cout << img35Matrix << std::endl;
        tr.testPoseMatrix(img35Matrix); 
        
        std::cout << "pose" << std::endl; 
        std::cout << tr.poseMatrix << std::endl; 


                
         tr.doMinSquare(tr.getAMatrix({40.05056096,-158.4709264}, true) , tr.getLMatrix({241737.9833458330831490,6160289.3355145351961255}, {40.05056096,-158.4709264}, true) );


}