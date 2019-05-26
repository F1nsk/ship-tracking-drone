
#include "tracker.hpp"


int main(int argc, char **argv)
{
    
        ros::init( argc, argv, "pathplanner");
        std::vector< std::vector<double>> a = { {1,2},
                                                {3,4}};
        std::vector<std::vector<double>> b = { {4,3},
                                                {2,1}};

        
        // Eigen::MatrixXd img57Matrix(3,3); 
        // img57Matrix << 0.287590235,-0.957695703,0.010526037,0.957282404,0.287776087,0.028201463,-0.030037562,0.001966,0.999547;

     
       
        tracker tr; 
        // tr.altitude =110.760312717483; 
        // tr.planeheight = 5.732144;
        // //tr.getposeMatrix(0, 1.5708, 0); // double yawAngle_, double pitchAngle_, double rollAngle_ 
        // std::cout << img35Matrix << std::endl;
        // tr.testPoseMatrix(img35Matrix); 
       // tr.getAMatrix({ -139, -1515}, true);
        // std::cout << "pose" << std::endl; 
        // std::cout << tr.poseMatrix << std::endl; 

        tr.test(); 
                
         //tr.doMinSquare(tr.getAMatrix({40.05056096,-158.4709264}, true) , tr.getLMatrix({241737.9833458330831490,6160289.3355145351961255}, {40.05056096,-158.4709264}, true) );


        

}