
#include "tracker.hpp"


int main(int argc, char **argv)
{
    
        ros::init( argc, argv, "pathplanner");
        std::vector< std::vector<double>> a = { {1,2},
                                                {3,4}};
        std::vector<std::vector<double>> b = { {4,3},
                                                {2,1}};

       
        tracker tr; 
        tr.getposeMatrix(0, 0.349, 0); 
        std::cout << tr.poseMatrix << std::endl; 
       std::cout << tr.getAMatrix({0,0}) << std::endl;

        std::cout << tr.getLMatrix({150,500}, {0,0})<< std::endl;
         tr.doMinSquare(tr.getAMatrix({0,0}) , tr.getLMatrix({150,500}, {0,0}));



}