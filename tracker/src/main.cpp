
#include "tracker.hpp"


int main(int argc, char **argv)
{
    
        ros::init( argc, argv, "pathplanner");
        std::vector< std::vector<double>> a = { {1,2},
                                                {3,4}};
        std::vector<std::vector<double>> b = { {4,3},
                                                {2,1}};

       
        tracker tr; 

       tr.printMatrix(tr.transpose(a)); 



}