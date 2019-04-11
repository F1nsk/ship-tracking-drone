

#include"stateMachine.hpp" 





int main(int argc, char **argv)
{

    ros::init( argc, argv, "statemachine");

    stateMachine sm;     

    ros::Rate loop_rate(10);

    ros::Subscriber sub = sm.n.subscribe("/oject_detector", 1, &stateMachine::ClassifierCallBck, &sm); 
    ros::Subscriber sub1 = sm.n.subscribe("/lowlevel", 1, &stateMachine::lowLevelDetectorCallBck, &sm); 
    ros::Subscriber sub2 = sm.n.subscribe("/takeOffCMD",1 , &stateMachine::takeOffCMDCallBck, &sm); 


 
    while(ros::ok)
    {

        sm.stateChanger(); 
        loop_rate.sleep();
            
        ros::spinOnce(); 

    }


}