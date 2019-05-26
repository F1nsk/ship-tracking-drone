

#include"stateMachine.hpp" 





int main(int argc, char **argv)
{

    ros::init( argc, argv, "statemachine");

    stateMachine sm;     
    // image_transport::ImageTransport it(sm.nH);


    ros::Rate loop_rate(10);

    ros::Subscriber sub = sm.nH.subscribe("/oject_detector", 1, &stateMachine::ClassifierCallBck, &sm); 
    ros::Subscriber sub1 = sm.nH.subscribe("/low_lvl_detector", 1, &stateMachine::lowLevelDetectorCallBck, &sm); 
    ros::Subscriber sub2 = sm.nH.subscribe("/takeOffCMD",1 , &stateMachine::takeOffCMDCallBck, &sm); 
    // image_transport::Subscriber subImg = it.subscribe("/drone/camera/image_raw", 1, &stateMachine:: imageCallback, &sm);



 
    while(ros::ok)
    {

        sm.stateChanger(); 
        loop_rate.sleep();
            
        ros::spinOnce(); 

    }


}