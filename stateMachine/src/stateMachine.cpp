

#include"stateMachine.hpp"

stateMachine::stateMachine()
{


}; 



void stateMachine::stateChanger()
{   
    
    switch(StateNumber)
    {   
        case 0:
            stateTakeOff(); 
            std::cout << " waiting for CMD " << std::endl;
            break; 

        case 1: 
            stateSearching(); 
            std::cout << " taking off, starting search" << std::endl; 
            break; 
        case 2:
            stateClassify(); 
            std::cout << " starting classifier" << std::endl; 
            break; 

        case 3:
             std::cout << " not of interest continuing " << std::endl; 
            break; 

        case 4: 
            stateTrack(); 
            std::cout << "of interest - tracking" << std::endl; 
            break; 

        default: std::cout << "default \n"  << std::endl; 
            break;     


    }


        // if(takeOffCMD == true)
        // {
        //     stateTakeOff(); 
        //     stateSearching(); 
        //     std::cout << " taking off, starting search" << std::endl; 
        // }
        // else  if(detector == true)
        // {
        //     stateClassify(); 
        //     std::cout << " starting classifier" << std::endl; 
        // }
        // else if( classifer != true)
        // {
        //     //continue searching 
        //     std::cout << " not of interest continuing " << std::endl; 
        // }
        // else if(classifer == true)
        // {
        //     stateTrack(); 
        //     std::cout << "of interest - tracking" << std::endl; 
        // }
        // else 
        // {
             
        //     std::cout << " waiting for CMD " << std::endl; 
        // }


}






void stateMachine::stateSearching()
{

    publish(areaSearchPUB, true); 


}


void stateMachine::stateClassify()
{
    publish(classifierPUB, true);
}



void stateMachine::stateTakeOff()
{
   
}


void stateMachine::stateTrack()
{
    publish(trackerDetectorPUB, true); 

}




void stateMachine::lowLevelDetectorCallBck(const std_msgs::Bool::ConstPtr& msg)
{

    lowLevelDetection = msg->data; 

    if(lowLevelDetection == true)
    {
        StateNumber = 2; 
    }


}

void stateMachine::takeOffCMDCallBck(const std_msgs::Bool::ConstPtr& msg)
{
    takeOffCMD = msg->data; 

    if(takeOffCMD == true)
    {
        StateNumber = 1; 
    }



}


void stateMachine::ClassifierCallBck(const std_msgs::Int8ConstPtr& msg) 
{

    int temp = msg->data; 

    if( temp > 0)
        {
            highLevelDection = true; 
        }
    else
        {
            highLevelDection = false; 
        }
    
    if(highLevelDection == true)
    {
        StateNumber = 4;  
    }

    if(highLevelDection == false)
    {
        StateNumber = 1; 
    }

}






void stateMachine::publish(ros::Publisher somePUB, bool someBoolean)
{      
    std_msgs::Bool tempMsg; 
    tempMsg.data = someBoolean; 

    somePUB.publish(tempMsg); 

}



stateMachine::~stateMachine()
{


}; 


