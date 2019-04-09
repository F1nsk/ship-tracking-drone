

#include"stateMachine.hpp"

stateMachine::stateMachine()
{


}; 



void stateMachine::stateChanger()
{   
    int state = 0;
    int i =0; 

    
    // while(ros::ok)
    // {   
    //     switch (state)
    //     {
    //         case 0:   //take off 
    //         stateTakeOff(); 
    //         break; 
                
              

    //         case 1:  //searching 
        
            




    //         case 2: //classify 




    //         case 4: //Track


               

    //     }



    //}

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


