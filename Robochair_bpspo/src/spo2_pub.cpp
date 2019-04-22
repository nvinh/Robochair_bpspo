/**
 * Description: This ros node publishes extracts from the device spo2 values and publish it 
 *              It's not publishing the BPM value yet.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "CerealPort.h" 

void t(std::string* s);


//char SPO2_response;

//ros
bool resultsReceived = false;
std::string spo2Result;


int main (int argc, char **argv)
{
	ros::init(argc, argv, "spo2_talker");
    // To retrieve parameters in private namespace, I need to set the namespace of the nodehandle to the private name (~)
    // It is a solution for ROS resolve parameters in private namespace, which would otherwise be hard to resolve.
    // For example, we can't simply use ~port.
    // Solution was found here. http://www.ros.org/wiki/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle
	ros::NodeHandle n("~");
 	ros::Publisher spo2_pub = n.advertise<std_msgs::String>("spo2_results", 1000);
	ros::Rate loop_rate(10);
	std::string portArg;
    if (n.getParam("port", portArg))
    {
        ROS_INFO("Got param ~port: %s", portArg.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param '~port'");
    }
    
	////////////
	//Connecting to SP2 device
	////////////
	
	//serial port
	//cereal::CerealPort sport;
	cereal::CerealPort *sport=new cereal::CerealPort; 

	sport->open(portArg.c_str(), 9600);

	//sport->write("s 170\r\n", 7);
	
	///////////////////////////////	
	//reading from the serial port
	///////////////////////////////
		

	
	
	boost::function<void(std::string*)> f;
	f = &t;
	sport->startReadLineStream(f);

	std::cout<<"going into ros loop"<<std::endl;
	while(ros::ok())
	{
				
        if (spo2Result.compare("---")!=0)
        {
            std_msgs::String msgToSent;
            msgToSent.data = spo2Result;
            ROS_INFO("%s", msgToSent.data.c_str());
            spo2_pub.publish(msgToSent);
        }
	
		
		ros::spinOnce();

		loop_rate.sleep();
		

	}
    sport->close();
	
    return 0;
}


void t(std::string* s){
	//std::cout <<"Read (t2): " << *s << "\n";
    std::string str = *s;
    std::string spo2Str;
    std::string bpmStr;
    
    std::size_t foundSPO2 = str.find("SPO2=");         // position of spo2 keyword
    
    std::size_t foundBPM = str.find("BPM=");
    
    if (foundSPO2!=std::string::npos && foundBPM!=std::string::npos) // if found both tags.
    {
        spo2Str = str.substr (foundSPO2+5,3); //get Spo2 value
        bpmStr = str.substr(foundBPM+4,3); //get BPM value
         
     }
    spo2Result = spo2Str;
    //std::cout << spo2Str  <<  "            " << bpmStr << '\n';
}




