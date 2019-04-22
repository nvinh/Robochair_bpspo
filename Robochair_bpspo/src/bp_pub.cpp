#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "CerealPort.h" 
using namespace std;
void t(char* c, int t);
void t2(std::string* s);
void t3(std::string* s);
void t4(std::string* s);
void bpControlCallback(const std_msgs::String::ConstPtr& msg);

char P_response[20];	
char M_response[20];
char F_response[20];
char S_response[100];	
char res_type = 'Z'; 
int noOfCharInRes = 0; //index
std::string incomingData;

//status
int currentPressure;
int ID;
int sysPressure;
int mean;
int diaPressure;
int pulseRate;
int augmentationIndex;

//ros
bool resultsReceived = false;
bool errorReceived = false;
bool modeReceived = false;
int resultsPublished = 0; //a new result will be published 3 times
int errorPublished = 0;	
int modePublished = 0;	

int main (int argc, char **argv)
{
	ros::init(argc, argv, "bp_talker");
	ros::NodeHandle n("~");
	ros::Publisher bp_pub = n.advertise<std_msgs::String>("bp_results", 1000);
	ros::Rate loop_rate(10);
	//subscribing to bp_control
	ros::Subscriber sub = n.subscribe("bp_control", 1000, bpControlCallback);

	//retrieving ros parameter
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
	//Connecting to BP device
	////////////

	//serial port
	//cereal::CerealPort sport;
	cereal::CerealPort *sport = new cereal::CerealPort;
	sport->open(portArg.c_str(), 9600);
	//sport->write("s 170\r\n", 7);

	///////////////////////////////
	//reading from the serial port
	///////////////////////////////

	boost::function<void(char*, int)> f;
	f = &t;
	sport->startReadStream(f);

	boost::function<void(std::string*)> f2;
	f2 = &t2;
	//sport->startReadLineStream(f2);

	boost::function<void(std::string*)> f3;
	f3 = &t3;
	/*boost::function<void(std::string*)> f4;
	f4 = &t4;*/
	char start = 'P';
	char end = '\r';
	//sport->startReadBetweenStream(f3, start, end);	
	
	int count = 0;
	while(ros::ok())
	{
				
		std_msgs::String msg;
		std::stringstream ss;
		std::stringstream ss2;
		std::stringstream ss3;
		
		if(resultsReceived){
			if(resultsPublished<1)	{
				//ss << "BP results received" << count;
				ss << "BPResult_";
				//create the result string			
				int location=0;
				/*			
				while(S_response[location] !='\x020'){
					location++;
				}
				*/
				//reading ID, sys, mean, dia, pr, and AI			
				int i;
				for(i=0;i<7;i++){
					while(S_response[location] !='\x020'){
						ss<<S_response[location];				
						location++;
					}
					location++;
					ss<<"_";	
				}
				location =0;
				/*
				for(i=0; i < 100; i++){
				 ss << S_response[i];	
				}*/		
				msg.data = ss.str();
				ROS_INFO("%s", msg.data.c_str());
				bp_pub.publish(msg);
				std::cout << "msg.data: " << msg.data;	
				resultsPublished++; // a news result will be published only 3 times
			}else{
				resultsReceived=false;//result published and there are no new results
				resultsPublished = 0;
			}		
		}
		else if(errorReceived){
			if(errorPublished<1){
			//error object initialisation - ss2
			ss2<<"BPError_";
			int location=0;
			
			//reading Error
			std::cout << "F_response[0] : " << F_response[0] << "\n";
			std::cout << "F_response[1] : " << F_response[1] << "\n";
			std::cout << "F_response[2] : " << F_response[2] << "\n";
			std::cout << "F_response[3] : " << F_response[3] << "\n";
			std::cout << "F_response[4] : " << F_response[4] << "\n";
			switch (F_response[3]) {
			case '0':ss2 << "0_Failed to start_\n"; break;
			case '1':ss2 << "1_No valid systolic pressure_\n"; break;
			case '2':ss2 << "2_Button was pressed to cancel measurement_\n"; break;
			case '3':ss2 << "3_Failed to reinflate_\n"; break;
			case '4':ss2 << "4_Error during processing_\n"; break;
			case '5':ss2 << "5_Error during processing of suprasystolic signal_\n"; break;
			case '6':ss2 << "6_Error finding feature points_\n"; break;
			case '7':ss2 << "7_Processing finished successfully_\n"; break;
			case '8':ss2 << "8_Measurement does not exist_\n"; break;
			}
			location=0;

			msg.data=ss2.str();
			ROS_INFO("%s",msg.data.c_str());
			bp_pub.publish(msg);
			errorPublished++;
			//std::cout << "msg.data: " << msg.data;	
			}else{
				errorReceived=false;
				errorPublished = 0;
			}
			
		}else if(modeReceived){
			std::cout << "Mode received...\n";
			if(modePublished<1){
			std::cout << "Mode published <1 ...\n";	
			//mode object initialisation - ss3
			//ss3<<"BPMode_";
			int location=0;
			
			//reading Mode
			int i;
			std::cout << "M_response[0] : " << M_response[0] << "\n";
			std::cout << "M_response[1] : " << M_response[1] << "\n";
			std::cout << "M_response[2] : " << M_response[2] << "\n";
			std::cout << "M_response[3] : " << M_response[3] << "\n";
			std::cout << "M_response[4] : " << M_response[4] << "\n";
			switch (M_response[3]) {
			case '0':ss3 << "BPMo00_0_Initial%_\n"; break;
			case '1':ss3 << "BPMo01_1_Offline(no connection)_\n"; break;
			case '2':ss3 << "BPMo02_2_Ready%_\n"; break;
			case '3':ss3 << "BPMo03_3_Measuring blood presure_\n"; break;
			case '4':ss3 << "BPMo04_4_Deflating cuff after blood pressure measurement_\n"; break;
			case '5':ss3 << "BPMo05_5_Inflating to suprasystolic pressure_\n"; break;
			case '6':ss3 << "BPMo06_6_Acquiring suprasystolic pressure%_\n"; break;
			case '7':ss3 << "BPMo07_7_Processing data%_\n"; break;
			case '8':ss3 << "BPMo08_8_Displaying Storage Location menu%_\n"; break;
			case '9':ss3 << "BPMo09_9_Displaying Store /Recall menu_\n"; break;
			case '10':ss3 << "BPMo0A_A_Displaying Extra info menu_\n"; break;
			case '11':ss3 << "BPMo0B_B_Displaying settings menu_\n"; break;
			case '12':ss3 << "BPMo0C_C_SDisplaying Set target menu_\n"; break;
			}
			location=0;

			msg.data = ss3.str();
			ROS_INFO("%s", msg.data.c_str());
			bp_pub.publish(msg);
			modePublished++;
			std::cout << "msg.data: " << msg.data;
			}
			else {
				modeReceived = false;
				modePublished = 0;
			}
		}else{
			//
		}
        ros::spinOnce();
		loop_rate.sleep();
		++count;		
	}
    sport->close();
return 0;
}

void t(char* c, int t){
	resultsPublished = 0; // a new result is receved	
	char cc = c[0];	//response type
	switch(cc){
		case 'P':
			res_type = 'P';
			break;
		case 'M':
			res_type = 'M';
			break;
		case 'F':
			res_type = 'F';
			break;
		case 'S':
			res_type = 'S';
			break;
	}
	//std::cout << "res_type: " << res_type <<"\n";
	switch(res_type){
		case 'P':
			if(noOfCharInRes == 6){
				noOfCharInRes=0;
				res_type = 'Z';		
				//std::cout<<"Pressure: " << P_response[0] <<" " << P_response[1] <<" " << P_response[2] <<" " << P_response[3] <<" " << P_response[4] <<" " << P_response[5] <<"\n";
			}else{
				//Sstd::cout<<"Pressure response\n";
				P_response[noOfCharInRes]= cc;
				noOfCharInRes++;
			}
			break;
		case 'M':
			//if(noOfCharInRes == 5) {
			if(cc=='\x0a'){
			noOfCharInRes=0;
			res_type = 'Z';	
			std::cout<<"Mode\n";
			modeReceived = true;
		
			std::cout<<"Mode: " << M_response[0] <<" " << M_response[1] <<" " << M_response[2] <<" " << M_response[3] <<" " << M_response[4] <<"\n";
			}else{
				M_response[noOfCharInRes]= cc;
				noOfCharInRes++;
			}
			break;
		case 'F':
			//if(noOfCharInRes == 9) {
			if(cc=='\x0a'){
			noOfCharInRes=0;
			res_type = 'Z';
			std::cout<<"BPError\n";
			errorReceived = true;
			
			std::cout<<"BPError: " << F_response[0]<<""  << F_response[1]<<""<< F_response[2]<<""<< F_response[3]<<""<< F_response[4]<<""<< F_response[5]<<""<< F_response[6]<<""<< F_response[7]<<""<< F_response[8]<<"\n";
			}else{
				F_response[noOfCharInRes]= cc;
				noOfCharInRes++;
			}
			break;
		case 'S':
			//if(noOfCharInRes == 20) {
			if(cc == '\x0a') {
				noOfCharInRes=0;
				res_type = 'Z';			
				std::cout<<"Result received\n";
				resultsReceived = true;	
			}else{
				S_response[noOfCharInRes]= cc;
				noOfCharInRes++;
			}
			break;
	}
}
void t2(std::string* s){
	std::cout <<"Read (t2): " << s << "\n";
}

//void startReadBetweenStream(boost::function<void(std::string*)> f, char start, char end){
void t3(std::string* s){
	//std::cout<<"Read (t3): " << &s <<"\n";
	printf("R: %s", s);
}
/*void t4(std::string* s){
	//std::cout<<"Read (t4): " << &s <<"\n";
	printf("R: %s", s);
	
}*/

void responseRead(){

}

void bpControlCallback(const std_msgs::String::ConstPtr& msg)
{	
	//int i;
	cereal::CerealPort *sport=new cereal::CerealPort; 
	sport->open("/dev/ttyUSB0", 9600);
	//std::cout << "msg----" << msg;
	std::cout << "test3\n";
	std::string cmd;
	cmd=msg->data.c_str();
	cmd.resize (5);
	std::cout << "cmd length: " <<cmd.length()<<"\n";
	if (cmd == "start") {
		//cereal::CerealPort *sport=new cereal::CerealPort; 
		//sport->open("/dev/ttyUSB0", 9600);
		std::cout << "test2\n";
		sport->write("s 170\r\n", 7);

	}
	else if (cmd == "stopp") {
		//cereal::CerealPort *sport=new cereal::CerealPort; 
		//sport->open("/dev/ttyUSB0", 9600);
		std::cout << "test1\n";
		sport->write("c\r\n");
	}
	else if (cmd == "pause") {
		std::cout << "test5";

	}
	else {
		std::cout << "invalid command";
	}
	std::cout <<"Control command received ";
}


