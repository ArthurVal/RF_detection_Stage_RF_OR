
#include "ros/ros.h"
#include "ros/node_handle.h"
#include "rf_riddle/RF.h"
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cstring> 
#include "RF_detection.h" 
#include "RF_stub.h"

void printHelp(){
	std::cout << "====================================================================================" << std::endl;
	std::cout << "Input Arguments :" << std::endl;
	std::cout << "\t --stub : enable stub mode (fake UART detection)" << std::endl;
	std::cout << "\t --thetaDisable : Disable Theta for detection (2D detection R & Phi)" << std::endl;
	std::cout << "\t --verbose : print data" << std::endl;
	std::cout << "\t --help : show this help" << std::endl;
	std::cout << "====================================================================================" << std::endl;
}

int parseArgument(int argc, char** argv, bool* enableStub, bool* thetaDisable, bool* verbose, bool* remote){
	for(int i = 0 ; i < argc ; ++i){		
		if(strcmp ("--help",argv[i]) == 0){
			printHelp();
			return -1;
		}	

		if(strcmp ("--stub",argv[i]) == 0)
			*enableStub = true;

		if(strcmp ("--thetaDisable",argv[i]) == 0)
			*thetaDisable = true;

		if(strcmp ("--verbose",argv[i]) == 0)
			*verbose = true;

		if(strcmp ("--remote",argv[i]) == 0)
			*remote = true;		
	}
	return 0;
}

/*----------------------------------main---------------------------------------*/

int main(int argc, char *argv[]){

	bool stub = false;
	bool thetaDis = true;
	bool verbose = false;
	bool remote = true;
	RF_detection *detector;

	if(parseArgument(argc, argv, &stub, &thetaDis, &verbose, &remote) == -1)
		return 0;
	
	ROS_INFO("[RF node] Initialization of node : RF_detection_node");
	ros::init(argc, argv, "RF_detection_node");

	ros::NodeHandle r;

	ROS_INFO("[RF node] Publishing to topic visualization_marker for Rviz");
	ros::Publisher chatter_pub_line_rviz = r.advertise<visualization_msgs::Marker>("visualization_marker", 100 );

	ROS_INFO("[RF node] Publishing to topic rf_detection for RIDDLE_GUI");
	ros::Publisher chatter_rf = r.advertise<rf_riddle::RF>("rf_riddle_intensity_map_topic", 100 );

	if(stub){
		detector = new RF_stub(&chatter_pub_line_rviz,
													&chatter_rf,
													remote,
													thetaDis,
													verbose
													);
	}else{
		detector = new RF_detection(&chatter_pub_line_rviz,
																&chatter_rf,
																remote,
																thetaDis,
																verbose
																);	
	}	

	ROS_INFO("[RF node] Creation of service rf_riddle_intensity_map");
	ros::ServiceServer server_data_rf = r.advertiseService("rf_riddle_intensity_map_srv", &RF_detection::updateRF, detector);
	ROS_INFO("[RF node] Creation of service rf_riddle_set_param");
	ros::ServiceServer server_param_rf = r.advertiseService("rf_riddle_set_param", &RF_detection::updateRFParam, detector);


		//Loop rate frequency in Hz (correspond to the publishing frequency of the topic)
	ros::Rate loop_rate(10);

	while(ros::ok()){	
		if(!(detector->getIsRemote())){
				//Fake data, not used in auto mode
			rf_riddle::getRFData::Request req; 
			rf_riddle::getRFData::Response res;
			detector->updateRF(req,res);
		}
		detector->updateParam();
		ros::spinOnce();
		loop_rate.sleep();
	}		
	ROS_INFO("[RF node] ShutDown of node : RF_detection_node");
	return 0;
}
