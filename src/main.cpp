
#include "ros/ros.h"
#include "ros/node_handle.h"
#include <visualization_msgs/Marker.h>

#include "RF_detection.h"

int main(int argc, char *argv[]){
	ROS_INFO("Initialization of node : RF_detection_node");
	ros::init(argc, argv, "RF_detection_node");
	ros::NodeHandle r;
	//ROS_INFO("Publishing to topic rf_detection");
	//ros::Publisher chatter_pub_gauss = n->advertise</*TODO MSG RF_DETECTION*/>("rf_detection", 1000);

	ROS_INFO("Publishing to topic rf_detection");
	ros::Publisher chatter_pub_line_rviz = r.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );

	
	ros::Rate loop_rate(15);

	RF_detection detector(argc,argv, &chatter_pub_line_rviz, NULL, true);	
	while(ros::ok()){
	
		detector.updateRF();

		ros::spinOnce();
		loop_rate.sleep();		
	}	
	return 0;
}
