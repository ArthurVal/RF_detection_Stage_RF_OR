#ifndef RFDETECT_H 
#define RFDETECT_H

#include "ros/ros.h"
#include "ros/node_handle.h"
#include <visualization_msgs/Marker.h>

#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cstring> 
#include <cmath>

#define N_RF_MAX 50

#define THETA_DISABLE 1



#ifndef M_PI
  #define M_PI 3.14159265
#endif 

typedef struct {
	unsigned char n;	
	double dist[N_RF_MAX], theta[N_RF_MAX], phi[N_RF_MAX];		
} rf_data_spherical; 

typedef struct {
	unsigned char n;	
	double x[N_RF_MAX], y[N_RF_MAX], z[N_RF_MAX];		
} rf_data_cartesian; 

class RF_detection 
{
	protected:
			//Attributs
		rf_data_spherical data_uart_spherical;
		rf_data_cartesian data_uart_cartesian;
		ros::Publisher* chatter_pub_line_rviz;
		ros::Publisher* chatter_pub_gauss;

		bool debug;
		bool thetaDisable;

		unsigned int iter;

		//TBD

			//Method
		virtual void getDataUART();
		void convToCart();
		void printOutput();

	public:
			//Constructor
		RF_detection(	ros::Publisher* chatter_line_rviz = NULL, 
									ros::Publisher* chatter_gauss = NULL,  
									bool thetadis = true, 
									bool print = false);

			//Destructor
		~RF_detection(){};

			//Main function (Acquisiiton UART -> Publishing result on topic "Marker rviz" + topic with gaussian distribution on "rf_detection")
		int updateRF();		
	
}; // class RF_detection 

#endif // RFDETECT_H
