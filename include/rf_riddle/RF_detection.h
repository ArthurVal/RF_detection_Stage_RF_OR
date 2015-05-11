#ifndef RFDETECT_H 
#define RFDETECT_H

#include "ros/ros.h"
#include "ros/node_handle.h"
#include <visualization_msgs/Marker.h>

#include <sstream>
#include <stdlib.h>
#include <string> 
#include <cmath>

#define N_RF_MAX 10

#define STUB_N_DETECTION 2
#define STUB_FREQ 0.5

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
	private:
			//Attributs
		rf_data_spherical data_uart_spherical;
		rf_data_cartesian data_uart_cartesian;
		ros::Publisher* chatter_pub_line_rviz;
		ros::Publisher* chatter_pub_gauss;
		bool enableStub;
		int angle;
		unsigned int iter;
		//TBD

			//Method
		void stubUART();
		void convToCart();


	protected:
	public:
			//Constructor
		RF_detection(int argc, char* argv[], ros::Publisher* chatter_line_rviz, ros::Publisher* chatter_gauss, bool stub = true);

			//Destructor
		~RF_detection(){};

			//Main function (Acquisiiton UART or Stub -> Publishing result on topic "Marker rviz" + topic with gaussian distribution on "rf_detection")
		int updateRF();
	
}; // class RF_detection 

#endif // RFDETECT_H
