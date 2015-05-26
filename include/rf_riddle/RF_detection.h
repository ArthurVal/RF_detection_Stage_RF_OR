#ifndef RFDETECT_H 
#define RFDETECT_H

#include "ros/ros.h"
#include "ros/node_handle.h"
#include "rf_riddle/RF.h"
#include "rf_riddle/RFBase.h"
#include <visualization_msgs/Marker.h>

#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cstring> 
#include <cmath>

#define N_RF_MAX 50
#define SIZE_DATA_RF 360 //360 Points => 1Pts/Degre

#ifndef M_PI
  #define M_PI 3.14159265
#endif 

typedef struct {
	unsigned char n;	
	double dist[N_RF_MAX], theta[N_RF_MAX], phi[N_RF_MAX];		
} rf_detection_spherical; 

typedef struct {
	unsigned char n;	
	double x[N_RF_MAX], y[N_RF_MAX], z[N_RF_MAX];		
} rf_detection_cartesian; 

typedef struct {
	unsigned int index;	
	double angle[SIZE_DATA_RF], intensity[SIZE_DATA_RF];		
} rf_intensity_map; 

class RF_detection 
{
	
	protected:
			//Attributs
		rf_detection_spherical data_uart_spherical_RF;
		rf_detection_cartesian data_uart_cartesian_RF;

		rf_detection_spherical data_uart_spherical_camera;
		rf_detection_cartesian data_uart_cartesian_camera;

		rf_intensity_map data_intensity_map_RF_phi;
		rf_intensity_map data_intensity_map_RF_theta;

		ros::Publisher* chatter_pub_line_rviz;
		ros::Publisher* chatter_pub_gauss;

		bool verbose;
		bool thetaDisable;
		
		double M_basis[4][4];

		unsigned int iter;

		//TBD

			//Method
		virtual void getDataUART();
		void convToCart();
		void convToCam();
		void printOutput();

	public:
			//Constructor
		RF_detection(	ros::Publisher* chatter_line_rviz = NULL, 
									ros::Publisher* chatter_gauss = NULL,  
									bool thetadis = true, 
									bool print = false);

			//Destructor
		~RF_detection(){};

			//Main function (Acquisition UART -> Publishing result on topic "Marker rviz" + topic with gaussian distribution on "rf_detection")
		int updateRF();		
	
}; // class RF_detection 

#endif // RFDETECT_H
