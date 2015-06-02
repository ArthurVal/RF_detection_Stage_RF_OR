#ifndef RFDETECT_H 
#define RFDETECT_H

#include "ros/ros.h"
#include "ros/node_handle.h"
#include "rf_riddle/RF.h"
#include "rf_riddle/RFBase.h"
#include "rf_riddle/RFSetup.h"
#include "rf_riddle/getRFData.h"
#include <visualization_msgs/Marker.h>

#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cstring> 
#include <cmath>
#include <algorithm>

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
	private:
	
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
		bool isRemote;
		
		double M_basis_R[3][3];
		double Quaternion[4];
		double M_basis_T[3];

		double minTheta, maxTheta, minPhi, maxPhi, acquisitionTime;
		unsigned int nPoint;

		unsigned int iter;

		//TBD

			//Method
		virtual void getDataUART();
			//Initial Matrix (Edit this function to change the matrix basis change)
		void initBasisChangeMatrix();
		void convToCart();
		void convToCam();
		void RotToQuaternion(double* in_RotMatrix, double* out_QuaterVector);
		void QuaternionToRot(double* out_RotMatrix, double* in_QuaterVector);
		void printOutput();

	public:
			//Constructor
		RF_detection(	ros::Publisher* chatter_line_rviz = NULL, 
									ros::Publisher* chatter_gauss = NULL,  
									bool thetadis = true, 
									bool print = false);

		RF_detection(	ros::Publisher* chatter_line_rviz = NULL,
									bool thetadis = true, 
									bool print = false);

			//Destructor
		~RF_detection(){};

			//Main function (Acquisition UART -> Publishing result on topic "Marker rviz" + topic with gaussian distribution on "rf_riddle_intensity_map" if auto , else, if remote, respond to service call of "rf_riddle_intensity_map" service)
		bool updateRF(rf_riddle::getRFData::Request &req, rf_riddle::getRFData::Response &res);		
	
}; // class RF_detection 

#endif // RFDETECT_H
