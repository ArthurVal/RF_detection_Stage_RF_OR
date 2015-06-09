#ifndef RFSTUB_H 
#define RFSTUB_H

#include "RF_detection.h"
#include <time.h>

#ifndef M_PI
  #define M_PI 3.14159265
#endif 

class RF_stub : public RF_detection
{
	protected:
		double phiStub;		//Phi
		double thetaStub;	//Theta
		double rStub;			//Distance

		int nStub;				//Number of detection

			//Stub fake UART input
		virtual void getDataRF(); 

	public:
			//Constructor
		RF_stub(	ros::Publisher* chatter_line_rviz = NULL, 
									ros::Publisher* chatter_gauss = NULL,
									bool remote = false,  
									bool thetadis = true, 
									bool print = false);

			//Destructor
		~RF_stub(){};

}; //class RF_stub
#endif // RFSTUB_H
