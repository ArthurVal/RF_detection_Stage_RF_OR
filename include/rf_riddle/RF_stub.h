#ifndef RFSTUB_H 
#define RFSTUB_H

#include "RF_detection.h"
#include <time.h>

#define STUB_N_DETECTION 2

#ifndef M_PI
  #define M_PI 3.14159265
#endif 

class RF_stub : public RF_detection
{
	protected:
		double phiStub;
		double thetaStub;
		double rStub;
		int nStub;
		virtual void getDataUART(); //New getDataUART() with stubmode

	public:
			//Constructor
		RF_stub(	ros::Publisher* chatter_line_rviz = NULL, 
									ros::Publisher* chatter_gauss = NULL,  
									bool thetadis = true, 
									bool print = false);

			//Destructor
		~RF_stub(){};

}; //class RF_stub
#endif // RFSTUB_H
