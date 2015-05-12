#include "RF_stub.h"

/*=================================================================================*/
/*----------------------------		 RF_stub::RF_stub()		----------------------------*/
/*=================================================================================*/

RF_stub::RF_stub(ros::Publisher* chatter_line_rviz, ros::Publisher* chatter_gauss, bool thetadis, bool print) : RF_detection(chatter_line_rviz, chatter_gauss, thetadis, print)
{	
	srand (time(NULL));
	nStub = rand() % 5 + 1;
	phiStub = 0;
	thetaStub = 90;
	rStub = 1;
	ROS_INFO("[RF node] Stub mode activated");
}	

/*===================================================================================*/
/*=============================== Protected functions ===============================*/
/*===================================================================================*/


/*=================================================================================*/
/*------------------------		 RF_detection::getDataUART()		-------------------------*/
/*------------------ Function that simulate the UART data input -------------------*/
/*=================================================================================*/

void RF_stub::getDataUART()
{ 

	data_uart_spherical.n = nStub;

	for(int i = 0 ; i < (data_uart_spherical.n) ; ++i){

		rStub = 2 - sin(ros::Time::now().toSec() + 2*i);
		phiStub = 45 * cos(ros::Time::now().toSec() + 2*i);
		if(!thetaDisable)
			thetaStub = 90 + 45 * sin(ros::Time::now().toSec() + i);

			//R
		data_uart_spherical.dist[i] = rStub;
			//Phi
		data_uart_spherical.phi[i] = phiStub;
			//Theta
		data_uart_spherical.theta[i] = thetaStub;
	}
}
