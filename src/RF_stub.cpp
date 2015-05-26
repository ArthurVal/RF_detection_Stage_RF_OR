#include "RF_stub.h"

/*=================================================================================*/
/*----------------------------		 RF_stub::RF_stub()		---------------------------*/
/*=================================================================================*/

RF_stub::RF_stub(ros::Publisher* chatter_line_rviz, ros::Publisher* chatter_gauss, bool thetadis, bool print) 
: RF_detection(chatter_line_rviz, chatter_gauss, thetadis, print)
{	
	srand (time(NULL));
	nStub = rand() % 5 + 1; // between 1 & 5 detections
	phiStub = 0;
	thetaStub = 90;
	rStub = 1;
	ROS_INFO("[RF node] Stub mode activated");
}	


/*===================================================================================*/
/*===================================================================================*/
/*=============================== Protected functions ===============================*/
/*===================================================================================*/
/*===================================================================================*/



/*=================================================================================*/
/*------------------------		 RF_detection::getDataUART()		---------------------*/
/*------------------ Function that simulate the UART data input -------------------*/
/*=================================================================================*/


void RF_stub::getDataUART()
{ 

	if((iter%100) == 0) //Every 20 iteration, new nStub
		nStub = rand() % 6; //0 -> 5 detections

	data_uart_spherical_RF.n = nStub;

	data_intensity_map_RF_phi.index = data_intensity_map_RF_theta.index = iter;

	for(int j = 0 ; j < SIZE_DATA_RF ; ++j)
		data_intensity_map_RF_phi.intensity[j] = data_intensity_map_RF_theta.intensity[j] = 0;

	for(int i = 0 ; i < (data_uart_spherical_RF.n) ; ++i){ 
			//For all detection :

		rStub = 2 - sin(ros::Time::now().toSec() + 2*i);
		phiStub = 0 + 45 * cos(ros::Time::now().toSec() + 0.5*i);

		if(phiStub < -180){
			while(phiStub < -180){
				phiStub += 360;
			}
		}
		if(phiStub > 180){
			while(phiStub > 180){
				phiStub -= 360;
			}
		}

		if(!thetaDisable){
			thetaStub = 90 + 45 * sin(ros::Time::now().toSec() + i);

			if(thetaStub < 0){
				while(thetaStub < 0){
					thetaStub += 180;
				}
			}
			if(thetaStub > 180){
				while(thetaStub > 180){
					thetaStub -= 180;
				}
			}
		}
	

			//R
		data_uart_spherical_RF.dist[i] = rStub;
			//Phi
		data_uart_spherical_RF.phi[i] = phiStub;
			//Theta
		data_uart_spherical_RF.theta[i] = thetaStub;

				//Intensity map
		double sigma = rStub/2; // Ecart type entre 0.5 et 1.5 degres

		for(int j = 0 ; j < SIZE_DATA_RF ; ++j){
			data_intensity_map_RF_phi.intensity[j] = data_intensity_map_RF_phi.intensity[j] + (1.0/(sigma * sqrt(2*M_PI))) *
																																												exp(-0.5 * pow(((data_intensity_map_RF_phi.angle[j]-phiStub) / sigma),2) );			
		}
		for(int j = 0 ; j < (SIZE_DATA_RF/2) ; ++j){
			data_intensity_map_RF_theta.intensity[j] = data_intensity_map_RF_theta.intensity[j] + (1.0/(sigma * sqrt(2*M_PI))) *
																																												exp(-0.5 * pow(((data_intensity_map_RF_theta.angle[j]-thetaStub) / sigma),2) );			
		}



	}
}
