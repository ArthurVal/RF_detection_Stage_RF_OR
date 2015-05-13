#include "RF_detection.h"

/*=================================================================================*/
/*-----------------------		 RF_detection::RF_detection()		-----------------------*/
/*=================================================================================*/
RF_detection::RF_detection(ros::Publisher* chatter_line_rviz, ros::Publisher* chatter_gauss, bool thetadis, bool print)
{	
		
	chatter_pub_line_rviz = chatter_line_rviz;
	chatter_pub_gauss = chatter_gauss;

		//Set to default (May be changed in parseArgument)
	thetaDisable = thetadis;
	verbose = print;

	if(verbose)
		ROS_INFO("[RF node] Verbose activated");
	if(thetaDisable)
		ROS_INFO("[RF node] 2D RF data Acquisition");

	data_uart_spherical_camera.n = 0;
	data_uart_spherical_RF.n = 0;	
	data_uart_cartesian_camera.n = 0;
	data_uart_cartesian_RF.n = 0;
	
	data_intensity_map_RF.index = 0;
	for(int i = 0 ; i < SIZE_DATA_RF ; ++i){
		data_intensity_map_RF.intensity[i] = 0;
		data_intensity_map_RF.phi[i] = i * (360.0/SIZE_DATA_RF);	
	}	

	M_basis[0][0] = 1;
	M_basis[0][1] = 0;
	M_basis[0][2] = 0;

	M_basis[0][3] = 0;

	M_basis[1][0] = 0;
	M_basis[1][1] = 1;
	M_basis[1][2] = 0;

	M_basis[1][3] = 0;

	M_basis[2][0] = 0;
	M_basis[2][1] = 0;
	M_basis[2][2] = 1;

	M_basis[2][3] = 0;

	ROS_INFO("[RF node] Matrix corresponding to the change of basis (RF -> Camera):");
	ROS_INFO("%f  %f  %f  %f",M_basis[0][0],M_basis[0][1],M_basis[0][2],M_basis[0][3]);
	ROS_INFO("%f  %f  %f  %f",M_basis[1][0],M_basis[1][1],M_basis[1][2],M_basis[1][3]);
	ROS_INFO("%f  %f  %f  %f",M_basis[2][0],M_basis[2][1],M_basis[2][2],M_basis[2][3]);

	iter = 0;	

}


/*=================================================================================*/
/*--------------------------		 RF_detection::run()		---------------------------*/
/*=================================================================================*/

int RF_detection::updateRF()
{
		//============//
		// Algo updateRF() //
		//============//

		//Récuperation valeur I, Q & Theta
		//Conversion I,Q => dist
		//Application changement référenciel RF => référenciel camera
		//Update msg & publish to rostopic
	
		//Get the UART data from RF detection
	getDataUART();

	if(data_uart_spherical_RF.n == 0){
		return 0;
	}
		//Convert data to cartesian coordinates	
	this->convToCart();

		//Convert to the camera referential
	this->convToCam();

	if(true){
		ROS_INFO("---------------------------------------");
  	ROS_INFO("Iter %d | Detection RF : %d detection", iter, data_uart_spherical_camera.n);
	}

			//Msg rviz detection initialization
	visualization_msgs::Marker detect_rf[N_RF_MAX];
			//Msg rviz TEXT (attached to the line) initialization
	visualization_msgs::Marker text_rf[N_RF_MAX];


	for(int i = 0 ; i < data_uart_spherical_camera.n; ++i){
		//For all marker detect by the RF do :

			//Initialization of outputs
		detect_rf[i].header.frame_id = text_rf[i].header.frame_id = "/camera_depth_frame";
		detect_rf[i].header.stamp = text_rf[i].header.stamp = ros::Time::now();

		std::ostringstream ID;
		ID << "RF_detection_" << i;
		detect_rf[i].ns = text_rf[i].ns = ID.str();

		detect_rf[i].id = 0;
		text_rf[i].id = 1;
	
		if(thetaDisable)
			detect_rf[i].type = visualization_msgs::Marker::LINE_LIST;
		else
			detect_rf[i].type = visualization_msgs::Marker::POINTS;

		text_rf[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		
		detect_rf[i].action = text_rf[i].action = visualization_msgs::Marker::ADD;

		detect_rf[i].pose.orientation.w = text_rf[i].pose.orientation.w = 1.0;
		
			//Scaling
		detect_rf[i].scale.x = 0.05;
		text_rf[i].scale.z = 0.07;

			//Color
				//Visible
		detect_rf[i].color.a = 1.0;
		text_rf[i].color.a = 1.0;
				//Red line & white text
		detect_rf[i].color.r = 1.0;
		text_rf[i].color.g = 1.0;
		text_rf[i].color.r = 1.0;
		text_rf[i].color.b = 1.0;

			//Create cartesian point detected
		geometry_msgs::Point p;

		if(thetaDisable){

				//Creating a line, require 2 point
			p.x = data_uart_cartesian_camera.x[i];
			p.y = data_uart_cartesian_camera.y[i] ;
			p.z = data_uart_cartesian_camera.z[i] + 1;
			detect_rf[i].points.push_back(p);

			p.x = data_uart_cartesian_camera.x[i];
			p.y = data_uart_cartesian_camera.y[i];
			p.z = data_uart_cartesian_camera.z[i] - 1;
			detect_rf[i].points.push_back(p);

		}else{
			p.x = data_uart_cartesian_camera.x[i];
			p.y = data_uart_cartesian_camera.y[i];
			p.z = data_uart_cartesian_camera.z[i];
			detect_rf[i].points.push_back(p);
		}

			//Create Text (Same position than detection)
		text_rf[i].pose.position.x = data_uart_cartesian_camera.x[i];
		text_rf[i].pose.position.y = data_uart_cartesian_camera.y[i];
		text_rf[i].pose.position.z = p.z - 0.2;

		std::ostringstream textOutput;
		textOutput << "Detection RF  " << i << "\nR = " << data_uart_spherical_camera.dist[i] << "\nAngle Phi = " << data_uart_spherical_camera.phi[i] << "\nAngle Theta = " << data_uart_spherical_camera.theta[i];

		text_rf[i].text = textOutput.str();

			//Publish to ROS
		detect_rf[i].lifetime = text_rf[i].lifetime = ros::Duration(0.5);
	



		if(chatter_pub_line_rviz){
			chatter_pub_line_rviz->publish(detect_rf[i]);
			chatter_pub_line_rviz->publish(text_rf[i]);
		}
	


	}// for n_detection

		//Msg of the intensity map for the GUI
	rf_riddle::RF intensity_map_rf;

	intensity_map_rf.index = data_intensity_map_RF.index;	
	for(int i = 0 ; i < SIZE_DATA_RF ; ++i){
		intensity_map_rf.phi.push_back(data_intensity_map_RF.phi[i]);
		intensity_map_rf.intensity.push_back(data_intensity_map_RF.intensity[i]);
	}

	if(chatter_pub_gauss)
		chatter_pub_gauss->publish(intensity_map_rf);
	
	
	if(verbose)
		printOutput();

	++iter;
		
	return 1;
}

/*===================================================================================*/
/*===================================================================================*/
/*=============================== Protected functions ===============================*/
/*===================================================================================*/
/*===================================================================================*/



/*=================================================================================*/
/*---------------------		 RF_detection::getDataUART()		-------------------------*/
/*------------------ Function that get the UART data input ------------------------*/
/*=================================================================================*/

void RF_detection::getDataUART()
{
	std::cout << "==== UART not implemented yet ====" << std::endl;
	/*TODO*/
}

/*=================================================================================*/
/*-----------------------		 RF_detection::convToCart()		-------------------------*/
/*---------- Classical conversion from spheric coordinates to cartesian -----------*/
/*=================================================================================*/

void RF_detection::convToCart()
{
		data_uart_cartesian_RF.n = data_uart_spherical_RF.n;

		for(int i = 0 ; i <= (data_uart_spherical_RF.n-1) ; ++i){

		data_uart_cartesian_RF.x[i] = data_uart_spherical_RF.dist[i] * sin(data_uart_spherical_RF.theta[i] * M_PI / 180.0) * cos(data_uart_spherical_RF.phi[i] * M_PI / 180.0);
		
		data_uart_cartesian_RF.y[i] = data_uart_spherical_RF.dist[i] * sin(data_uart_spherical_RF.theta[i] * M_PI / 180.0) * sin(data_uart_spherical_RF.phi[i] * M_PI / 180.0);
	
		data_uart_cartesian_RF.z[i] = data_uart_spherical_RF.dist[i] * cos(data_uart_spherical_RF.theta[i] * M_PI / 180.0); 
	}	
}

/*=================================================================================*/
/*-----------------------		 RF_detection::convToCam()		-------------------------*/
/*------------ Conversion from RF referential to camera referential----------------*/
/*=================================================================================*/

void RF_detection::convToCam()
{
	data_uart_spherical_camera.n = data_uart_spherical_RF.n;
	data_uart_cartesian_camera.n = data_uart_cartesian_RF.n;

	for(int i = 0 ; i <= (data_uart_spherical_RF.n-1) ; ++i){

			//Transform (TODO : Determiner la matrice de passage d'un referentiel à l'autre via le calibrage)
		data_uart_cartesian_camera.x[i] = data_uart_cartesian_RF.x[i]*M_basis[0][0] 
																		+ data_uart_cartesian_RF.y[i]*M_basis[0][1]
																		+ data_uart_cartesian_RF.z[i]*M_basis[0][2]
																		+ M_basis[0][3];

		data_uart_cartesian_camera.y[i] = data_uart_cartesian_RF.x[i]*M_basis[1][0] 
																		+ data_uart_cartesian_RF.y[i]*M_basis[1][1]
																		+ data_uart_cartesian_RF.z[i]*M_basis[1][2]
																		+ M_basis[1][3];

		data_uart_cartesian_camera.z[i] = data_uart_cartesian_RF.x[i]*M_basis[2][0] 
																		+ data_uart_cartesian_RF.y[i]*M_basis[2][1]
																		+ data_uart_cartesian_RF.z[i]*M_basis[2][2]
																		+ M_basis[2][3];


				//Spherical Update
		data_uart_spherical_camera.dist[i] = sqrt(pow(data_uart_cartesian_camera.x[i],2) 
																						+ pow(data_uart_cartesian_camera.y[i],2) 
																						+ pow(data_uart_cartesian_camera.z[i],2)
																					 		);

		data_uart_spherical_camera.phi[i] = atan2(data_uart_cartesian_camera.y[i],
																							data_uart_cartesian_camera.x[i]
																							) * 180 / M_PI;
		data_uart_spherical_camera.theta[i] = acos(data_uart_cartesian_camera.z[i] /
																							(data_uart_spherical_camera.dist[i]) 
																							) * 180 / M_PI;		
	}	
}






/*=================================================================================*/
/*-----------------------		 RF_detection::printOutput()		-------------------------*/
/*---------------------- Print coordinates for debug purpose ----------------------*/
/*=================================================================================*/

void RF_detection::printOutput()
{

	
	ROS_INFO(">>>>>>>>>>>><<<<<<<<<<<<");	
	ROS_INFO("> RF node verbose: ");
	ROS_INFO(">> Iteration %d",iter);
	if(thetaDisable)
		ROS_INFO(">> Theta angle disable (2D detection)");
	ROS_INFO(">> Number of detection : %d", data_uart_spherical_camera.n);
	
	if(data_uart_spherical_camera.n >= 1){
		for(int i = 0; i < data_uart_spherical_camera.n ; ++i){
			ROS_INFO("------------------------");			
			ROS_INFO(">> Detection %d :", i);		
			ROS_INFO(">>> Spherical :");			
			ROS_INFO(">> R = %f", data_uart_spherical_camera.dist[i]);			
			ROS_INFO(">> Phi = %f", data_uart_spherical_camera.phi[i]);			
			ROS_INFO(">> Theta = %f", data_uart_spherical_camera.theta[i]);
			ROS_INFO(">>> Cartesian :");			
			ROS_INFO(">> x = %f", data_uart_cartesian_camera.x[i]);			
			ROS_INFO(">> y = %f", data_uart_cartesian_camera.y[i]);			
			ROS_INFO(">> z = %f", data_uart_cartesian_camera.z[i]);			
		}
		ROS_INFO("------------------------");
		ROS_INFO(">>>>>>>>>>>><<<<<<<<<<<<");				
	}
}

