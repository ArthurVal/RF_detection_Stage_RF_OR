#include "RF_detection.h"

/*=================================================================================*/
/*-----------------------		 RF_detection::RF_detection()		-----------------------*/
/*=================================================================================*/
RF_detection::RF_detection(int argc, char* argv[], ros::Publisher* chatter_line_rviz, ros::Publisher* chatter_gauss, bool stub)
{		
	chatter_pub_line_rviz = chatter_line_rviz;
	chatter_pub_gauss = chatter_gauss;

	enableStub = stub;
	if(enableStub)
		ROS_INFO("Stub mode activated");

	data_uart_spherical.n = 0;
	angle = 0;
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


		//=================//
		// Algo stub updateRF() //
		//=================//

		//Simulation de  dist & theta
		//Application changement référenciel RF => référenciel camera
		//Update msg & publish to rostopic


	
		//Get the UART data from RF detection
	if(!enableStub){
		//UART Acquisition

		//------------//
		//		TODO		//
		//------------//
		/*
		data_uart.n =;
		data_uart.dist_detection =;
		data_uart.theta =;
		data_uart.phi = 0;
		...
		*/

	}else{
		//Stub fake detection
		this->stubUART();		
	}

	if(data_uart_spherical.n == 0){
		return 0;
	}

  ROS_INFO("Detection RF : %d detection", data_uart_spherical.n);

			//Msg rviz detection
	visualization_msgs::Marker detect_rf[N_RF_MAX];
			//Msg rviz TEXT (attached to the line)
	visualization_msgs::Marker text_rf[N_RF_MAX];

	for(int i = 0 ; i < data_uart_spherical.n; ++i){

			//Initialization of outputs
		detect_rf[i].header.frame_id = text_rf[i].header.frame_id = "/camera_depth_frame";
		detect_rf[i].header.stamp = text_rf[i].header.stamp = ros::Time::now();
		std::ostringstream ID;
		ID << "RF_detection_" << i;
		detect_rf[i].ns = text_rf[i].ns = ID.str();

		detect_rf[i].id = 0;
		text_rf[i].id = 1;
	
		if(THETA_DISABLE == 1)
			detect_rf[i].type = visualization_msgs::Marker::LINE_LIST;
		else
			detect_rf[i].type = visualization_msgs::Marker::POINTS;

		text_rf[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		
		detect_rf[i].action = text_rf[i].action = visualization_msgs::Marker::ADD;

		detect_rf[i].pose.orientation.w = text_rf[i].pose.orientation.w = 1.0;
		
			//Scaling
		detect_rf[i].scale.x = 0.01;

		text_rf[i].scale.z = 0.1;

			//Color
				//Visible
		detect_rf[i].color.a = 1.0;
		text_rf[i].color.a = 1.0;
				//Red line & green text
		detect_rf[i].color.r = 1.0;
		text_rf[i].color.g = 1.0;

			//Create Point or Line
		geometry_msgs::Point p;
		if(THETA_DISABLE == 1){

			p.x = data_uart_cartesian.x[i];
			p.y = data_uart_cartesian.y[i] ;
			p.z = data_uart_cartesian.z[i] + 1;
			detect_rf[i].points.push_back(p);

			p.x = data_uart_cartesian.x[i];
			p.y = data_uart_cartesian.y[i];
			p.z = data_uart_cartesian.z[i] - 1;
			detect_rf[i].points.push_back(p);
		}else{
			p.x = data_uart_cartesian.x[i];
			p.y = data_uart_cartesian.y[i];
			p.z = data_uart_cartesian.z[i];
			detect_rf[i].points.push_back(p);
		}
		
		detect_rf[i].text = "Test";
			//Create Text
		text_rf[i].pose.position.x = data_uart_cartesian.x[i];
		text_rf[i].pose.position.y = data_uart_cartesian.y[i];
		text_rf[i].pose.position.z = p.z + 2.2;
		std::ostringstream textOutput;
		textOutput << "Detection RF  " << i << "\nR = " << data_uart_spherical.dist[i] << "\nAngle Phi = " << data_uart_spherical.phi[i] << "\nAngle Theta = " << data_uart_spherical.theta[i];
		text_rf[i].text = textOutput.str();

		chatter_pub_line_rviz->publish(detect_rf[i]);
		chatter_pub_line_rviz->publish(text_rf[i]);		
			
	}// for n_detection
	++iter;		
	return 1;
}





/*=================================================================================*/
/*=============================== Private functions ===============================*/
/*=================================================================================*/


/*=================================================================================*/
/*------------------------		 RF_detection::stubUART()		-------------------------*/
/*=================================================================================*/

void RF_detection::stubUART()
{
	data_uart_spherical.n = STUB_N_DETECTION;

	for(int i = 0 ; i < (data_uart_spherical.n) ; ++i){
		data_uart_spherical.dist[i] = 2;
		data_uart_spherical.phi[i] = angle;	//Angle between x & z (0 -> 360)

		if(i == 1)
			data_uart_spherical.phi[i] = -angle;

		data_uart_spherical.theta[i] = 90; //Angle between x & y (0 -> 180)
	}
	convToCart();

	if(angle > 45)
		angle = -45;
	else
		angle = (angle +1);

}

/*=================================================================================*/
/*------------------------		 RF_detection::convToCart()		-------------------------*/
/*=================================================================================*/

void RF_detection::convToCart()
{
		data_uart_cartesian.n = data_uart_spherical.n;
		for(int i = 0 ; i <= (data_uart_spherical.n-1) ; ++i){

		data_uart_cartesian.x[i] = data_uart_spherical.dist[i] * sin(data_uart_spherical.theta[i] * M_PI / 180.0) * cos(data_uart_spherical.phi[i] * M_PI / 180.0);
		
		data_uart_cartesian.y[i] = data_uart_spherical.dist[i] * sin(data_uart_spherical.theta[i] * M_PI / 180.0) * sin(data_uart_spherical.phi[i] * M_PI / 180.0);
	
		data_uart_cartesian.z[i] = data_uart_spherical.dist[i] * cos(data_uart_spherical.theta[i] * M_PI / 180.0); 
	}	
}









