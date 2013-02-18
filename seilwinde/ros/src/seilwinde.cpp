/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob3_driver
 * ROS package name: cob_camera_axis
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author:
 *
 * Date of creation: Dez 2012
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointControllerState.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>
#include <cob_srvs/SetDefaultVel.h>

// external includes
#include <seilwinde/ElmoCtrl.h>

//####################
//#### node class ####
class NodeClass
{
	//
	public:
	// create a handle for this node, initialize node
	ros::NodeHandle n_;
		
	// declaration of topics to publish
	ros::Publisher topicPub_JointState_;
	ros::Publisher topicPub_Diagnostic_;
	
	// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointCommand_;
	
	// declaration of service servers
	ros::ServiceServer srvServer_Init_;
	ros::ServiceServer srvServer_Stop_;
	ros::ServiceServer srvServer_Recover_;
	//ros::ServiceServer srvServer_SetOperationMode_;
	ros::ServiceServer srvServer_SetDefaultVel_;
		
	// declaration of service clients
	//--

	// global variables
	ElmoCtrl * Seilwinde_;
	ElmoCtrlParams* SeilwindeParams_;
	
	std::string CanDevice_;
	std::string CanIniFile_;
	int CanBaudrate_;
	double MaxVelRadS_;
	int ModID_;
	double Offset_;
	int MotorDirection_;
	int EnoderIncrementsPerRevMot_;
	double GearRatio_;
	double WinchDiameter_;
	
	std::string JointName_;
	bool isInitialized_;
	bool isError_;
	bool finished_;
	double ActualPos_;
	double ActualVel_;

	// Constructor
	NodeClass(std::string name)	{
		n_ = ros::NodeHandle("/");
	
		isInitialized_ = false;
		isError_ = false;
		ActualPos_=0.0;
		ActualVel_=0.0;

		Seilwinde_ = new ElmoCtrl();
		SeilwindeParams_ = new ElmoCtrlParams();

		// implementation of topics to publish
		topicPub_JointState_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 1);
		topicPub_Diagnostic_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);


		// implementation of topics to subscribe
		topicSub_JointCommand_ = n_.subscribe("joint_command", 1, &NodeClass::topicCallback_JointStateCmd, this);
		
		// implementation of service servers
		srvServer_Init_ = n_.advertiseService("init", &NodeClass::srvCallback_Init, this);
		srvServer_Stop_ = n_.advertiseService("stop", &NodeClass::srvCallback_Stop, this);
		srvServer_Recover_ = n_.advertiseService("recover", &NodeClass::srvCallback_Recover, this);
		//srvServer_SetOperationMode_ = n_.advertiseService("set_operation_mode", &NodeClass::srvCallback_SetOperationMode, this);
		
		// implementation of service clients
		//--

		// read parameters from parameter server
		
		if(!n_.hasParam("EnoderIncrementsPerRevMot")) ROS_WARN("Used default parameter for EnoderIncrementsPerRevMot");
			n_.param<int>("EnoderIncrementsPerRevMot",EnoderIncrementsPerRevMot_, 8000);
		if(!n_.hasParam("CanDevice")) ROS_WARN("Used default parameter for CanDevice");
			n_.param<std::string>("CanDevice", CanDevice_, "/dev/pcan32");
		if(!n_.hasParam("CanBaudrateVal")) ROS_WARN("Used default parameter for CanBaudrateVal");
			n_.param<int>("CanBaudrateVal", CanBaudrate_, 1000);
		if(!n_.hasParam("ModId")) ROS_WARN("Used default parameter for ModId");
			n_.param<int>("ModId",ModID_, 1);
		if(!n_.hasParam("JointName")) ROS_WARN("Used default parameter for JointName");
			n_.param<std::string>("JointName",JointName_, "seilwinde");
		if(!n_.hasParam("MotorDirection")) ROS_WARN("Used default parameter for MotorDirection");
			n_.param<int>("MotorDirection",MotorDirection_, 1);
		if(!n_.hasParam("GearRatio")) ROS_WARN("Used default parameter for GearRatio");
			n_.param<double>("GearRatio",GearRatio_, 62.5);
		if(!n_.hasParam("MaxVelRadS")) ROS_WARN("Used default parameter for MaxVelRadS");
			n_.param<double>("MaxVelRadS",MaxVelRadS_, 1.0);
		if(!n_.hasParam("WinchDiameter")) ROS_WARN("Used default parameter for WinchDiameter");
			n_.param<double>("WinchDiameter",WinchDiameter_, 0.10);

		ROS_INFO("CanDevice=%s, CanBaudrate=%d, ModID=%d",CanDevice_.c_str(),CanBaudrate_,ModID_);

		//initializing and homing of camera_axis		
		SeilwindeParams_->SetCanIniFile(std::string("/"));
		SeilwindeParams_->SetMaxVel(MaxVelRadS_);
		SeilwindeParams_->SetGearRatio(GearRatio_);
		SeilwindeParams_->SetMotorDirection(MotorDirection_);
		SeilwindeParams_->SetEncoderIncrements(EnoderIncrementsPerRevMot_);
		
		//not used for velocity only mode
		SeilwindeParams_->SetHomingDir(1);
		SeilwindeParams_->SetHomingDigIn(1);
		
		//not used in velocity mode
		SeilwindeParams_->SetUpperLimit(0.0);
		SeilwindeParams_->SetLowerLimit(0.0);
		SeilwindeParams_->SetAngleOffset(0.0);
	
		SeilwindeParams_->Init(CanDevice_, CanBaudrate_, ModID_);
		

	}
	
	// Destructor
	~NodeClass() 
	{
		delete Seilwinde_;
	}

	// service callback functions
	// function will be called when a service is querried
	bool srvCallback_Init(cob_srvs::Trigger::Request &req,
				  cob_srvs::Trigger::Response &res )
	{
		if (isInitialized_ == false) {
			ROS_INFO("...initializing seilwinde...");
			// init powercubes 
			if (Seilwinde_->Init(SeilwindeParams_, false)) //false here means no home, as we dont need that in velicity mode
			{
				Seilwinde_->setGearVelRadS(0.0f);
				ROS_INFO("Initializing of seilwinde succesful");
				isInitialized_ = true;
				res.success.data = true;
				res.error_message.data = "initializing seilwinde successfull";
			}
			else
			{
				ROS_ERROR("Initializing seilwinde not succesful \n");
				res.success.data = false;
				res.error_message.data = "initializing seilwinde not successfull";
			}
		}
		else
		{
			ROS_WARN("...seilwinde already initialized...");			
			res.success.data = true;
			res.error_message.data = "seilwinde already initialized";
		}
		
		return true;
	}

	bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
				  cob_srvs::Trigger::Response &res )
	{
	if (isInitialized_ == true) {
		ROS_INFO("Stopping seilwinde");
	
		// stopping all arm movements
		if (Seilwinde_->Stop()) {
			ROS_INFO("Stopping seilwinde successful");
			res.success.data = true;
			res.error_message.data = "seilwinde stopped successfully";
		}
		else {
			ROS_ERROR("Stopping seilwinde not succesful. error");
			res.success.data = false;
			res.error_message.data = "stopping seilwinde not successful";
		}
	}

	return true;
	}
	
	bool srvCallback_Recover(cob_srvs::Trigger::Request &req,
				  	 cob_srvs::Trigger::Response &res )
	{
		if (isInitialized_ == true) {
			ROS_INFO("Recovering seilwinde");
			
			// stopping all arm movements
			if (Seilwinde_->RecoverAfterEmergencyStop()) {
				ROS_INFO("Recovering seilwinde succesful");
				res.success.data = true;
				res.error_message.data = "seilwinde successfully recovered";
			} else {
				ROS_ERROR("Recovering seilwinde not succesful. error");
				res.success.data = false;
				res.error_message.data = "recovering seilwinde failed";
			}
		} else {
			ROS_WARN("...seilwinde already recovered...");			
			res.success.data = true;
			res.error_message.data = "seilwinde already recovered";
		}

		return true;
	}

	/*!
	* \brief Executes the service callback for set_operation_mode.
	*
	* Changes the operation mode.
	* \param req Service request
	* \param res Service response
	*/
	/*bool srvCallback_SetOperationMode(	cob_srvs::SetOperationMode::Request &req,
										cob_srvs::SetOperationMode::Response &res )
	{
		ROS_INFO("Set operation mode to [%s]", req.operation_mode.data.c_str());
		n_.setParam("operation_mode", req.operation_mode.data.c_str());
		res.success.data = true; // 0 = true, else = false
		return true;
	}*/
	
	void topicCallback_JointStateCmd(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg) {
		//command winch speed in m/s of winding rope		
		if (isInitialized_ == true) {
			Seilwinde_->setGearVelRadS(msg->desired.velocities[0] / WinchDiameter_);
		}
	}
	
	void publishJointState()
	{
		if (isInitialized_ == true) {			
			// create message
			int DOF = 1;

			Seilwinde_->evalCanBuffer();
			Seilwinde_->getGearPosVelRadS(&ActualPos_,&ActualVel_);
			Seilwinde_->m_Joint->requestPosVel();

			sensor_msgs::JointState msg;
			msg.header.stamp = ros::Time::now();
			msg.name.resize(DOF);
			msg.position.resize(DOF);
			msg.velocity.resize(DOF);
			
			msg.name[0] = JointName_;
			msg.position[0] = ActualPos_;
			msg.velocity[0] = ActualVel_;

			//std::cout << "Joint " << msg.name[0] <<": pos="<<  msg.position[0] << " vel=" << msg.velocity[0] << std::endl;
				
			// publish message
			topicPub_JointState_.publish(msg);
		}
		// publishing diagnotic messages
	    diagnostic_msgs::DiagnosticArray diagnostics;
	    diagnostics.status.resize(1);
	    // set data to diagnostics
	    if(isError_)
	    {
	      diagnostics.status[0].level = 2;
	      diagnostics.status[0].name = "seilwinde";
	      diagnostics.status[0].message = "one or more drives are in Error mode";
	    }
	    else
	    {
	      if (isInitialized_)
	      {
	        diagnostics.status[0].level = 0;
	        diagnostics.status[0].name = n_.getNamespace(); //"schunk_powercube_chain";
	        diagnostics.status[0].message = "seilwinde initialized and running";
	      }
	      else
	      {
	        diagnostics.status[0].level = 1;
	        diagnostics.status[0].name = n_.getNamespace(); //"schunk_powercube_chain";
	        diagnostics.status[0].message = "seilwinde not initialized";
	      }
	    }
	    // publish diagnostic message
	    topicPub_Diagnostic_.publish(diagnostics);
	}


}; //NodeClass


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob_camera_axis");
	
	// create nodeClass
	NodeClass nodeClass(ros::this_node::getName());
 
	// main loop
 	ros::Rate loop_rate(100); // Hz
	while(nodeClass.n_.ok()) {
	  
		// publish JointState
		nodeClass.publishJointState();

		// sleep and waiting for messages, callbacks 
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

