/**
 * @file    scenario.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   User Scenario
 *
 * Scenario is defining containing int main() and defining the control scenario
 */

#include "Scheduler.h"
#include "Agent.h"
#include "Constraint.h"
#include "Coupling.h"

#include "Ur5e_IKT.h"

#include "Cmscgmres.h"
#include "Event.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv) {

//Initialize ros node
	ros::init(argc, argv, "controller");

	/****** Initialize Agent Instances ******/
	std::vector<Agent*> agentlist;


	//Initialize: Turtle instance: ardrone1
	Ur5e_IKT* ur5e = new Ur5e_IKT(agentlist.size());
	//Ur5e state={x,y,yaw} input={uforward,urotate}
	//Define penaltys of state Q and inputs R
	double ur5e_init_p[] = {
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, /*Q*/
		1.0, 0.5, 0.0, 0.0, 0.0, 0.0    /*R*/
	};
	//Initial desired Pose
	double ur5e_init_xdes[] = {1.57, -1.57, 1.57,-1.57,1.57,1.57};

	ur5e->setInitialDesiredState(ur5e_init_xdes);
	ur5e->setInitialParameter(ur5e_init_p);
	ur5e->setStateSubscriberRosTopicName       ("/joint_states");   // /joint_states
	ur5e->setDesiredStateSubscriberRosTopicName("/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal"); // //scaled_pos_joint_traj_controller/follow_joint_trajectory/goal
	ur5e->setPublisherRosTopicName             ("/scaled_pos_joint_traj_controller/follow_joint_trajectory/"); // /scaled_pos_joint_traj_controller/follow_joint_trajector/scaled_pos_joint_traj_controller/follow_joint_trajectory
	agentlist.push_back(ur5e); /*add to agentlist*/

	/****** Initialize Coupling Instances ******/
	std::vector<Controller*> controllerlist;

	//Initialize: Controller
	Cmscgmres* controller1 = new Cmscgmres(agentlist, controllerlist.size());
	controller1->setHorizonDiskretization(10);
	controller1->setHorizonLength(1);
	controller1->setTolerance(1e-8);
	controller1->setUpdateIntervall(0.01);
	controller1->setMaximumNumberofIterations(10);
	controller1->activateInfo_ControllerStates();
	//	controller1->activateInfo_ControllerTrace();
	//	controller1->activateInfo_Controller();
	//	controller1->startLogging2File();
	controllerlist.push_back(controller1);


	/****** Initialize Events ******/
	std::vector<Event*> eventlist;

	/****** Initialize Scheduler ******/
	Scheduler scheduler(argc, argv, controllerlist, eventlist);
	scheduler.run_control(0.01);
//	scheduler.run_vrep(0.01);
//	scheduler.run_simulation(0.1,10);

};



