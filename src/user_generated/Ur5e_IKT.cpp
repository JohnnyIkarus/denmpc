
/**
 * @file    Ur5e_IKT.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    Friday 24 November 2017
 * @time    11:07:32
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 *          Description of agent
 */

#include <Ur5e_IKT.h>

/*******************************************************
 * Constructor                                         *
 *******************************************************/
Ur5e_IKT::Ur5e_IKT(int id): Agent(id) {
    //Set initial values
    dim_x_ = 6;     //States
    dim_xdes_ = 6;  //Desired States
    dim_u_ = 6;     //Controls , joint_angles
    dim_udes_ = 6;  //Desired Controls
    dim_y_ = 0;
    dim_ydes_ = 0;
    dim_p_ = 5;     // Parameters: was genau sind das?
    dim_d_ = 0;     //Disturbance
    dim_l_ = 1;     // ?
    dim_v_ = 1;     // ?
    dim_eq_ = 0;    // ?
    dim_ineq_ = 2;
    //Allocate vectors
    x_     = defvector(dim_x_);
    xdes_  = defvector(dim_xdes_);
    u_     = defvector(dim_u_);
    udes_  = defvector(dim_udes_);
    y_     = defvector(dim_y_);
    ydes_  = defvector(dim_ydes_);
    d_     = defvector(dim_d_);
    p_     = defvector(dim_p_);
    x_init_    = defvector(dim_x_);
    xdes_init_ = defvector(dim_xdes_);
    u_init_    = defvector(dim_u_);
    udes_init_ = defvector(dim_udes_);
    y_init_    = defvector(dim_y_);
    ydes_init_ = defvector(dim_ydes_);
    d_init_    = defvector(dim_d_);
    p_init_    = defvector(dim_p_);

    //DEFINE ROS_NODE
    std::stringstream ss_ros_namespace;
    ss_ros_namespace << "Ur5e_IKT" << "_id" << id_;
    ros::NodeHandle nh(ss_ros_namespace.str());
    ros_node_ = nh;
    ROS_INFO("%s", ss_ros_namespace.str().c_str());

    //Creating control publisher
    ros::Publisher* pub0 = new ros::Publisher();
    //Starting Advertising
    *pub0 = ros_node_.advertise<control_msgs::FollowJointTrajectoryAction>("/scaled_pos_joint_traj_controller/follow_joint_trajectory", 1); //TODO: Welches Topic welche NAchricht?
    //Adding publisher to array
    ros_publishers_.push_back(pub0);

    //Creating subscriber
    ros::Subscriber* sub0 = new ros::Subscriber();
    //Starting subscription
    *sub0 = ros_node_.subscribe("/scaled_pos_joint_traj_controller/state", 1, &Ur5e_IKT::subStateCallback, this);//TODO: Welches Topic welche NAchricht?
    //Adding subscriber to array
    ros_state_subscribers_.push_back(sub0);
    //Creating subscriber
    ros::Subscriber* sub1 = new ros::Subscriber();
    //Starting subscription
    *sub1 = ros_node_.subscribe("/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal", 1, &Ur5e_IKT::subDesiredStateCallback, this);//TODO: Welches Topic welche NAchricht?
    //Adding subscriber to array
    ros_desired_state_subscribers_.push_back(sub1);
}
Ur5e_IKT::Ur5e_IKT(
    std::string state_subscriber_topic,
    std::string desired_state_subscriber_topic,
    std::string control_publish_topic,
    double* init_x = NULL,
    double* init_xdes = NULL,
    double* init_u = NULL,
    double* init_udes = NULL,
    double* init_p = NULL,
    double* init_d = NULL,
    int id = 0
): Ur5e_IKT(id) {
    if (init_x != NULL) {this->setInitialState (init_x);};
    if (init_xdes != NULL) {this->setInitialDesiredState (init_xdes);};
    if (init_u != NULL) {this->setInitialControl (init_u);};
    if (init_udes != NULL) {this->setInitialDesiredControl(init_udes);};
    if (init_p != NULL) {this->setInitialParameter (init_p);};
    this->reset2initialstate();
    this->setStateSubscriberRosTopicName(state_subscriber_topic);
    this->setDesiredStateSubscriberRosTopicName(desired_state_subscriber_topic);
    this->setPublisherRosTopicName(control_publish_topic);
};

/********************************************************
 * System dynamics                                      *
 ********************************************************/
void Ur5e_IKT::f(double  *out, double t, double *x, double *u, double *d, double *p) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::f(...)" << endl;
#endif
    out[0] = u[0] * cos(x[2]);
    out[1] = u[0] * sin(x[2]);
    out[2] = u[1];
}
void Ur5e_IKT::dfdxlambda(double  *out, double t, double *x, double *u, double *d, double *p, double *lambda) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dfdx(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = lambda[1] * u[0] * cos(x[2]) - lambda[0] * u[0] * sin(x[2]);
}
void Ur5e_IKT::dfdulambda(double  *out, double t, double *x, double *u, double *d, double *p, double *lambda) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dfdu(...)" << endl;
#endif
    out[0] = lambda[0] * cos(x[2]) + lambda[1] * sin(x[2]);
    out[1] = lambda[2];
}

/********************************************************
 * Stage costs                                          *
 ********************************************************/
void Ur5e_IKT::l(double  *out, double t, double *x, double *u, double *p, double *xdes, double *udes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::l(...)" << endl;
#endif
    out[0] = p[3] * pow(-u[0] + udes[0], 2.) + p[4] * pow(-u[1] + udes[1], 2.) + p[0] * pow(-x[0] + xdes[0], 2.) + p[1] * pow(-x[1] + xdes[1], 2.) + p[2] * pow(-x[2] + xdes[2], 2.);
}
void Ur5e_IKT::dldx(double  *out, double t, double *x, double *u, double *p, double *xdes, double *udes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dldx(...)" << endl;
#endif
    out[0] = -2.*p[0] * (-x[0] + xdes[0]);
    out[1] = -2.*p[1] * (-x[1] + xdes[1]);
    out[2] = -2.*p[2] * (-x[2] + xdes[2]);
}
void Ur5e_IKT::dldu(double  *out, double t, double *x, double *u, double *p, double *xdes, double *udes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dldu(...)" << endl;
#endif
    out[0] = -2.*p[3] * (-u[0] + udes[0]);
    out[1] = -2.*p[4] * (-u[1] + udes[1]);
}

/********************************************************
 * Final costs                                          *
 ********************************************************/
void Ur5e_IKT::v(double  *out, double t, double *x, double *p, double *xdes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::v(...)" << endl;
#endif
    out[0] = p[0] * pow(-x[0] + xdes[0], 2.) + p[1] * pow(-x[1] + xdes[1], 2.) + p[2] * pow(-x[2] + xdes[2], 2.);
}
void Ur5e_IKT::dvdx(double  *out, double t, double *x, double *p, double *xdes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dvdx(...)" << endl;
#endif
    out[0] = -2.*p[0] * (-x[0] + xdes[0]);
    out[1] = -2.*p[1] * (-x[1] + xdes[1]);
    out[2] = -2.*p[2] * (-x[2] + xdes[2]);
}

/********************************************************
 * Inequality constraints                               *
 ********************************************************/
void Ur5e_IKT::ci(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::ci(...)" << endl;
#endif
    out[0] = -0.5 + 0.5 * pow(u[0], 2.);
    out[1] = -0.5 + 0.5 * pow(u[1], 2.);
}
void Ur5e_IKT::dcidxmui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dcidxmui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
}
void Ur5e_IKT::dcidumui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dcidumui(...)" << endl;
#endif
    out[0] = mui[0] * u[0];
    out[1] = mui[1] * u[1];
}
void Ur5e_IKT::cia(double  *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *slack) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::cia(...)" << endl;
#endif
    out[0] = -0.5 + pow(slack[0], 2.) + 0.5 * pow(u[0], 2.);
    out[1] = -0.5 + pow(slack[1], 2.) + 0.5 * pow(u[1], 2.);
}
void Ur5e_IKT::dciadxmui(double  *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dciadxmui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
}
void Ur5e_IKT::dciadumui(double  *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dciadumui(...)" << endl;
#endif
    out[0] = mui[0] * u[0];
    out[1] = mui[1] * u[1];
}
void Ur5e_IKT::dciadamui(double  *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e_IKT::dciadamui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
}
