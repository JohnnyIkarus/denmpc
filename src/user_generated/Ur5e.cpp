
/**
 * @file    Ur5e.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    Friday 24 November 2017
 * @time    11:07:32
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 *          Description of agent
 */

#include <Ur5e.h>

/*******************************************************
 * Constructor                                         *
 *******************************************************/
Ur5e::Ur5e(int id): Agent(id) {
    //Set initial values
    dim_x_ = 3;
    dim_xdes_ = 3;
    dim_u_ = 2;
    dim_udes_ = 2;
    dim_y_ = 0;
    dim_ydes_ = 0;
    dim_p_ = 5;
    dim_d_ = 0;
    dim_l_ = 1;
    dim_v_ = 1;
    dim_eq_ = 0;
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
    ss_ros_namespace << "Ur5e" << "_id" << id_;
    ros::NodeHandle nh(ss_ros_namespace.str());
    ros_node_ = nh;
    ROS_INFO("%s", ss_ros_namespace.str().c_str());

    //Creating control publisher
    ros::Publisher* pub0 = new ros::Publisher();
    //Starting Advertising
    *pub0 = ros_node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //Adding publisher to array
    ros_publishers_.push_back(pub0);

    //Creating subscriber
    ros::Subscriber* sub0 = new ros::Subscriber();
    //Starting subscription
    *sub0 = ros_node_.subscribe("pose", 1, &Ur5e::subStateCallback, this);
    //Adding subscriber to array
    ros_state_subscribers_.push_back(sub0);
    //Creating subscriber
    ros::Subscriber* sub1 = new ros::Subscriber();
    //Starting subscription
    *sub1 = ros_node_.subscribe("desiredpose", 1, &Ur5e::subDesiredStateCallback, this);
    //Adding subscriber to array
    ros_desired_state_subscribers_.push_back(sub1);
}
Ur5e::Ur5e(
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
): Ur5e(id) {
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
void Ur5e::f(double  *out, double t, double *x, double *u, double *d, double *p) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::f(...)" << endl;
#endif
    out[0] = u[0] * cos(x[2]);
    out[1] = u[0] * sin(x[2]);
    out[2] = u[1];
}
void Ur5e::dfdxlambda(double  *out, double t, double *x, double *u, double *d, double *p, double *lambda) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dfdx(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = lambda[1] * u[0] * cos(x[2]) - lambda[0] * u[0] * sin(x[2]);
}
void Ur5e::dfdulambda(double  *out, double t, double *x, double *u, double *d, double *p, double *lambda) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dfdu(...)" << endl;
#endif
    out[0] = lambda[0] * cos(x[2]) + lambda[1] * sin(x[2]);
    out[1] = lambda[2];
}

/********************************************************
 * Stage costs                                          *
 ********************************************************/
void Ur5e::l(double  *out, double t, double *x, double *u, double *p, double *xdes, double *udes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::l(...)" << endl;
#endif
    out[0] = p[3] * pow(-u[0] + udes[0], 2.) + p[4] * pow(-u[1] + udes[1], 2.) + p[0] * pow(-x[0] + xdes[0], 2.) + p[1] * pow(-x[1] + xdes[1], 2.) + p[2] * pow(-x[2] + xdes[2], 2.);
}
void Ur5e::dldx(double  *out, double t, double *x, double *u, double *p, double *xdes, double *udes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dldx(...)" << endl;
#endif
    out[0] = -2.*p[0] * (-x[0] + xdes[0]);
    out[1] = -2.*p[1] * (-x[1] + xdes[1]);
    out[2] = -2.*p[2] * (-x[2] + xdes[2]);
}
void Ur5e::dldu(double  *out, double t, double *x, double *u, double *p, double *xdes, double *udes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dldu(...)" << endl;
#endif
    out[0] = -2.*p[3] * (-u[0] + udes[0]);
    out[1] = -2.*p[4] * (-u[1] + udes[1]);
}

/********************************************************
 * Final costs                                          *
 ********************************************************/
void Ur5e::v(double  *out, double t, double *x, double *p, double *xdes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::v(...)" << endl;
#endif
    out[0] = p[0] * pow(-x[0] + xdes[0], 2.) + p[1] * pow(-x[1] + xdes[1], 2.) + p[2] * pow(-x[2] + xdes[2], 2.);
}
void Ur5e::dvdx(double  *out, double t, double *x, double *p, double *xdes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dvdx(...)" << endl;
#endif
    out[0] = -2.*p[0] * (-x[0] + xdes[0]);
    out[1] = -2.*p[1] * (-x[1] + xdes[1]);
    out[2] = -2.*p[2] * (-x[2] + xdes[2]);
}

/********************************************************
 * Inequality constraints                               *
 ********************************************************/
void Ur5e::ci(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::ci(...)" << endl;
#endif
    out[0] = -0.5 + 0.5 * pow(u[0], 2.);
    out[1] = -0.5 + 0.5 * pow(u[1], 2.);
}
void Ur5e::dcidxmui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dcidxmui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
}
void Ur5e::dcidumui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dcidumui(...)" << endl;
#endif
    out[0] = mui[0] * u[0];
    out[1] = mui[1] * u[1];
}
void Ur5e::cia(double  *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *slack) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::cia(...)" << endl;
#endif
    out[0] = -0.5 + pow(slack[0], 2.) + 0.5 * pow(u[0], 2.);
    out[1] = -0.5 + pow(slack[1], 2.) + 0.5 * pow(u[1], 2.);
}
void Ur5e::dciadxmui(double  *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dciadxmui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
}
void Ur5e::dciadumui(double  *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dciadumui(...)" << endl;
#endif
    out[0] = mui[0] * u[0];
    out[1] = mui[1] * u[1];
}
void Ur5e::dciadamui(double  *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack) {
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Ur5e::dciadamui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
}
