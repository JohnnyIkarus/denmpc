#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <chrono>
#include <thread>

using namespace ur_rtde;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    RTDEControlInterface rtde_control("127.0.0.1");
    RTDEReceiveInterface rtde_receive("127.0.0.1");
    std::vector<double> init_q = rtde_receive.getActualQ();

    int A[2][2] = {{0, 0},
                   {1, 0}};
    int B[2][1] = {{1}, {0}};
    int C[2] = {0, 1};
    int D = 0;
    float Ts = 0.002; //sample Time
    float Tstop = 20;
    float steps = Tstop / Ts;
    int n = 5;

    int max_vel = 3;
    float startpos[6] = {-1.57, -1.57, 1.57, 0, 0, 0};
    std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
    std::vector<double> joint_speed = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    rtde_control.moveJ(joint_q);
    /*
    MATLAB CODE:

    cplant = ss(A,B,C,D);
    cplant.InputDelay = 0.004;
    cplant.OutputDelay = 0.004;

    opt = c2dOptions('Method','tustin','FractDelayApproxOrder',3); 
    dplant = c2d(cplant,Ts,opt); %discrete
    */
    // Wie in c++ state space model definieren? und wie diskretisieren?

    //Bewegen des Roboters 
    for(unsigned int i = 0; i <= steps; i++){
        rtde_control.speedJ(joint_speed, 0.5, 0.0001);
    }

    rtde_control.speedStop();
    rtde_control.stopScript();

    return 0;
}
