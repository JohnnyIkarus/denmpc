#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <chrono>
#include <thread>
#include "stateSpace/SimulateSystem.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <tuple>
#include <fstream>
#include <chrono> //used to time the code

using namespace ur_rtde;
using namespace std::chrono;
using namespace Eigen;
using namespace std;

int main(int argc, char *argv[])
{
    string path_to_csv = "csv/";
    SimulateSystem Simulation3;
	Simulation3.openFromFile(path_to_csv + "AFile.csv", path_to_csv + "BFile.csv", path_to_csv + "CFile.csv",path_to_csv + "DFile.csv", path_to_csv +"x0File.csv",path_to_csv + "inputFile.csv");
	
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	Simulation3.runSimulation();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[micro seconds]" << std::endl;
	Simulation3.saveData(path_to_csv +"AFileOutput.csv", path_to_csv +"BFileOutput.csv",path_to_csv + "CFileOutput.csv", path_to_csv +"DFileOutput.csv",path_to_csv +"x0FileOutput.csv",path_to_csv + "inputSequenceFileOutput.csv", path_to_csv +"simulatedStateSequenceFileOutput.csv", path_to_csv +"simulatedOutputSequenceFileOutput.csv");
    return 0;
}
