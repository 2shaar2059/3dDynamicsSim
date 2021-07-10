#include <iostream>
#include <math.h>
#include <chrono>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <Python.h>

#include "RigidBody.h"
#include "utils.h"

using namespace Eigen;

void runPythonScript(const char* filename){
	Py_Initialize();

	FILE* fp = _Py_fopen(filename, "r");
	PyRun_SimpleFile(fp, filename);

	Py_Finalize();
}

int main()
{
	auto start = std::chrono::steady_clock::now();

	double maxTime = 10;
	double currTime = 0;
	double dt = 0.001; //timestep between state updates

	Quaternion<double> rotation_initial = AngleAxis<double>(rad(0.0), Vector3d::UnitX())
	                                      * AngleAxis<double>(rad(0.0),  Vector3d::UnitY())
	                                      * AngleAxis<double>(rad(0.0), Vector3d::UnitZ());

	Vector3d ang_vel_inital;	ang_vel_inital << 0,	1000,		0; //thetaXdot, thetaYdot, thetaZdot 	[rad/s, rad/s, rad/s]
	Vector3d lin_pos_inital;	lin_pos_inital << 0,	0,		0; //posX, psoY, posZ					[m, m, m]
	Vector3d lin_vel_inital;	lin_vel_inital << 1,	0,		0; //velX, velY, velZ					[m/s, m/s, m/s]

	Matrix3d InertiaTensor; InertiaTensor << 90e-6, 0, 0,
									         0, 45e-6, 0,
									         0, 0, 45e-6;

	RigidBody rigidBody1 = RigidBody(1, InertiaTensor, rotation_initial, ang_vel_inital, lin_pos_inital, lin_vel_inital);

	while (currTime <= maxTime) {
		rigidBody1.clearAppliedForcesAndMoments();
		rigidBody1.applyForce(rigidBody1.getRotationGtoB()*Vector3d(0, 0, -0.2), Vector3d(0, .03, 0));
		rigidBody1.applyForce(rigidBody1.getRotationGtoB()*Vector3d(0, 0, 0.2), Vector3d(0, 0, 0));
		
		//rigidBody1.applyMoment(Vector3d(1, 0, 0));
		
		rigidBody1.applyForce(rigidBody1.getRotationGtoB()*Vector3d(0, 0, 9.81), Vector3d(0, 0, 0));
		rigidBody1.update(dt);
		currTime += dt;
	}

	auto end = std::chrono::steady_clock::now();
	std::cout << "Elapsed time to Simulate: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";
	rigidBody1.showPlots();
	rigidBody1.logDataToFile();

	runPythonScript("animate.py");
	return 0;
}