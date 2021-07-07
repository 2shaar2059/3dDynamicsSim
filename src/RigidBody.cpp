#include <eigen3/Eigen/Dense>
#include <fstream>

#include "RigidBody.h"
#include "utils.h"

using namespace Eigen;
namespace plt = matplotlibcpp;

RigidBody::RigidBody(double mass, Matrix3d InertiaTensor, Quaternion<double> rotation_initial, Vector3d w_b_initial, Vector3d P_g_initial, Vector3d V_g_initial) {
	g << 0, 0, -9.81; //gravity

	m = mass; //RigidBody's total mass

	this->InertiaTensor = InertiaTensor; //InertiaTensor along body frame axes
	InertiaTensorInverse = 	InertiaTensor.inverse();

	currentTime = 0;

	numberOfDatapointsLogged = 0;

	x.segment(0,3) = P_g_initial; //using a block operation to embed the 3 components of the global-linear-position-vector into positions 0,1,2 of the overall state vector, x
	
	x.segment(3,3) = V_g_initial; //using a block operation to embed the 3 components of the global-linear-velocity-vector into positions 3,4,5 of the overall state vector, x

	x.segment(6,4) = rotation_initial.coeffs(); //using a block operation to embed the 4 components of the orientation-quaternion into positions 6,7,8,9 of the overall state vector, x

	Vector3d H_b = InertiaTensor * w_b_initial;
	x.segment(10,3) = H_b; //using a block operation to embed the 3 components of the body-angular-momentum-vector into positions 10,11,12 of the overall state vector, x

	u << 0, 0, 0, 0, 0, 0;
}

Quaternion<double> RigidBody::getRotationGtoB() {
	return Quaternion<double>(x(9), x(6), x(7), x(8)); //returning the quaternion that is represented by the 4 quaternion coefficents that are embedded in the state vector, x
}

Quaternion<double> RigidBody::getRotationBtoG() {
	return getRotationGtoB().inverse();
}

void RigidBody::applyMoment(Vector3d moment) {
	u.segment(3,3) += moment; //using a block operation to embed the 3 components of the applied-body-moment-vector into positions 3,4,5 of the overall input vector, u
}

void RigidBody::applyForce(Vector3d force, Vector3d pointOfApplication) {
	u.segment(0,3) += force; //using a block operation to embed the 3 components of the applied-body-force-vector into positions 0,1,2 of the overall input vector, u
	applyMoment(pointOfApplication.cross(force)); //accounting for the applied-body-force's resulting moment
}

void RigidBody::clearAppliedForcesAndMoments() {
	u << 0, 0, 0, 0, 0, 0;
}

Vector13d RigidBody::f(Vector13d x, Vector6d u) {
	Vector13d xdot; //derivative of the state vector

	xdot.segment(0,3) = x.segment(3,3); //using a block operation to embed the components of the global-linear-velocity-vector (derivative of global linear position) into positions 0,1,2 of the overall state vector

	Vector3d Fnet_b; Fnet_b << u(0), u(1), u(2);
	Vector3d Anet_b = Fnet_b / m;
	Quaternion<double> q = getRotationGtoB();
	Vector3d Anet_g = q.inverse() * Anet_b + g;
	xdot.segment(3,3) = Anet_g; //using a block operation to embed the components of the global-linear-acceleration-vector (derivative of global linear velocity) into positions 3,4,5 the overall state vector

	Vector3d H_b; H_b << x(10), x(11), x(12);
	Vector3d w_b = InertiaTensorInverse * H_b;
	Quaternion<double> w_b_quaternion_form = Quaternion<double>(0, w_b(0), w_b(1), w_b(2));
	Quaternion<double> qdot = Quaternion<double>(0.5 * (q * w_b_quaternion_form).coeffs()); //formula 14 from pg.7 of https://arxiv.org/pdf/0811.2889.pdf
	xdot.segment(6,4) = qdot.coeffs(); //using a block operation to embed the components of the derivative-of-the-orientation-quaternion into positions 6,7,8,9 of the overall state vector

	Vector3d Tnet_b; Tnet_b << u(3), u(4), u(5);
	xdot.segment(10,3) = Tnet_b; //using a block operation to embed the components of the body-torque-vector (derivative of body angular momentum) into positions 10,11,12 of the overall state vector

	return xdot;
}

VectorXd RigidBody::rk4(VectorXd x, VectorXd u, double dt) {
	double half_dt = dt * 0.5;
	VectorXd k1 = f(x, u);
	VectorXd k2 = f(x + half_dt * k1, u);
	VectorXd k3 = f(x + half_dt * k2, u);
	VectorXd k4 = f(x + dt * k3, u);
	return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void RigidBody::update(double dt) {
	Quaternion<double> q = getRotationGtoB();
	Matrix3d R = q.toRotationMatrix();
	orientations.push_back(R);

	Vector3d euler = R.eulerAngles(0, 1, 2);
	thetaX.push_back(euler(0));
	thetaY.push_back(euler(1));
	thetaZ.push_back(euler(2));

	Vector3d H_b; H_b << x(10), x(11), x(12);
	Vector3d w_b = InertiaTensorInverse * H_b;
	w_bX.push_back(w_b(0));
	w_bY.push_back(w_b(1));
	w_bZ.push_back(w_b(2));

	Vector3d w_g = q.inverse() * w_b;
	w_gX.push_back(w_g(0));
	w_gY.push_back(w_g(1));
	w_gZ.push_back(w_g(2));

	H_bX.push_back(H_b(0));
	H_bY.push_back(H_b(1));
	H_bZ.push_back(H_b(2));

	Vector3d H_g = q.inverse() * H_b;
	H_gX.push_back(H_g(0));
	H_gY.push_back(H_g(1));
	H_gZ.push_back(H_g(2));

	Vector3d V_g; V_g << x(3), x(4), x(5);
	velX_global_arr.push_back(V_g(0));
	velY_global_arr.push_back(V_g(1));
	velZ_global_arr.push_back(V_g(2));

	Vector3d P_g; P_g << x(0), x(1), x(2);
	posX_global_arr.push_back(P_g(0));
	posY_global_arr.push_back(P_g(1));
	posZ_global_arr.push_back(P_g(2));

	time.push_back(currentTime);

	x = rk4(x, u, dt);

	//numerically integrating the quaternion makes its length "drift" away from having a unit norm. Need to renormalize:
	double q_length = getRotationGtoB().norm();
	x.segment(6, 4)/= q_length; //using a block operation to renormalize the components of state vector, x, that are the 4 coefficents (at indexes 6,7,8,9) of the Rigid Body's orientation-quaternion 

	currentTime += dt;
	numberOfDatapointsLogged++;
}

void RigidBody::logDataToFile() {
	std::ofstream logFile;
	logFile.open("3dAnimationData.txt");

	for(long i=0; i<numberOfDatapointsLogged; i++){
		logFile << time[i]<< "\n";
		logFile << orientations[i]<< "\n";
		logFile << posX_global_arr[i]<< "\n";
		logFile << posY_global_arr[i]<< "\n";
		logFile << posZ_global_arr[i]<< "\n";
		logFile << "----------------------------\n";
	}
	logFile.close();
}

void plotWrapper(std::vector<double> x, std::vector<double> y, char* label, int sublpotRows, int subplotCols, int positionInSubplot){
	plt::subplot(sublpotRows, subplotCols, positionInSubplot);
	plt::plot(x, y, label);
	plt::legend();
	plt::grid(true);
} 

void RigidBody::showPlots() {
	plotWrapper(time, thetaX, "thetaX", 1, 3, 1);
	plotWrapper(time, thetaY, "thetaY", 1, 3, 2);
	plotWrapper(time, thetaZ, "thetaZ", 1, 3, 3);
	plt::show();

	plotWrapper(time, w_bX, "w_bX", 4, 3, 1);
	plotWrapper(time, w_bY, "w_bY", 4, 3, 2);
	plotWrapper(time, w_bZ, "w_bZ", 4, 3, 3);

	plotWrapper(time, w_gX, "w_gX", 4, 3, 4);
	plotWrapper(time, w_gY, "w_gY", 4, 3, 5);
	plotWrapper(time, w_gZ, "w_gZ", 4, 3, 6);

	plotWrapper(time, H_bX, "H_bX", 4, 3, 7);
	plotWrapper(time, H_bY, "H_bY", 4, 3, 8);
	plotWrapper(time, H_bZ, "H_bZ", 4, 3, 9);

	plotWrapper(time, H_gX, "H_gX", 4, 3, 10);
	plotWrapper(time, H_gY, "H_gY", 4, 3, 11);
	plotWrapper(time, H_gZ, "H_gZ", 4, 3, 12);
	plt::show();

	plotWrapper(time, velX_global_arr, "velX_global_arr", 2, 3, 1);
	plotWrapper(time, velY_global_arr, "velY_global_arr", 2, 3, 2);
	plotWrapper(time, velZ_global_arr, "velZ_global_arr", 2, 3, 3);

	plotWrapper(time, posX_global_arr, "posX_global_arr", 2, 3, 4);
	plotWrapper(time, posY_global_arr, "posY_global_arr", 2, 3, 5);
	plotWrapper(time, posZ_global_arr, "posZ_global_arr", 2, 3, 6);
	plt::show();
}