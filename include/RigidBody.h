#include <eigen3/Eigen/Dense>
#include "../../matplotlib-cpp-master/matplotlibcpp.h"


using namespace Eigen;

class RigidBody {
public:
	/*
		"w_b_initial" is the initial angular velocity expressed in the body coordinates
		"P_g_initial" is the initial linear position expressed in the global coordinates
		"V_g_initial" is the initial linear velocity expressed in the global coordinates
	*/
	RigidBody(double mass, Matrix3d InertiaTensor, double timestep, Quaternion<double> rotation_initial, Vector3d w_b_initial, Vector3d P_g_initial, Vector3d V_g_initial);

	Quaternion<double> getRotationGtoB(); //return the quaternion representing the rotation from the global to body frame
	
	Quaternion<double> getRotationBtoG(); //return the quaternion representing the rotation from the body to global frame
	
	/*
		"force" is expressed in the body coordinates
		"pointOfApplication" is expressed in the body coordinates
	*/
	void applyForce(Vector3d force, Vector3d pointOfApplication);
	
	/*
		"moment" is expressed in the body coordinates
	*/
	void applyMoment(Vector3d moment);
	
	void clearAppliedForcesAndMoments();
	
	void update(double);
	
	void showPlots();

private:
	Vector3d g; //gravity: [m/s^2]
	double m; //RigidBody's mass: [kg]
	Matrix3d InertiaTensor; // [kg*m^2]
	Matrix3d InertiaTensorInverse;

	Vector3d Fnet_b; //Net force vector (body frame)
	Vector3d Anet_b; //Net acceleration vector (body frame)
	Vector3d Anet_g; //Net acceleration vector (global frame)
	Vector3d V_g; //velocity vector (global frame)
	Vector3d P_g; //position vector (global frame)

	Vector3d Tnet_b; //Net torque vector (body frame)
	Vector3d Alpha_net_b; //Net angular acceleration vector (body frame)
	Vector3d H_b; //angular momentum vector (Body frame)
	Vector3d w_b; //angular velocity vector (Body frame)
	Vector3d H_g; //angular momentum vector (Global frame)
	Vector3d w_g; //angular velocity vector (Global frame)

	Quaternion<double> q; //quaternion representing rotation from global frame to body frame

	double dt; //timestep: [s]

	//Vectors for logging data to be plotted

	std::vector <double> thetaX;
	std::vector <double> thetaY;
	std::vector <double> thetaZ;

	std::vector <double> w_bX;
	std::vector <double> w_bY;
	std::vector <double> w_bZ;

	std::vector <double> w_gX;
	std::vector <double> w_gY;
	std::vector <double> w_gZ;

	std::vector <double> H_bX;
	std::vector <double> H_bY;
	std::vector <double> H_bZ;

	std::vector <double> H_gX;
	std::vector <double> H_gY;
	std::vector <double> H_gZ;

	std::vector <double> posX_global_arr;
	std::vector <double> posY_global_arr;
	std::vector <double> posZ_global_arr;

	std::vector <double> velX_global_arr;
	std::vector <double> velY_global_arr;
	std::vector <double> velZ_global_arr;

	std::vector <double> accelX_global_arr;
	std::vector <double> accelY_global_arr;
	std::vector <double> accelZ_global_arr;

	std::vector <double> signal3;
	std::vector <double> signal5;
	std::vector <double> signal6;
	std::vector <double> signal10;

	std::vector <double> time;
};
