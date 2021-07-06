#include <eigen3/Eigen/Dense>
#include "../../matplotlib-cpp-master/matplotlibcpp.h"


using namespace Eigen;


struct state{
	Vector3d P_g; //position vector (global frame)
	Vector3d V_g; //velocity vector (global frame)
	Quaternion<double> q; //quaternion representing rotation from global frame to body frame
	Vector3d H_b; //angular momentum vector (Body frame)
};


struct state_dot{
	Vector3d V_g; //velocity vector (global frame)
	Vector3d Anet_g; //Net acceleration vector (global frame)
	Quaternion<double> q_dot; //TODO
	Vector3d Tnet_b; //Net torque vector (body frame)
};

typedef Matrix<double, 13, 1> Vector13d;
typedef Matrix<double, 6, 1> Vector6d;


class RigidBody {
public:
	/*
		"w_b_initial" is the initial angular velocity expressed in the body coordinates
		"P_g_initial" is the initial linear position expressed in the global coordinates
		"V_g_initial" is the initial linear velocity expressed in the global coordinates
	*/
	RigidBody(double mass, Matrix3d InertiaTensor, Quaternion<double> rotation_initial, Vector3d w_b_initial, Vector3d P_g_initial, Vector3d V_g_initial);

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

	/*
		returns the derivative of the state vector
	*/
	Vector13d f(Vector13d x, Vector6d u);
	
	/*Fourth order Runge-Kutta integration.
		Keyword arguments:
		x -- vector of states
		u -- vector of inputs (constant for dt)
		dt -- time for which to integrate
	*/
	VectorXd rk4(VectorXd x, VectorXd u, double dt);

	/*
		updates the state by
			1. calculating the state dynamics (xdot)
			2. integrating the state dynamics with an ode integrator with a timestep of "dt"
		logs the updated state data to the private vector variables
	*/
	void update(double dt);

	/*
		writes the rotation matrix and linear position private vectors to a file for a python script to use for an animation
	*/
	void logDataToFile();
	
	void showPlots();

private:
	Vector3d g; //gravity: [m/s^2]
	double m; //RigidBody's mass: [kg]
	Matrix3d InertiaTensor; // [kg*m^2]
	Matrix3d InertiaTensorInverse;

	/*
		position vector (global frame)
		velocity vector (global frame)
		quaternion representing rotation from global frame to body frame
		angular momentum vector (Body frame)
	*/
	Vector13d x; //state

	/*
		Net force vector (body frame)
		Net torque vector (body frame)
	*/
	Vector6d u; //inputs (applied forces [N] and torques [Nm])

	double currentTime; //current time into simulation: [s]

	//Vectors for logging data to be plotted
	long numberOfDatapointsLogged; //keeps track of how many simulation timesteps logged data

	std::vector <Matrix3d> orientations; //holds the rotation matrix from the global to body frame at each timestep

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
