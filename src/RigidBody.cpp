#include <eigen3/Eigen/Dense>

#include "RigidBody.h"
#include "utils.h"

using namespace Eigen;
namespace plt = matplotlibcpp;


struct{
	Vector3d P_g; //position vector (global frame)
	Vector3d V_g; //velocity vector (global frame)
	Quaternion<double> q; //quaternion representing rotation from global frame to body frame
	Vector3d H_b; //angular momentum vector (Body frame)
}state;


struct{
	Vector3d V_g; //velocity vector (global frame)
	Vector3d Anet_g; //Net acceleration vector (global frame)
	Quaternion<double> q_dot; //TODO
	Vector3d Tnet_b; //Net torque vector (body frame)
}state_dot;


/*
	Body Frame's origin is at the center of mass
*/

RigidBody::RigidBody(double mass, Matrix3d InertiaTensor, double timestep, Quaternion<double> rotation_initial, Vector3d w_b_initial, Vector3d P_g_initial, Vector3d V_g_initial) {
	g << 0, 0, -9.81; //gravity
	
	m = mass; //RigidBody's total mass
	
	this->InertiaTensor = InertiaTensor; //InertiaTensor along body frame axes
	InertiaTensorInverse = 	InertiaTensor.inverse();
	
	dt = timestep;

	//setting the components of the global linear position
	x(0) = P_g_initial(0);
	x(1) = P_g_initial(1);
	x(2) = P_g_initial(2);

	//setting the components of the global linear velocity
	x(3) = V_g_initial(0);
	x(4) = V_g_initial(1);
	x(5) = V_g_initial(2);

	//setting the components of the rotation quaternion
	x(6) = rotation_initial.w();
	x(7) = rotation_initial.x();
	x(8) = rotation_initial.y();
	x(9) = rotation_initial.z();

	//setting the components of the body angular momentum
	Vector3d H_b = InertiaTensor*w_b_initial;
	x(10) = H_b(0);
	x(11) = H_b(1);
	x(12) = H_b(2);

	u << 0,0,0,0,0,0;

/*
	q = rotation_initial;

	w_b = w_b_initial;
	w_g = q.inverse()*w_b_initial;
	H_g = q.inverse()*H_b;

	V_g = V_g_initial;
	P_g = P_g_initial;

	Fnet_b << 0, 0, 0;
	Tnet_b << 0, 0, 0;
*/

}

Quaternion<double> RigidBody::getRotationGtoB(){
	return Quaternion<double>(x(6), x(7), x(8), x(9));

	//return q;
}

Quaternion<double> RigidBody::getRotationBtoG(){
	return (Quaternion<double>(x(6), x(7), x(8), x(9))).inverse();
	//return q.inverse();
}


void RigidBody::applyForce(Vector3d force, Vector3d pointOfApplication) {
	u(0) += force(0);
	u(1) += force(1);
	u(2) += force(2);

	Vector3d moment = pointOfApplication.cross(force);
	u(3) += moment(0);
	u(4) += moment(1);
	u(5) += moment(2);

	/*
	Fnet_b += force;
	Tnet_b += pointOfApplication.cross(force);
	*/
}

void RigidBody::applyMoment(Vector3d moment) {
	u(3) += moment(0);
	u(4) += moment(1);
	u(5) += moment(2);

	//Tnet_b += moment;
}

void RigidBody::clearAppliedForcesAndMoments() {
	u << 0,0,0,0,0,0;

	/*
	Fnet_b << 0, 0, 0;
	Tnet_b << 0, 0, 0;
	*/
}



Vector13d RigidBody::f(Vector13d x, Vector6d u){
	Vector13d xdot;

	//setting the components of the global linear velocity
	xdot(0) = x(3);
	xdot(1) = x(4);
	xdot(2) = x(5);

	//setting the components of the global linear acceleration
	Vector3d Fnet_b; Fnet_b << u(0), u(1), u(2);
	Vector3d Anet_b = Fnet_b / m;
	Quaternion<double> q = Quaternion<double>(x(6), x(7), x(8), x(9));
	Vector3d Anet_g = q.inverse() * Anet_b + g;
	xdot(3) = Anet_g(0);
	xdot(4) = Anet_g(1);
	xdot(5) = Anet_g(2);


	//setting the components of the derivative of the rotation quaternion
	Vector3d H_b; H_b << x(10), x(11), x(12);
	Vector3d w_b = InertiaTensorInverse*H_b;
	Quaternion<double> w_b_quaternion = Quaternion<double>(0, w_b(0), w_b(1), w_b(2));
	Quaternion<double> qdot = Quaternion<double>(0.5 * (q*w_b_quaternion).coeffs()); //TODO: might have to switch the order of w_B and q (pg.7 https://arxiv.org/pdf/1708.08680.pdf)
	
	xdot(6) = qdot.w();
	xdot(7) = qdot.x();
	xdot(8) = qdot.y();
	xdot(9) = qdot.z();
	

	//setting the components of the body toruque
	Vector3d Tnet_b; Tnet_b << u(3), u(4), u(5);
	xdot(10) = Tnet_b(0);
	xdot(11) = Tnet_b(1);
	xdot(12) = Tnet_b(2);

	return xdot;	
}

VectorXd RigidBody::rk4(VectorXd x, VectorXd u, double dt){
	double half_dt = dt * 0.5;
	VectorXd k1 = f(x, u);
	VectorXd k2 = f(x + half_dt * k1, u);
	VectorXd k3 = f(x + half_dt * k2, u);
	VectorXd k4 = f(x + dt * k3, u);
	return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void RigidBody::update(double currTime) {

	x = rk4(x, u, dt);
	Quaternion<double> q = Quaternion<double>(x(6), x(7), x(8), x(9));
	Vector3d H_b; H_b << x(10), x(11), x(12);
	Vector3d w_b = InertiaTensorInverse*H_b;



	Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
	Matrix3d R = q.toRotationMatrix();
	std::cout<<R(2,1)<<"\n";
	//std::cout<<atan2(R(2,1), R(1,1))<<"\n";
	//std::cout<<q.toRotationMatrix()<<"\n\n";
	thetaX.push_back(euler(0));
	thetaY.push_back(euler(1));
	thetaZ.push_back(euler(2));

	w_bX.push_back(w_b(0));
	w_bY.push_back(w_b(1));
	w_bZ.push_back(w_b(2));

	Vector3d w_g = q.inverse()*w_b;
	w_gX.push_back(w_g(0));
	w_gY.push_back(w_g(1));
	w_gZ.push_back(w_g(2));

	H_bX.push_back(H_b(0));
	H_bY.push_back(H_b(1));
	H_bZ.push_back(H_b(2));

	Vector3d H_g = q.inverse()*H_b;
	//std::cout<<H_g<<"\n\n";
	H_gX.push_back(H_g(0));
	H_gY.push_back(H_g(1));
	H_gZ.push_back(H_g(2));

	// accelX_global_arr.push_back(Anet_g(0));
	// accelY_global_arr.push_back(Anet_g(1));
	// accelZ_global_arr.push_back(Anet_g(2));

	Vector3d V_g; V_g << x(3), x(4), x(5);
	velX_global_arr.push_back(V_g(0));
	velY_global_arr.push_back(V_g(1));
	velZ_global_arr.push_back(V_g(2));

	Vector3d P_g; P_g << x(0), x(1), x(2);
	posX_global_arr.push_back(P_g(0));
	posY_global_arr.push_back(P_g(1));
	posZ_global_arr.push_back(P_g(2));



	/*Anet_b = Fnet_b / m;
	Anet_g = q.inverse() * Anet_b + g;
	V_g += Anet_g * dt;
	P_g += V_g * dt;

	//angular dynamics update
	Alpha_net_b = InertiaTensorInverse * (Tnet_b - w_b.cross(InertiaTensor * w_b)); //net angular acceleration in the body frame

	w_b += Alpha_net_b * dt; //angular velocity in the body frame
	double w_norm = w_b.norm();
	double realPart = cos(w_norm * dt / 2.0);
	Vector3d imagPart = (w_b / w_norm) *  sin(w_norm * dt / 2);

	Quaternion<double> rotation_from_last_timestep = Quaternion<double>(realPart, imagPart(0), imagPart(1), imagPart(2));
	q = q * rotation_from_last_timestep;



	Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
//	std::cout<<euler<<"\n";
	thetaX.push_back(deg(euler(0)));
	thetaY.push_back(deg(euler(1)));
	thetaZ.push_back(deg(euler(2)));

	w_bX.push_back(deg(w_b(0)));
	w_bY.push_back(deg(w_b(1)));
	w_bZ.push_back(deg(w_b(2)));

	w_g = q.inverse()*w_b;
	w_gX.push_back(deg(w_g(0)));
	w_gY.push_back(deg(w_g(1)));
	w_gZ.push_back(deg(w_g(2)));

	H_b += Tnet_b*dt; 	//H_b = InertiaTensor*w_b;
	H_bX.push_back(H_b(0));
	H_bY.push_back(H_b(1));
	H_bZ.push_back(H_b(2));

	H_g = q.inverse()*H_b;
	//std::cout<<H_g<<"\n\n";
	H_gX.push_back(H_g(0));
	H_gY.push_back(H_g(1));
	H_gZ.push_back(H_g(2));

	accelX_global_arr.push_back(Anet_g(0));
	accelY_global_arr.push_back(Anet_g(1));
	accelZ_global_arr.push_back(Anet_g(2));

	velX_global_arr.push_back(V_g(0));
	velY_global_arr.push_back(V_g(1));
	velZ_global_arr.push_back(V_g(2));

	posX_global_arr.push_back(P_g(0));
	posY_global_arr.push_back(P_g(1));
	posZ_global_arr.push_back(P_g(2));
*/
	time.push_back(currTime);
}

void RigidBody::showPlots() {
	plt::subplot(1, 3, 1);
	plt::plot(time, thetaX, "thetaX");
	plt::legend();
	plt::subplot(1, 3, 2);
	plt::plot(time, thetaY, "thetaY");
	plt::legend();
	plt::subplot(1, 3, 3);
	plt::plot(time, thetaZ, "thetaZ");
	plt::legend();
	plt::show();

	plt::subplot(4, 3, 1);
	plt::plot(time, w_bX, "w_bX");
	plt::legend();
	plt::subplot(4, 3, 2);
	plt::plot(time, w_bY, "w_bY");
	plt::legend();
	plt::subplot(4, 3, 3);
	plt::plot(time, w_bZ, "w_bZ");
	plt::legend();
	plt::subplot(4, 3, 4);
	plt::plot(time, w_gX, "w_gX");
	plt::legend();
	plt::subplot(4, 3, 5);
	plt::plot(time, w_gY, "w_gY");
	plt::legend();
	plt::subplot(4, 3, 6);
	plt::plot(time, w_gZ, "w_gZ");
	plt::legend();

	plt::subplot(4, 3, 7);
	plt::plot(time, H_bX, "H_bX");
	plt::legend();
	plt::subplot(4, 3, 8);
	plt::plot(time, H_bY, "H_bY");
	plt::legend();
	plt::subplot(4, 3, 9);
	plt::plot(time, H_bZ, "H_bZ");
	plt::legend();
	plt::subplot(4, 3, 10);
	plt::plot(time, H_gX, "H_gX");
	plt::legend();
	plt::subplot(4, 3, 11);
	plt::plot(time, H_gY, "H_gY");
	plt::legend();
	plt::subplot(4, 3, 12);
	plt::plot(time, H_gZ, "H_gZ");
	plt::legend();
	plt::show();
/*


	plt::subplot(3, 3, 1);
	plt::plot(time, accelX_global_arr, "accX_global_arr");
	plt::legend();
	plt::subplot(3, 3, 2);
	plt::plot(time, accelY_global_arr, "accY_global_arr");
	plt::legend();
	plt::subplot(3, 3, 3);
	plt::plot(time, accelZ_global_arr, "accZ_global_arr");
	plt::legend();
	*/

	plt::subplot(3, 3, 4);
	plt::plot(time, velX_global_arr, "velX_global_arr");
	plt::legend();
	plt::subplot(3, 3, 5);
	plt::plot(time, velY_global_arr, "velY_global_arr");
	plt::legend();
	plt::subplot(3, 3, 6);
	plt::plot(time, velZ_global_arr, "velZ_global_arr");
	plt::legend();

	plt::subplot(3, 3, 7);
	plt::plot(time, posX_global_arr, "posX_global_arr");
	plt::legend();
	plt::subplot(3, 3, 8);
	plt::plot(time, posY_global_arr, "posY_global_arr");
	plt::legend();
	plt::subplot(3, 3, 9);
	plt::plot(time, posZ_global_arr, "posZ_global_arr");
	plt::legend();
	plt::show();
}