/*Helpful links:
	"Computing Euler Angles: Tracking Attitude Using Quaternions" timestamp=59:40 https://www.youtube.com/watch?v=98Mfe-Vfgo0&list=PLxdnSsBqCrrEx3A6W94sQGClk6Q4YCg-h&index=9
	https://www.researchgate.net/profile/Guoping-He/publication/344138714_Notes_on_Extended_Kalman_Filter_based_Visual-Inertial_Navigation_Systems/links/5f551cc092851c250b990b74/Notes-on-Extended-Kalman-Filter-based-Visual-Inertial-Navigation-Systems.pdf (pg.36, eq.240)
	https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html
*/

#include <iostream>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>

#include "utils.h"

#define print(x) cout<<(#x)<<":\n"<<(x)<<"\n\n";

using namespace Eigen;
using namespace std;


int main() {
	double dt = 0.005; //timestep (seconds)

	//initial rotation matrix from initial global frame to initial body frame
	Matrix3d R; R << 1, 0, 0,
	         0, 1, 0,
	         0, 0, 1;
	Quaternion<double> q = Quaternion<double>(R);

	ifstream myfile; // stream object used to open test data file
	myfile.open("gyro.csv", ifstream::in);
	if (!myfile) {
		cout << "file messed up" << endl;
	}
	else {
		string rowString; // read each line of the file into rowString
		while (myfile >> rowString) {
			stringstream s_stream(rowString); //create string stream from the string
			double row[3];
			Vector3d w;
			for (int i = 0; i < 3; i++) {
				string cell;
				getline(s_stream, cell, ','); //get first string delimited by comma
				row[i] = rad(stod(cell));
			}
			w << row[0], row[1], row[2];
			double w_norm = w.norm();
			double realPart = cos(w_norm * dt / 2.0);
			Vector3d imagPart = (w / w_norm) *  sin(w_norm * dt / 2);
			q = q * Quaternion<double>(realPart, imagPart(0), imagPart(1), imagPart(2));
		}
	}

	myfile.close();

	print(R * q);


	return 0;
}