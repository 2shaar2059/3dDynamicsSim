/*Helpful links:

	https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html
	https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html
	https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html
	https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
	https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

	https://cseweb.ucsd.edu/classes/sp16/cse169-a/slides/CSE169_17.pdf
	https://phys.libretexts.org/Bookshelves/University_Physics/Book%3A_Mechanics_and_Relativity_(Idema)/07%3A_General_Rotational_Motion/7.03%3A_Rotations_About_an_Arbitrary_Axis
	https://physics.stackexchange.com/questions/268698/compute-an-objects-inertia-around-an-arbitrary-axis-using-its-known-values-for
	https://www.cs.umd.edu/~mount/Indep/Jeremy_Ulrich/final-rept.pdf
	https://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf
*/

#include <iostream>
#include <eigen3/Eigen/Geometry>


#define print(x) std::cout<<(#x)<<":\n"<<(x)<<"\n\n";

using namespace Eigen;


double rad(double degrees) {
	return M_PI * degrees / 180.0;
}

int main() {
	Vector3d rotationAxisUnnormalized; rotationAxisUnnormalized << 1.0, 1.0, 1.0; //the vector about which other vectors are getting rotated
	Vector3d rotationAxisNormalized = rotationAxisUnnormalized.normalized();
	double rotationAngle = rad(120.0);

	Quaternion<double> q;  q = AngleAxis<double>(rotationAngle, rotationAxisNormalized);
	Vector3d v0; v0 << 1.0, 0.0, 0.0; //vector to be rotated
	Vector3d v1 = q * v0; //resulting (rotated) vector

	print(rotationAxisUnnormalized);
	print(rotationAxisNormalized);
	print(v0);
	print(v1);

	return 0;
}