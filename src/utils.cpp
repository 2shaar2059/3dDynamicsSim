#include "utils.h"
#include <math.h>


double rad(double degrees) {
	return M_PI * degrees / 180.0;
}

double deg(double radians) {
	return radians * 180.0 / M_PI;
}