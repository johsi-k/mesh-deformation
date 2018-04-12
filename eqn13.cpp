#include <Eigen/Eigen>

using namespace Eigen;

// TODO: find out k1, k2, a1, a2
// eqn 13
double get_h_field(double t) {
	
	const double k1 = 0;
	const double k2 = 0;
	const double a1 = 0;
	const double a2 = 0;

	if (k1 == 0 && k2 == 0) return t;
	else if (k1 != 0 && k2 == 0) return (t*t/2 + 2*a1*t) / (a1 + t);
	else if (k1 == 0 && k2 != 0) return (t*t/2 + 2*a2*t) / (a2 + t);
	return (t*t*t/3 + (a1 + a2)*t*t/2 + a1*a2*t) / (a1 + t) / (a2 + t);
}