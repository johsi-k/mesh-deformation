#include <Eigen\Eigen>

using namespace Eigen;

// eqn 25
Vector4d orthoParamsToQuarternion(Vector3d orthoParams) {
	const double theta1 = orthoParams[0] / 2;
	const double theta2 = orthoParams[1];
	const double theta3 = orthoParams[2];

	const double x = sin(theta1) * sin(theta2) * sin(theta3);
	const double y = sin(theta1) * sin(theta2) * cos(theta3);
	const double z = sin(theta1) * cos(theta2);
	const double w = cos(theta1);

	return Vector4d(x, y, z, w);
}

// eqn 27
// TODO: Find out what n_abs_sqr is
Vector4d conformalParamsToQuaternion(Vector3d conformalParams) {

	const double n1 = conformalParams[0];
	const double n2 = conformalParams[1];
	const double n3 = conformalParams[2];

	const double n_abs_sqr = 0;
	const double denom = 1 / (n_abs_sqr + 4);

	const double x = 4 * n1 * denom;
	const double y = 4 * n2 * denom;
	const double z = 4 * n3 * denom;
	const double w = (4 - n_abs_sqr) * denom;

	return Vector4d(x, y, z, w);
}