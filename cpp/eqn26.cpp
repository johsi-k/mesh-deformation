#include "..\headers\eqn26.h"
#include <Eigen\Eigen>

using namespace Eigen;

Matrix3d quaternion2rotationMatrix(Vector4d quaternion) {

	const double x = quaternion[0];
	const double y = quaternion[1];
	const double z = quaternion[2];
	const double w = quaternion[3];

	Matrix3d rotationMatrix;
	rotationMatrix <<
		1 - 2*y*y - 2*z*z, 
		2*x*y + 2*w*z, 
		2*x*z - 2*w*y,

		2*x*y - 2*w*z, 
		1 - 2*x*x - 2*z*z, 
		2*y*z + 2*w*x,

		2*x*z + 2*w*y, 
		2*y*z - 2*w*x,
		1 - 2*x*x - 2*y*y;

	return rotationMatrix;

}