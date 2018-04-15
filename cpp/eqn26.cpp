#include "..\headers\eqn26.h"
#include <Eigen\Geometry>

Matrix3f quaternion2rotationMatrix(Vector4f quaternion) {

	const float x = quaternion[0];
	const float y = quaternion[1];
	const float z = quaternion[2];
	const float w = quaternion[3];

	Matrix3f rotationMatrix;
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