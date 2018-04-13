#include "..\headers\eqn25.h"

// eqn 25
Vector4f orthoParamsToQuarternion(Vector3f orthoParams) {
	const float theta1 = orthoParams[0] / 2;
	const float theta2 = orthoParams[1];
	const float theta3 = orthoParams[2];

	const float x = sin(theta1) * sin(theta2) * sin(theta3);
	const float y = sin(theta1) * sin(theta2) * cos(theta3);
	const float z = sin(theta1) * cos(theta2);
	const float w = cos(theta1);

	return Vector4f(x, y, z, w);
}

// eqn 27
// TODO: Find out what n_abs_sqr is
Vector4f conformalParamsToQuaternion(Vector3f conformalParams) {

	const float n1 = conformalParams[0];
	const float n2 = conformalParams[1];
	const float n3 = conformalParams[2];

	const float n_abs_sqr = 0;
	const float denom = 1 / (n_abs_sqr + 4);

	const float x = 4 * n1 * denom;
	const float y = 4 * n2 * denom;
	const float z = 4 * n3 * denom;
	const float w = (4 - n_abs_sqr) * denom;

	return Vector4f(x, y, z, w);
}