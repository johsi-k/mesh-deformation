#include <Eigen/Geometry>

#include "..\headers\eqn6.h"
#include "..\headers\eqn13.h"
#include "..\headers\eqn14.h"
#include "..\headers\eqn25.h"
#include "..\headers\eqn26.h"


Matrix3f get_scaling_matrix();

Surface_mesh deform_mesh(const Surface_mesh *mesh0) {

	const vector<Vector3f> p;
	const vector<Vector3f> e0_1, e0_2, e0_3;
	const vector<float> A, B, C;

	Vector4f *quat;
	bool is_one_axis = true;

	if (is_one_axis) {
		vector<int> fixed_ids;
		vector<int> handle_ids;
		VectorXf theta_initial;
		float theta_input;
		VectorXf ortho;

		eqn6(*mesh0, fixed_ids, handle_ids, theta_initial, theta_input, ortho);

		quat = &orthoParamsToQuarternion(ortho);
	}
	else {
		Vector3f conformal;
		// eq2(*mesh0, conformal);

		quat = &conformalParamsToQuaternion(conformal);
	}

	const Matrix3f rotationMatrix = quaternion2rotationMatrix(*quat);
	const Matrix3f scaleMatrix = get_scaling_matrix();
	const Matrix3f transformMatrix = rotationMatrix * scaleMatrix;

	vector<Vector3f> e1_1, e1_2, e1_3;
	for (int i = 0; i < e0_1.size(); i++) {
		e1_1.push_back( ( transformMatrix * e0_1[i] ) );
		e1_2.push_back( ( transformMatrix * e0_2[i] ) );
		e1_3.push_back( ( transformMatrix * e0_3[i] ) );
	}

	Surface_mesh mesh1;
	reconstruct_mesh(mesh0, mesh1);
	// vector<Vector3f> ~p = eqn14(A, B, C, e1_1, e1_2, e1_3)

	return mesh1;
}

// TODO: find out what is t0, t1, h0, h1
Matrix3f get_scaling_matrix() {

	const float t0 = 0;
	const float h0 = get_h_field(t0);

	const float t1 = 0;
	const float h1 = get_h_field(t1);

	const float s = sqrt(h0 / h1);

	return Scaling(s, s, s);

}