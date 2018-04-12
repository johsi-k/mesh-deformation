#include <Eigen/Eigen>
#include <surface_mesh/Surface_mesh.h>

#include "eqn6.cpp"
#include "eqn13.cpp"
#include "eqn14.cpp"
#include "eqn25.cpp"
#include "eqn26.cpp"

using namespace surface_mesh;
using namespace Eigen;

Surface_mesh deform_mesh(const Surface_mesh *mesh0) {

	const vector<Vector3f> p;
	const vector<Vector3f> e1, e2, e3;
	const vector<float> A, B, C;

	Vector4d *quat;
	bool is_one_axis = true;

	if (is_one_axis) {
		vector<int> fixed_ids;
		vector<int> handle_ids;
		VectorXf theta_initial;
		float theta_input;
		VectorXd ortho;

		theta(*mesh0, fixed_ids, handle_ids, theta_initial, theta_input, ortho);

		quat = &orthoParamsToQuarternion(ortho);
	}
	else {
		Vector3d conformal;
		// eq2(*mesh0, conformal);

		quat = &conformalParamsToQuaternion(conformal);
	}

	// TODO: apply rotation matrix to initial frames
	const Matrix3d rotationMatrix = quaternion2rotationMatrix(*quat);

	// TODO: apply scaling to rotated moving frames
	const Matrix3d scaleMatrix = get_scaling_matrix();

	Surface_mesh mesh1 = reconstruct_mesh(mesh0);
	// vector<Vector3f> ~p = eqn14(A, B, C, ~e1, ~e2, ~e3)

	return mesh1;
}

// TODO: find out what is t0, t1, h0, h1
Matrix3d get_scaling_matrix() {

	const double t0 = 0;
	const double h0 = get_h_field(t0);

	const double t1 = 0;
	const double h1 = get_h_field(t1);

	const double s = sqrt(h0 / h1);

	return Scaling(s, s, s);

}