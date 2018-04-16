#include "..\headers\DeformableMesh.h"
#include "..\headers\eqn6.h"

DeformableMesh::DeformableMesh(Surface_mesh &mesh) {
	this->_original = mesh;
}

void DeformableMesh::deform_mesh(const Surface_mesh *mesh0) {

	const MatrixX3f PD1, PD2;
	const VectorXf PV1, PV2;

	const vector<Vector3f> p;

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
	for (int i = 0; i < e0_1.size(); i++) {
		this->e1_1[i] = rotationMatrix * this->e0_1[i];
		this->e1_2[i] = rotationMatrix * this->e0_2[i];
		this->e1_3[i] = rotationMatrix * this->e0_3[i];
	}

	//const Matrix3f scaleMatrix = get_scaling_matrix();

	reconstruct_mesh();
	// vector<Vector3f> ~p = eqn14(A, B, C, e1_1, e1_2, e1_3)

}

void DeformableMesh::reconstruct_mesh() {

	typedef SparseMatrix<float> MatrixType;
	typedef SparseLU< MatrixType > SolverType;
	typedef Surface_mesh::Vertex Vertex;
	typedef Triplet<float, int> Triplet; 

	const int ne = _original.edges_size();
	const int nv = _original.vertices_size();
	vector<Triplet> A_entries; // i, j, value
	vector<float>   b_entries;

	int n = 0;

	for (auto edge : _original.edges()) {
		int v0 = _original.vertex(edge, 0).idx();
		int v1 = _original.vertex(edge, 1).idx();

		// A
		A_entries.push_back(Triplet(n, v0,  1));
		A_entries.push_back(Triplet(n, v1, -1));

		A_entries.push_back(Triplet(n + 1, v0,  1));
		A_entries.push_back(Triplet(n + 1, v1, -1));

		A_entries.push_back(Triplet(n + 2, v0,  1));
		A_entries.push_back(Triplet(n + 2, v1, -1));

		// b
		b_entries.push_back(
			this->A[v0] * e1_1[v0].x() + 
			this->B[v0] * e1_2[v0].x() +
			this->C[v0] * e1_3[v0].x()
		);
		b_entries.push_back(
			this->A[v0] * e1_1[v0].y() +
			this->B[v0] * e1_2[v0].y() +
			this->C[v0] * e1_3[v0].y()
		);
		b_entries.push_back(
			this->A[v0] * e1_1[v0].z() +
			this->B[v0] * e1_2[v0].z() +
			this->C[v0] * e1_3[v0].z()
		);

		n += 3;
	}

	MatrixType A(ne, nv);
	VectorXf b = VectorXf::Zero(nv);
	VectorXf x;

	A.setFromTriplets(A_entries.begin(), A_entries.end());
	
	for (int i = 0; i < nv; i++) {
		b[i] = b_entries[nv];
	}

	//solve linear system
	SolverType solver;
	solver.compute(A);
	x = solver.solve(b);

	std::cout << A << std::endl;

	std::cout << "result" << std::endl;
	std::cout << x.transpose() << std::endl;
}

float DeformableMesh::get_h_field(float t) {

	const float k1 = 0;
	const float k2 = 0;
	const float a1 = 0;
	const float a2 = 0;

	if (k1 == 0 && k2 == 0) return t;
	else if (k1 != 0 && k2 == 0) return (t*t / 2 + 2 * a1*t) / (a1 + t);
	else if (k1 == 0 && k2 != 0) return (t*t / 2 + 2 * a2*t) / (a2 + t);
	return (t*t*t / 3 + (a1 + a2)*t*t / 2 + a1 * a2*t) / (a1 + t) / (a2 + t);
}

// TODO: find out what is t0, t1, h0, h1
Matrix3f DeformableMesh::get_scaling_matrix() {

	const float t0 = 0;
	const float h0 = get_h_field(t0);

	const float t1 = 0;
	const float h1 = get_h_field(t1);

	const float s = sqrt(h0 / h1);

	return Scaling(s, s, s);

}

Matrix3f DeformableMesh::quaternion2rotationMatrix(Vector4f quaternion) {

	const float x = quaternion[0];
	const float y = quaternion[1];
	const float z = quaternion[2];
	const float w = quaternion[3];

	Matrix3f rotationMatrix;
	rotationMatrix <<
		1 - 2 * y*y - 2 * z*z,
		2 * x*y + 2 * w*z,
		2 * x*z - 2 * w*y,

		2 * x*y - 2 * w*z,
		1 - 2 * x*x - 2 * z*z,
		2 * y*z + 2 * w*x,

		2 * x*z + 2 * w*y,
		2 * y*z - 2 * w*x,
		1 - 2 * x*x - 2 * y*y;

	return rotationMatrix;

}

// eqn 25
Vector4f DeformableMesh::orthoParamsToQuarternion(Vector3f orthoParams) {
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
Vector4f DeformableMesh::conformalParamsToQuaternion(Vector3f conformalParams) {

	const float n1 = conformalParams[0];
	const float n2 = conformalParams[1];
	const float n3 = conformalParams[2];

	const float n_abs_sqr = conformalParams.squaredNorm();
	const float denom = 1 / (n_abs_sqr + 4);

	const float x = 4 * n1 * denom;
	const float y = 4 * n2 * denom;
	const float z = 4 * n3 * denom;
	const float w = (4 - n_abs_sqr) * denom;

	return Vector4f(x, y, z, w);
}