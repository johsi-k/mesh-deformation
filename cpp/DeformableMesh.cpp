#include "..\headers\DeformableMesh.h"
#include "..\headers\eqn6.h"

DeformableMesh::DeformableMesh(Surface_mesh &mesh) : _original(mesh), mesh( *(new Surface_mesh(mesh)) ) {

	for (auto e : _original.edges()) {
		Matrix3f m_frame = Matrix3f();
		m_frame <<
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f;

		this->frame_rotated.push_back(m_frame); 

		Point p0 = _original.position(_original.vertex(e, 0));
		Point p1 = _original.position(_original.vertex(e, 1));
		Point p = p1 - p0;

		Vector3f ve = p.matrix();
		this->coeffs.push_back(ve);
	}

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
	for (int i = 0; i < _original.vertices_size(); i++) {
		this->frame_rotated[i] = rotationMatrix * this->frame_origin[i];
	}

	const Matrix3f scaleMatrix = get_scaling_matrix();

	reconstruct_mesh();

}

void DeformableMesh::reconstruct_mesh() {

	typedef SparseMatrix<float> MatrixType;
	typedef SparseLU< MatrixType > SolverType;
	typedef Surface_mesh::Vertex Vertex;
	typedef Triplet<float, int> Triplet; 

	const int ne = _original.edges_size();
	const int nv = _original.vertices_size();
	vector<Triplet> A_entries; // i, j, value

	MatrixX3f b(ne, 3);

	for (auto edge : _original.edges()) {

		int v0 = _original.vertex(edge, 0).idx();
		int v1 = _original.vertex(edge, 1).idx();

		int eid = edge.idx();

		// A
		A_entries.push_back(Triplet(eid, v0, -1));
		A_entries.push_back(Triplet(eid, v1,  1));

		b.row(eid) = this->frame_rotated[eid] * this->coeffs[eid];
	}

	MatrixType A(ne, nv);
	A.setFromTriplets(A_entries.begin(), A_entries.end());
	const MatrixType At = A.transpose();
	const MatrixType AA = At * A;

	const VectorXf bx = At * b.col(0);
	const VectorXf by = At * b.col(1);
	const VectorXf bz = At * b.col(2);

	//solve linear system
	SolverType solver;
	solver.compute(AA);
	const VectorXf x = solver.solve(bx);
	const VectorXf y = solver.solve(by);
	const VectorXf z = solver.solve(bz);

	std::cout << A << std::endl;
	std::cout << AA << std::endl;
	
	cout << "original | new" << endl;
	int i = 0;
	for (auto v : _original.vertices()) {
		Point p = _original.position(v);
		cout << i << ": " << p.x() << ", " << p.y() << ", " << p.z()
			<< " | " << x[i] << ", " << y[i] << ", " << z[i]
			<< endl;

		mesh.position(v) = Vector3f(x[i], y[i], z[i]);
		i++;
	}

	i = 0;
	cout << "_original" << endl;
	for (auto v : _original.vertices()) {
		Point p = _original.position(v);
		cout << i << ": " << p.x() << ", " << p.y() << ", " << p.z() << endl;
		i++;
	}

	i = 0;
	cout << "mesh" << endl;
	for (auto v : mesh.vertices()) {
		Point p = mesh.position(v);
		cout << i << ": " << p.x() << ", " << p.y() << ", " << p.z() << endl;
		i++;
	}
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

	return Scaling(1.0f, 1.0f, s);

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