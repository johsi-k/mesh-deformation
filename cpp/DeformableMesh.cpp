#define EIGEN_DONT_ALIGN
#define _USE_MATH_DEFINES
#include <cmath>

#include "..\headers\DeformableMesh.h"
#include "..\headers\eqn6.h"
#include "igl\principal_curvature.h";


Vector3f global_to_local(const Vector3f& global,
	const Vector3f& origin,
	const Matrix3f& frame)
{
	return frame.inverse() * (global - origin);
}

DeformableMesh::DeformableMesh(Surface_mesh &mesh) : _original(mesh), mesh( *(new Surface_mesh(mesh)) ) {

	const int nv = _original.vertices_size();
	const int nf = _original.faces_size();

	MatrixX3f V(nv, 3);
	MatrixX3i F(nf, 3);

	for (auto v : _original.vertices()) {

		for (int i = 0; i < 3; i++)
			V( v.idx(), i ) = _original.position(v).matrix()[i];
	}

	for (Surface_mesh::Face f : _original.faces()) {

		int i = 0;
		
		for (auto v : _original.vertices(f)) {
			F( f.idx(), i ) = v.idx();
			i++;
		}

	}

	MatrixX3f PD1, PD2;
	VectorXf PV1, PV2;

	cout << V << endl;
	cout << F << endl;

	cout << "Principal" << endl;
	
	igl::principal_curvature<MatrixX3f, MatrixX3i, MatrixX3f, MatrixX3f, VectorXf, VectorXf>
		(V, F, PD1, PD2, PV1, PV2);
	
	cout << PD1 << endl;
	cout << PD2 << endl;

	for (auto e : _original.edges()) {
		Matrix3f m_frame = Matrix3f();
		m_frame <<
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f;

		this->frame_origin.push_back(m_frame);
		this->frame_rotated.push_back(m_frame);
	}

	vector<int> fixed_ids;
	vector<int> handle_ids;
	VectorXf theta_initial;
	float theta_input;

	fixed_ids.push_back(0);
	fixed_ids.push_back(1);
	fixed_ids.push_back(2);
	fixed_ids.push_back(3);

	handle_ids.push_back(12);
	handle_ids.push_back(13);
	handle_ids.push_back(14);
	handle_ids.push_back(15);

	theta_initial = VectorXf::Zero(mesh.vertices_size());
	theta_input = 30*M_PI/180;


	deform_mesh(fixed_ids, handle_ids, theta_initial, theta_input);
}

void DeformableMesh::deform_mesh(
	vector<int> fixed_ids, 
	vector<int> handle_ids, 
	VectorXf theta_initial, 
	float theta_input
) 
{
	cout << "fixed_ids size: " << fixed_ids.size() << "handle_ids size: " << handle_ids.size() << endl;
	cout << "initial theta:" << endl << theta_initial << endl;
	cout << "theta_input: " << endl << theta_input << endl;
	const vector<Vector3f> p;

	MatrixX3f params(_original.vertices_size(), 3);
	bool is_one_axis = true;

	if (is_one_axis) {
		VectorXf ortho;

		eqn6(_original, fixed_ids, handle_ids, theta_initial, theta_input, ortho);

		params.col(0) = ortho;
	}
	else {
		// eq2(*mesh0, conformal);
	}

	const MatrixX4f quat = orthoParamsToQuarternion(params);

	for (int i = 0; i < _original.vertices_size(); i++) {
		const Matrix3f rotationMatrix = quaternion2rotationMatrix(quat.row(i));
		this->frame_rotated[i] = rotationMatrix * this->frame_origin[i];
	}

	////const Matrix3f scaleMatrix = get_scaling_matrix();

	reconstruct_mesh(fixed_ids);

}

template <class T>
bool contains(const vector<T> &v, T t) {
	return std::find(v.begin(), v.end(), t) != v.end();
}

void DeformableMesh::reconstruct_mesh( vector<int> fixed_ids ) {

	typedef SparseMatrix<float> MatrixType;
	typedef SparseLU< MatrixType > SolverType;
	typedef Surface_mesh::Vertex Vertex;
	typedef Triplet<float, int> Triplet; 

	const int nv = _original.vertices_size();
	vector<Triplet> A_entries; // i, j, value

	vector<Surface_mesh::Edge> new_edges;

	for (auto edge : _original.edges()) {

		const Vertex v0 = _original.vertex(edge, 0);
		const Vertex v1 = _original.vertex(edge, 1);

		const int v0_idx = v0.idx();
		const int v1_idx = v1.idx();

		const bool is_v0_fixed = contains(fixed_ids, v0_idx);
		const bool is_v1_fixed = contains(fixed_ids, v1_idx);

		if (!(is_v0_fixed && is_v1_fixed)) {
			new_edges.push_back(edge);
		}
	}

	const int ne = new_edges.size();
	MatrixX3f b(ne, 3);

	vector<int> lookup;

	int j = 0;
	for (int i = 0; i < _original.vertices_size(); i++) {
		if (contains(fixed_ids, i)) {
			lookup.push_back(-1);
		} else
		{
			lookup.push_back(j);
			j++;
		}
	}

	int eid = 0;
	for (Surface_mesh::Edge edge : new_edges) {

		const Vertex v0 = _original.vertex(edge, 0);
		const Vertex v1 = _original.vertex(edge, 1);
		const int v0_idx = v0.idx();
		const int v1_idx = v1.idx();

		const bool is_v0_fixed = contains(fixed_ids, v0_idx);
		const bool is_v1_fixed = contains(fixed_ids, v1_idx);

		const Point p0 = _original.position(v0);
		const Point p1 = _original.position(v1);

		Vector3f b_row = this->frame_rotated[v1_idx] * global_to_local(p1, p0, this->frame_origin[v1_idx]);
		
		if (is_v0_fixed) b_row += _original.position(v0);
		else             A_entries.push_back(Triplet(eid, lookup[v0_idx], -1));

		if (is_v1_fixed) b_row -= _original.position(v1);
		else             A_entries.push_back(Triplet(eid, lookup[v1_idx], 1));

		b.row(eid) = b_row;
		eid++;
	}

	cout << _original.edges_size() << " " << ne << endl;

	MatrixType A(ne, nv - 4);
	A.setFromTriplets(A_entries.begin(), A_entries.end());
	const MatrixType At = A.transpose();
	const MatrixType AA = At * A;

	std::cout << A << std::endl;
	std::cout << AA << std::endl;

	const VectorXf bx = At * b.col(0);
	const VectorXf by = At * b.col(1);
	const VectorXf bz = At * b.col(2);

	//solve linear system
	SolverType solver;
	cout << "Test" << endl;
	solver.compute(AA);
	const VectorXf x = solver.solve(bx);
	const VectorXf y = solver.solve(by);
	const VectorXf z = solver.solve(bz);
	
	cout << "original | new" << endl;
	int i = 0;
	j = 0;
	for (auto v : _original.vertices()) {
		Point p = _original.position(v);
		int idx;
		Vector3f p_new;

		if (contains(fixed_ids, i)) {
			p_new = p;
		}
		else {
			p_new = Vector3f(x[j], y[j], z[j]);
			j++;
		}

		cout << i << ": " << p.transpose() << " | " << p_new.transpose() << endl;


		mesh.position(v) = p_new;
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
	cout << "A";
	const float x = quaternion[0];
	const float y = quaternion[1];
	const float z = quaternion[2];
	cout << "A";
	const float w = quaternion[3];
	cout << "A";

	Matrix3f rotationMatrix;
	cout << "A";
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

	cout << "A";
	return rotationMatrix;

}

// eqn 25
MatrixX4f DeformableMesh::orthoParamsToQuarternion(MatrixX3f orthoParams) {
	const int r = orthoParams.rows();
	MatrixX4f quat(r, 4);

	for (int i = 0; i < r; i++) {
		quat.row(i) = this->orthoParamsToQuarternion((Vector3f)orthoParams.row(i));
	}

	return quat;
}

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