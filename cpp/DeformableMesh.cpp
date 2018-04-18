#define EIGEN_DONT_ALIGN
#define _USE_MATH_DEFINES
#include <cmath>

#include "..\headers\DeformableMesh.h"
#include "..\headers\TriangleIntersect.h"
#include "igl\principal_curvature.h"
#include <algorithm>
#include "..\Ray.h"

Vector3f global_to_local(const Vector3f& global,
	const Vector3f& origin,
	const Matrix3f& frame)
{
	return  ((global - origin).transpose() * frame.inverse()).transpose();
}

DeformableMesh::DeformableMesh(Surface_mesh &mesh) : _original(mesh), mesh(*(new Surface_mesh(mesh))) {

	const int nv = _original.vertices_size();
	const int nf = _original.faces_size();

	MatrixX3f V(nv, 3);
	MatrixX3i F(nf, 3);

	for (auto v : _original.vertices()) {
		V.row(v.idx()) = _original.position(v).matrix();
	}

	for (Surface_mesh::Face f : _original.faces()) {

		int i = 0;

		for (auto v : _original.vertices(f)) {
			F(f.idx(), i) = v.idx();
			i++;
		}

	}

	MatrixX3f PD1, PD2;
	VectorXf PV1, PV2;

	igl::principal_curvature<MatrixX3f, MatrixX3i, MatrixX3f, MatrixX3f, VectorXf, VectorXf>
		(V, F, PD1, PD2, PV1, PV2);

	this->PV1 = PV1;
	this->PV2 = PV2;
	this->PD3.resize(_original.vertices_size(), 3);

	for (auto v : _original.vertices()) {
		const int vid = v.idx();
		Matrix3f m_frame(3, 3);
		//if (PD1.row(vid).squaredNorm() != 1.) {
			//m_frame = Matrix3f::Identity();
			m_frame << 
				0, 0, 1,
				1, 0, 0,
				0, 1, 0;
		/*}
		else {
			m_frame.row(0) = PD1.row(vid);
			m_frame.row(1) = PD2.row(vid);
			m_frame.row(2) = PD1.row(vid).cross(PD2.row(vid));
		}*/

		this->PD3.row(vid) = PD1.row(vid).cross(PD2.row(vid));

		this->frame_origin.push_back(m_frame);
		this->frame_rotated.push_back(m_frame);
	}

	this->computeInternalDistances(this->_original, this->localDepth);
}

void DeformableMesh::deform_mesh( const vector<int> &fixed_ids, const vector<int> &handle_ids,
	const VectorXf &theta_initial, const float theta_input )
{
	MatrixX3f params(_original.vertices_size(), 3);
	const bool is_one_axis = true;

	if (is_one_axis) {
		VectorXf ortho;

		get_orthos(fixed_ids, handle_ids, theta_initial, theta_input, ortho);

		params.col(0) = ortho;
	}
	else {
		//get_conformal(params);
	}

	const int r = params.rows();
	MatrixX4f quat(r, 4);
	orthoParamsToQuarternion(params, quat);

	for (int i = 0; i < _original.vertices_size(); i++) {
		const Matrix3f rotationMatrix = quaternion2rotationMatrix(quat.row(i));
		this->frame_rotated[i] = rotationMatrix * this->frame_origin[i];
	}

	reconstruct_mesh(fixed_ids);
	
	vector<float> deformDepth;
	this->computeInternalDistances(this->mesh, deformDepth);

	for (auto v : mesh.vertices()) {
		const Matrix3f scaleMatrix = get_scaling_matrix( v.idx(), localDepth[v.idx()], deformDepth[v.idx()] );

		this->frame_rotated[v.idx()] = scaleMatrix * this->frame_rotated[v.idx()];
	}

	reconstruct_mesh(fixed_ids);
}

template <class T>
bool contains(const vector<T> &v, T t) {
	return std::find(v.begin(), v.end(), t) != v.end();
}

void DeformableMesh::reconstruct_mesh(const vector<int> &fixed_ids) {

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
		}
		else
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

		Vector3f b_row = global_to_local(p1, p0, this->frame_origin[v1_idx]).transpose() * this->frame_rotated[v1_idx];

		if (is_v0_fixed) b_row += _original.position(v0);
		else             A_entries.push_back(Triplet(eid, lookup[v0_idx], -1));

		if (is_v1_fixed) b_row -= _original.position(v1);
		else             A_entries.push_back(Triplet(eid, lookup[v1_idx], 1));

		b.row(eid) = b_row;
		eid++;
	}

	MatrixType A(ne, nv - fixed_ids.size());
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

	int i = 0;
	for (auto v : _original.vertices()) {
		Point p = _original.position(v);
		Vector3f p_new;

		if (contains(fixed_ids, v.idx())) {
			p_new = p;
		}
		else {
			p_new = Vector3f(x[i], y[i], z[i]);
			i++;
		}

		mesh.position(v) = p_new;
	}
}

void DeformableMesh::get_conformal(vector<int>& fixed_ids, vector<int>& handle_ids,
	VectorXf& theta_initial, float theta_input, MatrixXf& out) {

}

void DeformableMesh::get_orthos(const vector<int>& fixed_ids, const vector<int>& handle_ids,
	const VectorXf& theta_initial, const float theta_input, VectorXf& out)
{
	typedef SparseMatrix<float> MatrixType;
	typedef SparseLU< MatrixType > SolverType;

	typedef Triplet<float, int> Triplet; // for filling out the matrix

	const int nv = _original.vertices_size(); //number of equations and unknowns

										 // build linear system Ax = B

										 // entries for A matrix
	vector<Triplet> entries; // i, j, value

	for (auto v : _original.vertices()) {

		int j = v.idx();

		//cout << "vertex:" << v << endl;

		// h is outgoing halfedge from each vertex
		Surface_mesh::Halfedge h0, h;
		h0 = h = _original.halfedge(v);

		float Wjj = 0; // weights on the diagonal

					   // if vertex index is found in handle or fixed group, set coefficients to 1
		bool is_handle_or_fixed = (find(handle_ids.begin(), handle_ids.end(), j) != handle_ids.end()) ||
			(find(fixed_ids.begin(), fixed_ids.end(), j) != fixed_ids.end());

		if (is_handle_or_fixed) {
			entries.push_back(Triplet(j, j, 1.0f));
		}
		else {

			do {
				// for every half edge, find vertices j, i, k, l 
				const Surface_mesh::Vertex vj = _original.from_vertex(h);
				const Surface_mesh::Vertex vi = _original.to_vertex(h);
				const Surface_mesh::Vertex vl = _original.to_vertex(_original.next_halfedge(h));
				const Surface_mesh::Vertex vk = _original.to_vertex(_original.next_halfedge(_original.opposite_halfedge(h)));

				const int i = vi.idx();

				// find points
				const Vector3f pj = _original.position(v);
				const Vector3f pi = _original.position(vi);
				const Vector3f pl = _original.position(vl);
				const Vector3f pk = _original.position(vk);

				// find length of edges
				const Vector3f li = pi - pl;
				const Vector3f lj = pj - pl;
				const Vector3f ki = pi - pk;
				const Vector3f kj = pj - pk;

				const float cot_a = !_original.is_boundary(h) ? li.dot(lj) / li.cross(lj).norm() : 0;
				const float cot_b = !_original.is_boundary(_original.opposite_halfedge(h)) ? ki.dot(kj) / ki.cross(kj).norm() : 0;

				//if (!_original.is_boundary(h)) {
				//	cot_a = li.dot(lj) / li.cross(lj).norm();
				//}

				//if (!_original.is_boundary(_original.opposite_halfedge(h))) {
				//	cot_b = ki.dot(kj) / ki.cross(kj).norm();
				//}


				// the weight of vertex i for reference vertex j
				const float Wji = cot_a + cot_b;
				Wjj += Wji;

				// add weight Wji
				entries.push_back(Triplet(j, i, -Wji));

				h = _original.next_halfedge(_original.opposite_halfedge(h)); // move to next vertex
			} while (h != h0);

			// add diagonal weight Wjj
			entries.push_back(Triplet(j, j, Wjj));

		}
	}

	MatrixType A;
	A.resize(nv, nv);
	A.setFromTriplets(entries.begin(), entries.end());

	//build b
	VectorXf b = VectorXf::Zero(A.cols(), 1);

	for (int i : fixed_ids) {
		b[i] = theta_initial[i];
	}

	for (auto i : handle_ids) {
		b[i] = theta_input;
	}

	//solve linear system
	SolverType solver;
	solver.compute(A);
	out = solver.solve(b);

}

float DeformableMesh::get_h_field(const int vid, const float t) {

	const float k1 = this->PV1[vid];
	const float k2 = this->PV2[vid];
	const float a1 = 1.0f / k1 - t;
	const float a2 = 1.0f / k2 - t;

	if (k1 == 0 && k2 == 0) return t;
	else if (k1 != 0 && k2 == 0) return (t*t / 2 + 2 * a1*t) / (a1 + t);
	else if (k1 == 0 && k2 != 0) return (t*t / 2 + 2 * a2*t) / (a2 + t);
	return (t*t*t / 3 + (a1 + a2)*t*t / 2 + a1 * a2*t) / (a1 + t) / (a2 + t);
}

Matrix3f DeformableMesh::get_scaling_matrix(const int vid, const float t0, const float t1) {

	const float h0 = get_h_field(vid, t0);
	const float h1 = get_h_field(vid, t1);

	const float s = sqrt(h0 / h1);

	return Scaling(1.0f, 1.0f, s);

}

Matrix3f DeformableMesh::quaternion2rotationMatrix(const Vector4f &quaternion) {

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
void DeformableMesh::orthoParamsToQuarternion(const MatrixX3f &orthoParams, MatrixX4f &out) {
	for (int i = 0; i < orthoParams.rows(); i++) {
		out.row(i) = this->orthoParamsToQuarternion((Vector3f)orthoParams.row(i));
	}
}

Vector4f DeformableMesh::orthoParamsToQuarternion(const Vector3f &orthoParams) {
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
Vector4f DeformableMesh::conformalParamsToQuaternion(const Vector3f &conformalParams) {

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

float truncFloat(float x) {
	return x <= 0 ? FLT_MAX : x;
}

void DeformableMesh::computeInternalDistances(const Surface_mesh &compute_mesh, vector<float> &out) {

	const float alpha = 0.5;
	const float beta = 0.5;
	
	for (auto v : _original.vertices()) {
		Vector3f inwardsRay = -compute_mesh.compute_vertex_normal(v);

		const float r1 = truncFloat(1.0f / this->PV1[v.idx()]);
		const float r2 = truncFloat(1.0f / this->PV2[v.idx()]);

		Ray r(_original.position(v), inwardsRay);
		float dist;
		TriangleIntersect::intersect(r, 0.01, dist, &mesh);
		float phi_p = min(min(alpha*dist, beta*r1), beta*r2);

		out.push_back(phi_p);
	}
}

void DeformableMesh::resetMesh() {
	mesh = _original;
}