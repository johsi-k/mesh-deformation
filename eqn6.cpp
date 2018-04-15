#include "..\headers\eqn6.h"


void eqn6(const Surface_mesh& mesh, vector<int>& fixed_ids, vector<int>& handle_ids,
	VectorXf& theta_initial, float theta_input, VectorXf& out)
{
	typedef SparseMatrix<float> MatrixType;
	typedef SparseLU< MatrixType > SolverType;

	typedef Triplet<float, int> Triplet; // for filling out the matrix

	const int nv = mesh.vertices_size(); //number of equations and unknowns

	// build linear system Ax = B

	// entries for A matrix
	vector<Triplet> entries; // i, j, value

	for (auto v : mesh.vertices()) {

		int j = v.idx();

		//cout << "vertex:" << v << endl;

		// h is outgoing halfedge from each vertex
		Surface_mesh::Halfedge h0, h;
		h0 = h = mesh.halfedge(v);

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
				Surface_mesh::Vertex vj = mesh.from_vertex(h);
				Surface_mesh::Vertex vi = mesh.to_vertex(h);
				Surface_mesh::Vertex vl = mesh.to_vertex(mesh.next_halfedge(h));
				Surface_mesh::Vertex vk = mesh.to_vertex(mesh.next_halfedge(mesh.opposite_halfedge(h)));

				int i = vi.idx();

				// find points
				Vector3f pj = mesh.position(v);
				Vector3f pi = mesh.position(vi);
				Vector3f pl = mesh.position(vl);
				Vector3f pk = mesh.position(vk);

				// find length of edges
				Vector3f li = pi - pl;
				Vector3f lj = pj - pl;
				Vector3f ki = pi - pk;
				Vector3f kj = pj - pk;

				float cot_a = 0;
				float cot_b = 0;

				if (!mesh.is_boundary(h)) {
					cot_a = li.dot(lj) / li.cross(lj).norm();
				}

				if (!mesh.is_boundary(mesh.opposite_halfedge(h))) {
					cot_b = ki.dot(kj) / ki.cross(kj).norm();
				}


				// the weight of vertex i for reference vertex j
				float Wji = cot_a + cot_b;
				Wjj += Wji;

				// add weight Wji
				entries.push_back(Triplet(j, i, -Wji));

				h = mesh.next_halfedge(mesh.opposite_halfedge(h)); // move to next vertex
			} while (h != h0);

			// add diagonal weight Wjj
			entries.push_back(Triplet(j, j, Wjj));

		}
	}

	MatrixType A;
	A.resize(nv, nv);
	A.setFromTriplets(entries.begin(), entries.end());

	//build b
	VectorXf b = VectorXf::Zero(A.cols());
	for (auto i : fixed_ids) {
		b[i] = theta_initial[i];
	}

	for (auto i : handle_ids) {
		b[i] = theta_input;
	}

	//solve linear system
	SolverType solver;
	solver.compute(A);
	out = solver.solve(b);

	std::cout << A << std::endl;

	std::cout << "result" << std::endl;
	std::cout << out.transpose() << std::endl;
}
