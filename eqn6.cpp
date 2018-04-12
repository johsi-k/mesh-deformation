//code for eq 6
#include <Eigen/Eigen>
#include <surface_mesh/Surface_mesh.h>

using namespace surface_mesh;
using namespace Eigen;
using namespace std;

vector <float> theta(const Surface_mesh& mesh, vector<int>& fixed_ids, vector<int>& handle_ids,
	VectorXf& theta_initial, float theta_input, Eigen::VectorXd& out)
{
	typedef Eigen::SparseMatrix<double> MatrixType;
	typedef Eigen::SparseLU< MatrixType > SolverType;

	typedef Eigen::Triplet<float, int> Triplet; // for filling out the matrix

	const int nv = mesh.vertices_size(); //number of equations and unknowns
										 //build linear system (AX = B)
										 //entries for A matrix

	vector<Triplet> entries; // i, j, value

	for (auto v : mesh.vertices()) {

		int j = v.idx();

		cout << "vertex:" << v << endl;

		// if indexes are found in handle or fixed group, set coefficients to 1

		// h is outgoing halfedge from each vertex
		Surface_mesh::Halfedge h0, h;
		h0 = h = mesh.halfedge(v);

		double Wjj = 0; // the weight on the diagonal


		do {
			// for every half edge, find vertices j, i, k, l 
			Surface_mesh::Vertex vj = mesh.from_vertex(h);
			Surface_mesh::Vertex vi = mesh.to_vertex(h);
			Surface_mesh::Vertex vl = mesh.to_vertex(mesh.next_halfedge(h));
			Surface_mesh::Vertex vk = mesh.to_vertex(mesh.next_halfedge(mesh.opposite_halfedge(h)));

			int i = vi.idx();

			// find points
			Point pj = mesh.position(v);
			Point pi = mesh.position(vi);
			Point pl = mesh.position(vl);
			Point pk = mesh.position(vk);

			// find length of edges
			Vector3f li = pi - pl;
			Vector3f lj = pj - pl;
			Vector3f ki = pi - pk;
			Vector3f kj = pj - pk;

			float cot_a = li.dot(lj) / li.cross(lj).norm();
			float cot_b = ki.dot(kj) / ki.cross(kj).norm();

			double Wji; //the weight of vertex i for reference vertex j

						//add weight Wji

			h = mesh.next_halfedge(mesh.opposite_halfedge(h)); // move to next vertex
		} while (h != h0);

		//add the weight wjj
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
}
