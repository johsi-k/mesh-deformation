//code for eq 6
#include <Eigen/SparseLU>
#include <Eigen/Sparse>
#include <surface_mesh/Surface_mesh.h>

using namespace surface_mesh;
using namespace Eigen;

void
eq(const Surface_mesh mesh,
	// may need additional stuff
	Eigen::VectorXd& out)
{
	typedef Eigen::SparseMatrix<float> MatrixType;
	typedef Eigen::SparseLU< MatrixType > SolverType;

	typedef Eigen::Triplet<float, int> Triplet; // for filling out the matrix

	const int nv = mesh.vertices_size(); //number of equations and unknowns
										 //build linear system (AX = B)
										 //entries for A matrix
	std::vector<Triplet> entries;
	for (auto v : mesh.vertices()) {

		Surface_mesh::Halfedge h0, h;
		h0 = h = mesh.halfedge(v);

		double Wjj = 0; // the weight on the diagonal


		do {

			double Wji; //the weight of vertex i for reference vertex j

						//add weight Wji

			h = mesh.next_halfedge(mesh.opposite_halfedge(h)); // move to next vertex
		} while (h != h0);

		//add the weight wjj
	}
}

//MatrixType A;
//A.resize(nv, nv);
//A.setFromTriplets(entries.begin(), entries.end());
//
////build right hand side
//Eigen::VectorXd b = Eigen::VectorXd::Zero(A.cols());
//
////solve linear system
//Eigen::SolverType solver;
//solver.compute(A);
//out = solver.solve(b);
//}