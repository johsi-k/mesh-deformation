////code for eq 6
//
//void
//eq(const Mesh& mesh,
//	..... // may need additional stuff
//	VectorXd& out)
//{
//	typedef SparseMatrix<float> MatrixType;
//	typedef SparseLU< MatrixType > SolverType;
//
//	typedef Triplet<float, int> Triplet; // for fillong out the matrix
//
//	const int nv = mesh.vertices_size(); //number of equations and unknowns
//										 //build linear system (AX = B)
//										 //entries for A matrix
//	std::vector<Triplet> entries;
//	for (auto v : mesh.vertices()) {
//
//		Mesh::Halfedge h0, h;
//		h0 = h = mesh.halfedge(v);
//
//		double Wjj = 0; // the weight on the diagonal
//
//
//		do {
//
//			double Wji; //the weight of vertex i for reference vertex j
//
//						//add weight Wji
//
//			h = mesh.next_halfedge(mesh.opposite_halfedge(h)); // move to next vertex
//		} while (h != h0);
//
//		//add the weight wjj
//	}
//}
//
//MatrixType A;
//A.resize(nv, nv);
//A.setFromTriplets(entries.begin(), entries.end());
//
////build right hand side
//VectorXd b = VectorXd::Zero(A.cols());
//
////solve linear system
//SolverType solver;
//solver.compute(A);
//out = solver.solve(b);
//}
