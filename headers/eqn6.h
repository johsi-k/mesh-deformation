#pragma once
#ifndef EQN6
#define EQN6

#include <Eigen\Eigen>
#include <surface_mesh\Surface_mesh.h>

using namespace Eigen;
using namespace surface_mesh;
using namespace std;

vector <float> theta(const Surface_mesh& mesh, vector<int>& fixed_ids, vector<int>& handle_ids,
	VectorXf& theta_initial, float theta_input, Eigen::VectorXd& out);

#endif