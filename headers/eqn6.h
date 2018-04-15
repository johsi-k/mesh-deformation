#pragma once
#ifndef EQN6
#define EQN6

#include <Eigen\Geometry>
#include <Eigen\Sparse>
#include <surface_mesh\Surface_mesh.h>

using namespace Eigen;
using namespace surface_mesh;
using namespace std;

void eqn6(const Surface_mesh& mesh, vector<int>& fixed_ids, vector<int>& handle_ids,
	VectorXf& theta_initial, float theta_input, VectorXf& out);

#endif