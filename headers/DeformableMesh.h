#pragma once
#ifndef DEFORMABLE_MESH
#define DEFORMABLE_MESH

#include <Eigen\Geometry>
#include <surface_mesh\Surface_mesh.h>

using namespace Eigen;
using namespace surface_mesh;
using namespace std;


class DeformableMesh
{

public:

	DeformableMesh(Surface_mesh &mesh);

	Surface_mesh &mesh;

	void deform_mesh(vector<int> fixed_ids, vector<int> handle_ids, VectorXf theta_initial, float theta_input);

private:

	Surface_mesh &_original;
	
	vector<Matrix3f> frame_rotated;
	vector<Matrix3f> frame_origin;

	vector<float> localDepth;

	void reconstruct_mesh( vector<int> fixed_ids );
	void get_orthos(const Surface_mesh& mesh, vector<int>& fixed_ids, vector<int>& handle_ids,
		VectorXf& theta_initial, float theta_input, VectorXf& out);
	float get_h_field(float t);
	Matrix3f get_scaling_matrix();
	Matrix3f quaternion2rotationMatrix(Vector4f quaternion);
	MatrixX4f orthoParamsToQuarternion(MatrixX3f orthoParams);
	Vector4f orthoParamsToQuarternion(Vector3f orthoParams);
	Vector4f conformalParamsToQuaternion(Vector3f conformalParams);
	vector<float> computeInternalDistances();

};

#endif