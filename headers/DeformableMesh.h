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

	void deform_mesh(const vector<int> &fixed_ids, const vector<int> &handle_ids, 
		const VectorXf &theta_initial, const float theta_input, const bool preserveVolume);
	void resetMesh();
	void getCurvature(const int index, Vector3f &e1, Vector3f &e2, Vector3f &e3);

private:

	Surface_mesh &_original;
	
	vector<Matrix3f> frame_rotated;
	vector<Matrix3f> frame_origin;
	VectorXf PV1, PV2;
	MatrixX3f *PD1, *PD2, *PD3;

	vector<float> localDepth;

	void reconstruct_mesh( const vector<int> &fixed_ids );

	void get_orthos(const vector<int>& fixed_ids, const vector<int>& handle_ids,
		const VectorXf& theta_initial, const float theta_input, VectorXf& out);
	void get_conformal(vector<int>& fixed_ids, vector<int>& handle_ids,
		VectorXf& theta_initial, float theta_input, MatrixXf& out);

	float get_h_field(const int vid, const float t);
	Matrix3f get_scaling_matrix(const int vid, const float t0, const float t1);
	Matrix3f quaternion2rotationMatrix(const Vector4f &quaternion);
	void orthoParamsToQuarternion(const MatrixX3f &orthoParams, MatrixX4f &out);
	Vector4f orthoParamsToQuarternion(const Vector3f &orthoParams);
	Vector4f conformalParamsToQuaternion(const Vector3f &conformalParams);
	void computeInternalDistances(const Surface_mesh &compute_mesh, vector<float> &out);

};

#endif