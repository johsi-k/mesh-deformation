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

	void deform_mesh(const Surface_mesh *mesh0);
	void test() {
		reconstruct_mesh();
	};

private:

	Surface_mesh &_original;
	
	vector<Matrix3f> frame_rotated;
	vector<Matrix3f> frame_origin;

	vector<float> localDepth;

	void reconstruct_mesh();
	float get_h_field(float t);
	Matrix3f get_scaling_matrix();
	Matrix3f quaternion2rotationMatrix(Vector4f quaternion);
	Vector4f orthoParamsToQuarternion(Vector3f orthoParams);
	Vector4f conformalParamsToQuaternion(Vector3f conformalParams);

};

#endif