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
	
	vector<Eigen::Vector3f> coeffs;
	vector<Eigen::Matrix3f> frame_rotated;
	vector<Eigen::Matrix3f> frame_origin;

	vector<float> localDepth;

	void reconstruct_mesh();
	float get_h_field(float t);
	Eigen::Matrix3f get_scaling_matrix();
	Eigen::Matrix3f quaternion2rotationMatrix(Eigen::Vector4f quaternion);
	Eigen::Vector4f orthoParamsToQuarternion(Eigen::Vector3f orthoParams);
	Eigen::Vector4f conformalParamsToQuaternion(Eigen::Vector3f conformalParams);

};

#endif