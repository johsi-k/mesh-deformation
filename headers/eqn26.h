#pragma once
#ifndef EQN26
#define EQN26

#include <Eigen\Geometry>

using namespace Eigen;

Matrix3f quaternion2rotationMatrix(Vector4f quaternion);

#endif