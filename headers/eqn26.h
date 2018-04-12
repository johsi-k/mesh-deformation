#pragma once
#ifndef EQN26
#define EQN26

#include <Eigen\Eigen>

using namespace Eigen;

Matrix3d quaternion2rotationMatrix(Vector4d quaternion);

#endif