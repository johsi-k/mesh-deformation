#pragma once
#ifndef EQN25
#define EQN25

#include <Eigen\Eigen>

using namespace Eigen;

Vector4d orthoParamsToQuarternion(Vector3d orthoParams);
Vector4d conformalParamsToQuaternion(Vector3d conformalParams);

#endif