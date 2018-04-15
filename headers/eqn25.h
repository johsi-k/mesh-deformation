#pragma once
#ifndef EQN25
#define EQN25

#include <Eigen\Geometry>

using namespace Eigen;

Vector4f orthoParamsToQuarternion(Vector3f orthoParams);
Vector4f conformalParamsToQuaternion(Vector3f conformalParams);

#endif