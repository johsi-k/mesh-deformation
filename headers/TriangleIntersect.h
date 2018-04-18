#pragma once
#include "../Ray.h"
#include "../include/surface_mesh/Surface_mesh.h"

using namespace surface_mesh;

class TriangleIntersect {
public:
	static Vector3f intersect(Ray r, float tmin, float &distance, Surface_mesh *mesh);
	static Matrix3f generateMatrix3fFromVectors(Vector3f a, Vector3f b, Vector3f c);
};