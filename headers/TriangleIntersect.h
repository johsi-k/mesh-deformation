#pragma once
#include "../Ray.h"
#include "../include/surface_mesh/Surface_mesh.h"

using namespace surface_mesh;

class TriangleIntersect {
public:
	static Vector3f intersect(Ray r, float tmin, Vector3f &intersectionPoint, Surface_mesh *mesh);
	static Matrix3f generateMatrix3fFromVectors(Vector3f a, Vector3f b, Vector3f c);
};