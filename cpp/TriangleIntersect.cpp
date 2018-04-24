#include "../headers/TriangleIntersect.h"

Vector3f TriangleIntersect::intersect(Ray r, float tmin, Vector3f &intersectionPoint, Surface_mesh *mesh) {
	int index = INT_MAX;
	float t = FLT_MAX;

	int f_i = 0;

	for (auto f : mesh->faces()) {
		Vector3f points[3];
		int i = 0;
		for (auto v : mesh->vertices(f)) {
			points[i] = mesh->position(v);
			i++;
		}
		Matrix3f t_m = generateMatrix3fFromVectors(points[0] - points[1], points[0] - points[2], points[0] - r.getOrigin());
		Matrix3f beta_m = generateMatrix3fFromVectors(points[0] - r.getOrigin(), points[0] - points[2], r.getDirection());
		Matrix3f gamma_m = generateMatrix3fFromVectors(points[0] - points[1], points[0] - r.getOrigin(), r.getDirection());

		Matrix3f A = generateMatrix3fFromVectors(points[0] - points[1], points[0] - points[2], r.getDirection());

		float beta = beta_m.determinant() / A.determinant();
		float gamma = gamma_m.determinant() / A.determinant();

		f_i++;

		if (beta < 0.0f || gamma < 0.0f) {
			continue;
		}

		if (beta + gamma > 1.0f) {
			continue;
		}

		float alpha = 1 - beta - gamma;
		float t_face = t_m.determinant() / A.determinant();

		if (t_face < t && t_face >= tmin) {
			t = t_face;
			index = f_i - 1;
		}
	}

	intersectionPoint = r.pointAtParameter(t);
	if (index < INT_MAX)
		return r.pointAtParameter(t);
	else
		return Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
}

Matrix3f TriangleIntersect::generateMatrix3fFromVectors(Vector3f a, Vector3f b, Vector3f c) {
	Matrix <float, 3, 3, ColMajor> M;
	M <<
		a.x(), a.y(), a.z(),
		b.x(), b.y(), b.z(),
		c.x(), c.y(), c.z();
	return M;
}