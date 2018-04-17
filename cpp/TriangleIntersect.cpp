#include "../headers/TriangleIntersect.h"

int TriangleIntersect::intersect(Ray r, float tmin, float &distance, Surface_mesh *mesh) {
	int index = INT_MAX;
	float t = FLT_MAX;

	Surface_mesh::Face_container container = mesh->faces();
	Surface_mesh::Face_iterator face_iter;
	Surface_mesh::Vertex_around_face_circulator vafc, vafc_end;

	int f_i = 0;

	for (face_iter = mesh->faces_begin(); face_iter != mesh->faces_end(); ++face_iter) {
		vafc = mesh->vertices(*face_iter);
		vafc_end = vafc;

		Vector3f points[3];
		int i = 0;
		do {
			Surface_mesh::Vertex v = *vafc;
			points[i] = mesh->position(v);
			i++;

		} while (++vafc != vafc_end);

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

	distance = r.pointAtParameter(t).norm();
	return index;
}

Matrix3f TriangleIntersect::generateMatrix3fFromVectors(Vector3f a, Vector3f b, Vector3f c) {
	Matrix <float, 3, 3, ColMajor> M;
	M <<
		a.x(), a.y(), a.z(),
		b.x(), b.y(), b.z(),
		c.x(), c.y(), c.z();
	return M;
}