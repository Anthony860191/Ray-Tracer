#include "Shape.h"
#include <cmath>

using namespace glm;
using namespace std;

Shape::Shape() :
    position(0.0f, 0.0f, 0.0f),
    rotate(0.0f, 0.0f, 0.0f),
    scale(1.0f, 1.0f, 1.0f),
    ambient(0.0f, 0.0f, 0.0f),
    diffuse(0.0f, 0.0f, 0.0f),
    specular(0.0f, 0.0f, 0.0f),
    s(0.0f),
	reflect(0.0f)
{
}

Shape::~Shape()
{
}

Plane::Plane() {}

hitInfo Plane::intersect(const vec3& ray, glm::vec3& origin, const float& t0, float& t1) const{
	hitInfo info;
	info.distance = INFINITY;

	float t = dot(rotate, (position - origin)) / dot(rotate, ray);
	if (t <= t1 && t >= t0) {
		vec3 pos = origin + t * ray;
		vec3 nor = rotate;
		info = { pos, nor, t };
		return info;
	}

	return info; // returns INFINITY for info.distance if there are no intersections
}

Sphere::Sphere() {}

// returns the minimum intersection with a sphere if there is one
hitInfo Sphere::intersect(const vec3& ray, glm::vec3& origin, const float& t0, float& t1) const{
	hitInfo info; // initialize hit variable to store info of the intersection
	info.distance = INFINITY;

	vec3 pc = origin - position;
	float a = dot(ray, ray);
	float b = 2 * dot(ray, pc);
	float c = dot(pc, pc) - scale.x * scale.x;
	float d = (b * b) - (4.0f * a * c);

	if (d > 0) {
		float inter1 = (-b + sqrt(d)) / (2 * a);
		float inter2 = (-b - sqrt(d)) / (2 * a);
		inter1 = std::min(inter1, inter2);
		if (inter1 <= t1 && inter1 >= t0) {
			vec3 pos = origin + inter1 * ray;
			vec3 nor = (pos - position) / scale.x;
			info ={ pos, nor, inter1 };
			return info;
		} 
	}
	return info; // returns INFINITY for info.distance if there are no intersections
}

Ellipsoid::Ellipsoid() {}

// returns the minimum intersection with an ellipsoid if there is one
hitInfo Ellipsoid::intersect(const vec3& ray, glm::vec3& origin, const float& t0, float& t1) const {
	hitInfo info; // initialize hit variable to store info of the intersection
	info.distance = INFINITY;

	mat4 scaleMatrix = glm::scale(mat4(1.0f), scale);
	mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);
	mat4 ellipsoid_E = translationMatrix * scaleMatrix;

	vec3 p = inverse(ellipsoid_E) * vec4(origin, 1.0f);
	vec3 v = normalize(inverse(ellipsoid_E) * vec4(ray, 0.0f));
	float a = dot(v, v);
	float b = 2 * dot(v, p);
	float c = dot(p, p) - 1.0;
	float d = (b * b) - (4.0f * a * c);
	if (d > 0) {
		float inter1 = (-b + sqrt(d)) / (2 * a);
		float inter2 = (-b - sqrt(d)) / (2 * a);
		inter1 = std::min(inter1, inter2);
		
		vec3 pos = p + inter1 * v;
		vec3 nor = normalize(inverse(transpose(ellipsoid_E)) * vec4(pos, 0.0f));
		pos = ellipsoid_E * vec4(pos, 1.0f);
		inter1 = length(pos - origin);

		if (dot(ray, (pos - origin)) < 0) {
			inter1 = -inter1;
		}

		if (inter1 <= t1 && inter1 >= t0) {
			info = { pos, nor, inter1};
			return info;
		}
	}
	return info; // returns INFINITY for info.distance if there are no intersections
}

TriMesh::TriMesh(vector<float>pos, vector<float>nor, vec3 lowBounds, vec3 upBounds) : posBuf(pos), norBuf(nor), lowerBounds(lowBounds), upperBounds(upBounds) {}

hitInfo TriMesh::intersect(const vec3& ray, glm::vec3& origin, const float& t0, float& t1) const
{
	hitInfo info;
	info.distance = INFINITY;

	// Transform ray to local coordinates
	mat4 scaleMatrix = glm::scale(mat4(1.0f), scale);
	mat4 rotationMatrix = glm::rotate(mat4(1.0f), rotate.x, vec3(1.0f, 0.0f, 0.0f))
		* glm::rotate(mat4(1.0f), rotate.y, vec3(0.0f, 1.0f, 0.0f))
		* glm::rotate(mat4(1.0f), rotate.z, vec3(0.0f, 0.0f, 1.0f));
	mat4 translationMatrix = glm::translate(mat4(1.0f), position);
	mat4 transformationMatrix = translationMatrix * rotationMatrix * scaleMatrix;

	vec3 originLocal = inverse(transformationMatrix) * vec4(origin, 1.0f);
	vec3 rayLocal = normalize(inverse(transformationMatrix) * vec4(ray, 0.0f));

	// Calculate Bounding Sphere
	float radius = distance(lowerBounds, upperBounds) / 2.0f;
	vec3 center = mix(lowerBounds, upperBounds, 0.5f);
	shared_ptr<Sphere> boundingSphere = make_shared<Sphere>();
	boundingSphere->position = center;
	boundingSphere->scale *= radius;

	hitInfo temp = boundingSphere->intersect(rayLocal, originLocal, t0, t1);

	// Check if the rays origin is within the sphere
	bool inSphere = false;
	if (distance(center, originLocal) <= radius) {
		inSphere = true;
	}

	// Check if there is an intersection with bounding sphere or ray originates within sphere before checking every mesh triangle
	if (temp.distance != INFINITY || inSphere) {
		float EPSILON = 0.000001f;
		for (int i = 0; i < posBuf.size(); i += 9) {
			vec3 edge1, edge2, tvec, pvec, qvec;
			float det, inv_det, u, v, t;

			vec3 vert0 = vec3(posBuf[i], posBuf[i + 1], posBuf[i + 2]);
			vec3 vert1 = vec3(posBuf[i + 3], posBuf[i + 4], posBuf[i + 5]);
			vec3 vert2 = vec3(posBuf[i + 6], posBuf[i + 7], posBuf[i + 8]);

			vec3 nor0 = vec3(norBuf[i], norBuf[i + 1], norBuf[i + 2]);
			vec3 nor1 = vec3(norBuf[i + 3], norBuf[i + 4], norBuf[i + 5]);
			vec3 nor2 = vec3(norBuf[i + 6], norBuf[i + 7], norBuf[i + 8]);

			/* find vectors for two edges sharing vert0 */
			edge1 = vert1 - vert0;
			edge2 = vert2 - vert0;

			/* begin calculating determinant - also used to calculate U parameter */
			pvec = cross(rayLocal, edge2);

			/* if determinant is near zero, ray lies in plane of triangle */
			det = dot(edge1, pvec);

			/* calculate distance from vert0 to ray origin */
			tvec = originLocal - vert0;
			inv_det = 1.0 / det;

			qvec = cross(tvec, edge1);

			if (det > EPSILON)
			{
				u = dot(tvec, pvec);
				if (u < 0.0 || u > det)
					continue;

				/* calculate V parameter and test bounds */
				v = dot(rayLocal, qvec);
				if (v < 0.0 || u + v > det)
					continue;

			}
			else if (det < -EPSILON)
			{
				/* calculate U parameter and test bounds */
				u = dot(tvec, pvec);
				if (u > 0.0 || u < det)
					continue;

				/* calculate V parameter and test bounds */
				v = dot(rayLocal, qvec);
				if (v > 0.0 || u + v < det)
					continue;
			}
			else continue;  /* ray is parallell to the plane of the triangle */

			t = dot(edge2, qvec) * inv_det;
			u *= inv_det;
			v *= inv_det;

			vec3 pos = vert0 + u * (vert1 - vert0) + v * (vert2 - vert0);
			vec3 nor = nor0 + u * (nor1 - nor0) + v * (nor2 - nor0);

			pos = transformationMatrix * vec4(pos, 1.0f);
			nor = normalize(inverse(transpose(transformationMatrix)) * vec4(nor, 0.0f));
			t = length(pos - origin);
			if (dot(ray, (pos - origin)) < 0) {
				t = -t;
			}
			if (t < info.distance && t <= t1 && t >= t0) {
				info = { pos, nor, t };
			}
		}
	}
	return info;
}