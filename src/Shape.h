#pragma  once
#ifndef SHAPE_H
#define SHAPE_H

#include <cassert>
#include <cstring>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include "hitInfo.h"

class Shape {
public:
	Shape();
	~Shape();
	virtual hitInfo intersect(const glm::vec3& ray, glm::vec3& origin, const float& t0, float& t1) const = 0;

	glm::vec3 position;
	glm::vec3 rotate;
	glm::vec3 scale;
	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;
	float s;
	float reflect;
};

class Plane : public Shape {
public:
	Plane();

	hitInfo intersect(const glm::vec3& ray, glm::vec3& origin, const float& t0, float& t1) const override;
};

class Sphere : public Shape {
public:
	Sphere();

	hitInfo intersect(const glm::vec3& ray, glm::vec3& origin, const float& t0, float& t1) const override;
};

class Ellipsoid : public Shape {
public:
	Ellipsoid();

	hitInfo intersect(const glm::vec3& ray, glm::vec3& origin, const float& t0, float& t1) const override;
};

class TriMesh : public Shape {
public:
	std::vector<float> posBuf; // list of vertex positions
	std::vector<float> norBuf; // list of vertex normals
	glm::vec3 lowerBounds;
	glm::vec3 upperBounds;

	TriMesh(std::vector<float> pos, std::vector<float> nor, glm::vec3 lowBounds, glm::vec3 upBounds);

	hitInfo intersect(const glm::vec3& ray, glm::vec3& origin, const float& t0, float& t1) const override;
};


#endif // SHAPE_H
