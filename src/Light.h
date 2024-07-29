#pragma  once
#ifndef LIGHT_H
#define LIGHT_H

#include <glm/gtc/matrix_transform.hpp>

class Light
{
public:
	Light();
	virtual ~Light();
	glm::vec3 pos;
	float intensity;
};

#endif
