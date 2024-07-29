#ifndef HITINFO_H
#define HITINFO_H

#include <glm/gtc/matrix_transform.hpp>

struct hitInfo {
	glm::vec3 position;
	glm::vec3 normal;
	float distance;
};

#endif // HITINFO_H