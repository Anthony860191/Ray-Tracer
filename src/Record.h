#ifndef RECORD_H
#define RECORD_H

#include <glm/gtc/matrix_transform.hpp>

struct Record {
	glm::vec3 pos;
	glm::vec3 nor;
	glm::vec3 ka;
	glm::vec3 kd;
	glm::vec3 ks;
	float s;
	float reflect;
	
};

#endif // RECORD_H