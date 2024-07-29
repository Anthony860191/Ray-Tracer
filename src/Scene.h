#pragma  once
#ifndef SCENE_H
#define SCENE_H

#include "Light.h"
#include "Shape.h"
#include "hitInfo.h"
#include "Record.h"

#include <vector>
#include <glm/gtc/matrix_transform.hpp>


class Scene
{
public:
	Scene();
	bool hit(glm::vec3 ray, glm::vec3 origin, float t0, float t1, std::shared_ptr<Record> rec);

	std::vector<std::shared_ptr<Light>> lights;
	std::vector<std::shared_ptr<Shape>> shapes;
};

#endif
