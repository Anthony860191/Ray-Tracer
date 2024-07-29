#include "Scene.h"

using namespace glm;
using namespace std;

Scene::Scene() :
	lights(),
	shapes()
{
}

bool Scene::hit(vec3 ray, vec3 origin, float t0, float t1, shared_ptr<Record> rec)
{
	bool hit = false;
	float t = INFINITY;

	for (int i = 0; i < shapes.size(); i++) {
		hitInfo temp = shapes[i]->intersect(ray, origin, t0, t1);
		if (temp.distance < t) {
			t = temp.distance;
			rec->pos = temp.position;
			rec->nor = temp.normal;
			rec->ka = shapes[i]->ambient;
			rec->kd = shapes[i]->diffuse;
			rec->ks = shapes[i]->specular;
			rec->s = shapes[i]->s;
			rec->reflect = shapes[i]->reflect;
			hit = true;
		}
	}
	return hit;
}

