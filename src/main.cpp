#include <cassert>
#include <cstring>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>


#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "Scene.h"
#include "Light.h"
#include "Shape.h"
#include "Image.h"
#include "Record.h"

// This allows you to skip the `std::` in front of C++ standard library
// functions. You can also say `using std::cout` to be more selective.
// You should never do this in a header file.
using namespace std;
using namespace glm;

shared_ptr<Scene> scene = make_shared<Scene>(); // scene that stores all the shapes and lights in a scene

// Generate a ray per pixel from the camera
vector<vector<vec3>> generateRays(vec3 camDir, vec3 camUp, float fovDegrees, int resolution) {
	int size = resolution;

	// Convert FOV from degrees to radians
	float fovRadians = radians(fovDegrees);

	// Calculate half width and height of the virtual plane
	float h = tanf(fovRadians / 2.0f);

	// Calculate the right vector
	vec3 right = normalize(cross(camDir, camUp));

	vector<vector<vec3>> rays(size, vector<vec3>(size));
	for (int y = 0; y < size; ++y) {
		for (int x = 0; x < size; ++x) {

			float ndcX = (2.0f * (x + 0.5f) / size) - 1.0f;
			float ndcY = (2.0f * (y + 0.5f) / size) - 1.0f;

			// Calculate direction vector of ray passing through current pixel
			vec3 rayDirection = normalize(camDir + ndcX * h * right + ndcY * h * camUp);

			// Normalize direction vector
			rayDirection = normalize(rayDirection);

			rays[x][y] = rayDirection;
		}
	}

	return rays;
}


// Compute the color of a ray 
int refCount;
vec3 computeRayColor(vec3 ray, vec3 origin, float t0, float t1) {
	shared_ptr<Record> rec = make_shared<Record>(); // variable to record the info of an object that is hit by a ray
	if (scene->hit(ray, origin, t0, t1, rec)) {
		vec3 color = rec->ka;
		vec3 n = normalize(rec->nor);

		// Calculate the shadows of an object generated by all lights of the scene
		for(int i = 0; i < scene->lights.size(); i++) {
			vec3 sray = normalize(scene->lights[i]->pos - rec->pos);
			shared_ptr<Record> srec = make_shared<Record>();
			float tlight = distance(rec->pos, scene->lights[i]->pos);
			if (!scene->hit(sray, rec->pos, 0.00001f, tlight, srec)) {
				vec3 eyeCam = normalize(origin - rec->pos);
				vec3 h = normalize(eyeCam + sray);

				float diffuse = glm::max(0.0f, dot(n, sray));
				float specular = glm::pow(glm::max(0.0f, dot(h, n)), rec->s);
				vec3 tempColor = scene->lights[i]->intensity * (rec->kd * diffuse + rec->ks * specular);
				color += tempColor;
			}
		}

		// Calculate reflection colors if an object is reflective
		if (rec->reflect != 0.0f && refCount < 6) {
			vec3 rray = 2.0f * dot(-ray, rec->nor) * rec->nor + ray;
			refCount++;
			return (1.0f - rec->reflect) * color + rec->reflect*computeRayColor(rray, rec->pos, 0.0001f, INFINITY);;
		}
		return color;
	}

	// if no object is hit return the background color.
	vec3 backColor = vec3(0.0f, 0.0f, 0.0f);
	return backColor;
}

int main(int argc, char **argv)
{
	if(argc < 4) {
		cout << "Not enough command line arguments given" << endl;
		return 0;
	}
	// SCENE
	int sceneIdx = atoi(argv[1]);
	// IMAGE SIZE
	int imgSize = atoi(argv[2]);
	// IMAGE FILENAME
	string filename(argv[3]);

	// Load geometry of bunny mesh
	vector<float> posBuf; // list of vertex positions
	vector<float> norBuf; // list of vertex normals
	vector<float> texBuf; // list of vertex texture coords

	vec3 mins = vec3(INFINITY, INFINITY, INFINITY);
	vec3 maxs = vec3(-INFINITY, -INFINITY, -INFINITY);

	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	string meshName = "../resources/bunny.obj";
	string errStr;
	bool rc = tinyobj::LoadObj(&attrib, &shapes, &materials, &errStr, meshName.c_str());
	if(!rc) {
		cerr << errStr << endl;
	} else {
		// Some OBJ files have different indices for vertex positions, normals,
		// and texture coordinates. For example, a cube corner vertex may have
		// three different normals. Here, we are going to duplicate all such
		// vertices.
		// Loop over shapes
		for(size_t s = 0; s < shapes.size(); s++) {
			// Loop over faces (polygons)
			size_t index_offset = 0;
			for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
				size_t fv = shapes[s].mesh.num_face_vertices[f];
				// Loop over vertices in the face.
				for(size_t v = 0; v < fv; v++) {
					// access to vertex
					tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+0]);
					if (posBuf.back() > maxs.x) {
						maxs.x = posBuf.back();
					}
					if (posBuf.back() < mins.x) {
						mins.x = posBuf.back();
					}
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+1]);
					if (posBuf.back() > maxs.y) {
						maxs.y = posBuf.back();
					}
					if (posBuf.back() < mins.y) {
						mins.y = posBuf.back();
					}
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+2]);
					if (posBuf.back() > maxs.z) {
						maxs.z = posBuf.back();
					}
					if (posBuf.back() < mins.z) {
						mins.z = posBuf.back();
					}
					if(!attrib.normals.empty()) {
						norBuf.push_back(attrib.normals[3*idx.normal_index+0]);
						norBuf.push_back(attrib.normals[3*idx.normal_index+1]);
						norBuf.push_back(attrib.normals[3*idx.normal_index+2]);
					}
					if(!attrib.texcoords.empty()) {
						texBuf.push_back(attrib.texcoords[2*idx.texcoord_index+0]);
						texBuf.push_back(attrib.texcoords[2*idx.texcoord_index+1]);
					}
				}
				index_offset += fv;
				// per-face material (IGNORE)
				shapes[s].mesh.material_ids[f];
			}
		}
	}
	
	//Initialize Camera parameters to be set by the scene
	vec3 camPos, camDir, camUp;
	float camFOV;

	// Set the scene depending on the inputted value
	if (sceneIdx == 1 || sceneIdx == 2) {
		// Set Camera Parameters
		camPos = vec3(0.0f, 0.0f, 5.0f);
		camDir = vec3(0.0f, 0.0f, -1.0f);
		camUp = vec3(0.0f, 1.0f, 0.0f);
		camFOV = 45.0f;

		// Red Sphere
		shared_ptr<Sphere> sphere1 = make_shared<Sphere>();
		sphere1->position = vec3(-0.5f, -1.0f, 1.0f);
		sphere1->diffuse = vec3(1.0f, 0.0f, 0.0f);
		sphere1->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere1->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere1->s = 100.0f;
		scene->shapes.push_back(sphere1);

		// Green Sphere
		shared_ptr<Sphere> sphere2 = make_shared<Sphere>();
		sphere2->position = vec3(0.5f, -1.0f, -1.0f);
		sphere2->diffuse = vec3(0.0f, 1.0f, 0.0f);
		sphere2->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere2->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere2->s = 100.0f;
		scene->shapes.push_back(sphere2);

		// Blue Sphere
		shared_ptr<Sphere> sphere3 = make_shared<Sphere>();
		sphere3->position = vec3(0.0f, 1.0f, 0.0f);
		sphere3->diffuse = vec3(0.0f, 0.0f, 1.0f);
		sphere3->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere3->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere3->s = 100.0f;
		scene->shapes.push_back(sphere3);

		// Create Lights
		shared_ptr<Light> light1 = make_shared<Light>();
		light1->intensity = 1.0f;
		light1->pos = vec3(-2.0f, 1.0f, 1.0f);
		scene->lights.push_back(light1);
	}
	else if (sceneIdx == 3) {
		// Set Camera Parameters
		camPos = vec3(0.0f, 0.0f, 5.0f);
		camDir = vec3(0.0f, 0.0f, -1.0f);
		camUp = vec3(0.0f, 1.0f, 0.0f);
		camFOV = 45.0f;

		// Red Ellipsoid
		shared_ptr<Ellipsoid> ellipsoid1 = make_shared<Ellipsoid>();
		ellipsoid1->position = vec3(0.5f, 0.0f, 0.5f);
		ellipsoid1->scale = vec3(0.5f, 0.6f, 0.2f);
		ellipsoid1->diffuse = vec3(1.0f, 0.0f, 0.0f);
		ellipsoid1->specular = vec3(1.0f, 1.0f, 0.5f);
		ellipsoid1->ambient = vec3(0.1f, 0.1f, 0.1f);
		ellipsoid1->s = 100.0f;
		scene->shapes.push_back(ellipsoid1);

		// Green Sphere
		shared_ptr<Sphere> sphere1 = make_shared<Sphere>();
		sphere1->position = vec3(-0.5f, 0.0f, -0.5f);
		sphere1->diffuse = vec3(0.0, 1.0f, 0.0f);
		sphere1->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere1->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere1->s = 100.0f;
		scene->shapes.push_back(sphere1);

		// Floor
		shared_ptr<Plane> plane1 = make_shared<Plane>();
		plane1->position = vec3(0.0f, -1.0f, 0.0f);
		plane1->rotate = vec3(0.0f, 1.0f, 0.0f);
		plane1->diffuse = vec3(1.0f, 1.0f, 1.0f);
		plane1->specular = vec3(0.0f, 0.0f, 0.0f);
		plane1->ambient = vec3(0.1f, 0.1f, 0.1f);
		plane1->s = 0.0f;
		scene->shapes.push_back(plane1);

		// Create Lights
		shared_ptr<Light> light1 = make_shared<Light>();
		light1->intensity = 0.5f;
		light1->pos = vec3(1.0f, 2.0f, 2.0f);
		scene->lights.push_back(light1);

		shared_ptr<Light> light2 = make_shared<Light>();
		light2->intensity = 0.5f;
		light2->pos = vec3(-1.0f, 2.0f, -1.0f);
		scene->lights.push_back(light2);

	}
	else if (sceneIdx == 4 || sceneIdx == 5) {
		// Set Camera Parameters
		camPos = vec3(0.0f, 0.0f, 5.0f);
		camDir = vec3(0.0f, 0.0f, -1.0f);
		camUp = vec3(0.0f, 1.0f, 0.0f);
		camFOV = 45.0f;

		// Red Sphere
		shared_ptr<Sphere> sphere1 = make_shared<Sphere>();
		sphere1->position = vec3(0.5f, -0.7f, 0.5f);
		sphere1->scale *= 0.3f;
		sphere1->diffuse = vec3(1.0f, 0.0f, 0.0f);
		sphere1->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere1->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere1->s = 100.0f;
		scene->shapes.push_back(sphere1);

		// Blue Sphere
		shared_ptr<Sphere> sphere2 = make_shared<Sphere>();
		sphere2->position = vec3(1.0f, -0.7f, 0.0f);
		sphere2->scale *= 0.3f;
		sphere2->diffuse = vec3(0.0f, 0.0f, 1.0f);
		sphere2->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere2->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere2->s = 100.0f;
		scene->shapes.push_back(sphere2);

		// Create Reflective Spheres
		shared_ptr<Sphere> rSphere1 = make_shared<Sphere>();
		rSphere1->position = vec3(-0.5f, 0.0f, -0.5f);
		rSphere1->reflect = 1.0f;
		scene->shapes.push_back(rSphere1);

		shared_ptr<Sphere> rSphere2 = make_shared<Sphere>();
		rSphere2->position = vec3(1.5f, 0.0f, -1.5f);
		rSphere2->reflect = 1.0f;
		scene->shapes.push_back(rSphere2);

		// Floor
		shared_ptr<Plane> plane1 = make_shared<Plane>();
		plane1->position = vec3(0.0f, -1.0f, 0.0f);
		plane1->rotate = vec3(0.0f, 1.0f, 0.0f);
		plane1->diffuse = vec3(1.0f, 1.0f, 1.0f);
		plane1->specular = vec3(0.0f, 0.0f, 0.0f);
		plane1->ambient = vec3(0.1f, 0.1f, 0.1f);
		plane1->s = 0.0f;
		scene->shapes.push_back(plane1);

		// Back Wall
		shared_ptr<Plane> plane2 = make_shared<Plane>();
		plane2->position = vec3(0.0f, 0.0f, -3.0f);
		plane2->rotate = vec3(0.0f, 0.0f, 1.0f);
		plane2->diffuse = vec3(1.0f, 1.0f, 1.0f);
		plane2->specular = vec3(0.0f, 0.0f, 0.0f);
		plane2->ambient = vec3(0.1f, 0.1f, 0.1f);
		plane2->s = 0.0f;
		scene->shapes.push_back(plane2);

		// Create Lights
		shared_ptr<Light> light1 = make_shared<Light>();
		light1->intensity = 0.5f;
		light1->pos = vec3(-1.0f, 2.0f, 1.0f);
		scene->lights.push_back(light1);

		shared_ptr<Light> light2 = make_shared<Light>();
		light2->intensity = 0.5f;
		light2->pos = vec3(0.5f, -0.5f, 0.0f);
		scene->lights.push_back(light2);

	}
	else if (sceneIdx == 6) {
		// Set Camera Parameters
		camPos = vec3(0.0f, 0.0f, 5.0f);
		camDir = vec3(0.0f, 0.0f, -1.0f);
		camUp = vec3(0.0f, 1.0f, 0.0f);
		camFOV = 45.0f;

		// Blue Bunny
		shared_ptr<TriMesh> bunny = make_shared<TriMesh>(posBuf, norBuf, mins, maxs);
		bunny->diffuse = vec3(0.0f, 0.0f, 1.0f);
		bunny->specular = vec3(1.0f, 1.0f, 0.5f);
		bunny->ambient = vec3(0.1f, 0.1f, 0.1f);
		bunny->s = 100.0f;
		scene->shapes.push_back(bunny);

		// Create Lights
		shared_ptr<Light> light1 = make_shared<Light>();
		light1->intensity = 1.0f;
		light1->pos = vec3(-1.0f, 1.0f, 1.0f);
		scene->lights.push_back(light1);
	}
	else if (sceneIdx == 7) {
		// Set Camera Parameters
		camPos = vec3(0.0f, 0.0f, 5.0f);
		camDir = vec3(0.0f, 0.0f, -1.0f);
		camUp = vec3(0.0f, 1.0f, 0.0f);
		camFOV = 45.0f;

		// Blue Bunny
		shared_ptr<TriMesh> bunny = make_shared<TriMesh>(posBuf, norBuf, mins, maxs);
		bunny->position = vec3(0.3f, -1.5f, 0.0f);
		bunny->rotate = vec3(20.0f * M_PI/ 180.0f, 0.0f, 0.0f);
		bunny->scale *= 1.5f;
		bunny->diffuse = vec3(0.0f, 0.0f, 1.0f);
		bunny->specular = vec3(1.0f, 1.0f, 0.5f);
		bunny->ambient = vec3(0.1f, 0.1f, 0.1f);
		bunny->s = 100.0f;
		scene->shapes.push_back(bunny);

		// Create Lights
		shared_ptr<Light> light1 = make_shared<Light>();
		light1->intensity = 1.0f;
		light1->pos = vec3(1.0f, 1.0f, 2.0f);
		scene->lights.push_back(light1);
	}
	else if (sceneIdx == 8) {
		// Set Camera Parameters
		camPos = vec3(-3.0f, 0.0f, 0.0f);
		camDir = vec3(1.0f, 0.0f, 0.0f);
		camUp = vec3(0.0f, 1.0f, 0.0f);
		camFOV = 60.0f;

		// Red Sphere
		shared_ptr<Sphere> sphere1 = make_shared<Sphere>();
		sphere1->position = vec3(-0.5f, -1.0f, 1.0f);
		sphere1->diffuse = vec3(1.0f, 0.0f, 0.0f);
		sphere1->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere1->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere1->s = 100.0f;
		scene->shapes.push_back(sphere1);

		// Green Sphere
		shared_ptr<Sphere> sphere2 = make_shared<Sphere>();
		sphere2->position = vec3(0.5f, -1.0f, -1.0f);
		sphere2->diffuse = vec3(0.0f, 1.0f, 0.0f);
		sphere2->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere2->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere2->s = 100.0f;
		scene->shapes.push_back(sphere2);

		// Blue Sphere
		shared_ptr<Sphere> sphere3 = make_shared<Sphere>();
		sphere3->position = vec3(0.0f, 1.0f, 0.0f);
		sphere3->diffuse = vec3(0.0f, 0.0f, 1.0f);
		sphere3->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere3->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere3->s = 100.0f;
		scene->shapes.push_back(sphere3);

		// Create Lights
		shared_ptr<Light> light1 = make_shared<Light>();
		light1->intensity = 1.0f;
		light1->pos = vec3(-2.0f, 1.0f, 1.0f);
		scene->lights.push_back(light1);
	}
	else if (sceneIdx == 9) {
		// Set Camera Parameters
		camPos = vec3(0.0f, 0.0f, 5.0f);
		camDir = vec3(0.0f, 0.0f, -1.0f);
		camUp = vec3(0.0f, 1.0f, 0.0f);
		camFOV = 45.0f;

		// Red Sphere
		shared_ptr<Sphere> sphere1 = make_shared<Sphere>();
		sphere1->position = vec3(0.5f, -0.7f, 0.5f);
		sphere1->scale *= 0.3f;
		sphere1->diffuse = vec3(1.0f, 0.0f, 0.0f);
		sphere1->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere1->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere1->s = 100.0f;
		scene->shapes.push_back(sphere1);

		// Blue Sphere
		shared_ptr<Sphere> sphere2 = make_shared<Sphere>();
		sphere2->position = vec3(1.0f, -0.7f, 0.0f);
		sphere2->scale *= 0.3f;
		sphere2->diffuse = vec3(0.0f, 0.0f, 1.0f);
		sphere2->specular = vec3(1.0f, 1.0f, 0.5f);
		sphere2->ambient = vec3(0.1f, 0.1f, 0.1f);
		sphere2->s = 100.0f;
		scene->shapes.push_back(sphere2);

		// Create Reflective Spheres
		shared_ptr<Sphere> rSphere1 = make_shared<Sphere>();
		rSphere1->position = vec3(-0.5f, 0.0f, -0.5f);
		rSphere1->diffuse = vec3(0.7f, 0.0f, 0.0f);
		rSphere1->specular = vec3(0.5f, 1.0f, 1.0f);
		rSphere1->ambient = vec3(0.2f, 0.1f, 0.1f);
		rSphere1->s = 100.0f;
		rSphere1->reflect = 0.3f;
		scene->shapes.push_back(rSphere1);

		shared_ptr<Sphere> rSphere2 = make_shared<Sphere>();
		rSphere2->position = vec3(1.5f, 0.0f, -1.5f);
		rSphere2->diffuse = vec3(0.0f, 0.0f, 0.7f);
		rSphere2->specular = vec3(1.0f, 1.0f, 0.5f);
		rSphere2->ambient = vec3(0.1f, 0.1f, 0.2f);
		rSphere2->s = 100.0f;
		rSphere2->reflect = 0.3f;
		scene->shapes.push_back(rSphere2);

		// Floor
		shared_ptr<Plane> plane1 = make_shared<Plane>();
		plane1->position = vec3(0.0f, -1.0f, 0.0f);
		plane1->rotate = vec3(0.0f, 1.0f, 0.0f);
		plane1->diffuse = vec3(1.0f, 1.0f, 1.0f);
		plane1->specular = vec3(0.0f, 0.0f, 0.0f);
		plane1->ambient = vec3(0.1f, 0.1f, 0.1f);
		plane1->s = 0.0f;
		scene->shapes.push_back(plane1);

		// Back Wall
		shared_ptr<Plane> plane2 = make_shared<Plane>();
		plane2->position = vec3(0.0f, 0.0f, -3.0f);
		plane2->rotate = vec3(0.0f, 0.0f, 1.0f);
		plane2->diffuse = vec3(1.0f, 1.0f, 1.0f);
		plane2->specular = vec3(0.0f, 0.0f, 0.0f);
		plane2->ambient = vec3(0.1f, 0.1f, 0.1f);
		plane2->s = 0.0f;
		scene->shapes.push_back(plane2);

		// Create Lights
		shared_ptr<Light> light1 = make_shared<Light>();
		light1->intensity = 0.5f;
		light1->pos = vec3(-1.0f, 2.0f, 1.0f);
		scene->lights.push_back(light1);

		shared_ptr<Light> light2 = make_shared<Light>();
		light2->intensity = 0.5f;
		light2->pos = vec3(0.5f, -0.5f, 0.0f);
		scene->lights.push_back(light2);

		}

	// Generate all rays from the camera
	vector<vector<vec3>> rays = generateRays(camDir, camUp, camFOV, imgSize);

	// Create the image
	auto image = make_shared<Image>(imgSize, imgSize);
	for(int y = 0; y < imgSize; ++y) {
		for (int x = 0; x < imgSize; ++x) {
			// Reset reflection count for next ray being shot
			refCount = 0;

			// Obtain color of a ray
			vec3 col = computeRayColor(rays[x][y],camPos, 0.0, INFINITY);

			col *= 255;
			// Cap the colors to a range of [0, 255]
			col.r = glm::max(0.0f, glm::min(255.0f, col.r));
			col.g = glm::max(0.0f, glm::min(255.0f, col.g));
			col.b = glm::max(0.0f, glm::min(255.0f, col.b));
			image->setPixel(x, y, col.r, col.g, col.b);
		}
	}

	// Write image to file
	image->writeToFile(filename);

	return 0;
}