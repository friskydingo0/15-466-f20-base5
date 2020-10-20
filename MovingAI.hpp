#include "Scene.hpp"
#include "WalkMesh.hpp"

// Based on https://youtu.be/mhjuuHl6qHM with some very obvious bugs
struct BoidAI {
	BoidAI();
	virtual ~BoidAI();

	Scene::Transform *transform = nullptr;

	glm::vec3 velocity;
	glm::vec3 acceleration;
	
	// Boid methods
	void update(float elapsed);
	void flock(const std::vector<BoidAI> &boids);
	glm::vec3 align(const std::vector<BoidAI> &boids);
	glm::vec3 separate(const std::vector<BoidAI> &boids);
	glm::vec3 cohesion(const std::vector<BoidAI> &boids);
};