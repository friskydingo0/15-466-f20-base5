#include "MovingAI.hpp"
#include <glm/gtc/random.hpp>

BoidAI::BoidAI(){
	velocity = glm::vec3(glm::diskRand(0.01f), 0.0f);
	acceleration = glm::vec3(0.0f, 0.0f, 0.0f);
}

void BoidAI::update(float elapsed) {
	transform->position += velocity * elapsed;
	velocity += acceleration * elapsed;
}

void BoidAI::flock(const std::vector<BoidAI> &boids) {
	//acceleration += separate(boids);	// Something weird happens if I turn this on.
	acceleration += align(boids);
	acceleration += cohesion(boids);
}

glm::vec3 BoidAI::separate(const std::vector<BoidAI> &boids) {
	
	float range = 0.05f;
	int boids_in_range = 0;
	glm::vec3 steering = glm::vec3(0.0f, 0.0f, 0.0f);
	for (auto &&boid : boids)
	{
		if (&boid == this) continue;
		
		float d = glm::length(transform->position - boid.transform->position);
		if (d < range)
		{
			steering += (transform->position - boid.transform->position)/d;
			steering += boid.velocity;
			boids_in_range++;
		}
	}
	if (boids_in_range > 0)
	{
		steering = steering / (float)boids_in_range;
		steering -= transform->position;
		steering = glm::normalize(steering) * 0.2f;
		steering -= velocity;
	}

	return glm::normalize(steering) * std::min(glm::length(steering), 0.1f);
}

glm::vec3 BoidAI::align(const std::vector<BoidAI> &boids) {
	
	float range = 10.0f;
	int boids_in_range = 0;
	glm::vec3 steering = glm::vec3(0.0f, 0.0f, 0.0f);
	for (auto &&boid : boids)
	{
		if (&boid == this) continue;
		
		float d = glm::length(transform->position - boid.transform->position);
		if (d < range)
		{
			steering += boid.velocity;
			boids_in_range++;
		}
	}
	if (boids_in_range > 0)
	{
		steering = steering / (float)boids_in_range;
		steering = glm::normalize(steering) * 0.2f;
		steering -= velocity;
	}

	return glm::normalize(steering) * std::min(glm::length(steering), 0.2f);;
}

glm::vec3 BoidAI::cohesion(const std::vector<BoidAI> &boids) {
	
	float range = 10.0f;
	int boids_in_range = 0;
	glm::vec3 steering = glm::vec3(0.0f, 0.0f, 0.0f);
	for (auto &&boid : boids)
	{
		if (&boid == this) continue;
		
		float d = glm::length(transform->position - boid.transform->position);
		if (d < range)
		{
			steering += boid.velocity;
			boids_in_range++;
		}
	}
	if (boids_in_range > 0)
	{
		steering = steering / (float)boids_in_range;
		steering -= transform->position;
		steering = glm::normalize(steering) * 0.2f;
		steering -= velocity;
		
	}

	return glm::normalize(steering) * std::min(glm::length(steering), 0.2f);;
}

BoidAI::~BoidAI() {

}