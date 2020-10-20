#include "WalkMesh.hpp"

#include "read_write_chunk.hpp"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

WalkMesh::WalkMesh(std::vector< glm::vec3 > const &vertices_, std::vector< glm::vec3 > const &normals_, std::vector< glm::uvec3 > const &triangles_)
	: vertices(vertices_), normals(normals_), triangles(triangles_) {

	//construct next_vertex map (maps each edge to the next vertex in the triangle):
	next_vertex.reserve(triangles.size()*3);
	auto do_next = [this](uint32_t a, uint32_t b, uint32_t c) {
		auto ret = next_vertex.insert(std::make_pair(glm::uvec2(a,b), c));
		assert(ret.second);
	};
	for (auto const &tri : triangles) {
		do_next(tri.x, tri.y, tri.z);
		do_next(tri.y, tri.z, tri.x);
		do_next(tri.z, tri.x, tri.y);
	}

	//DEBUG: are vertex normals consistent with geometric normals?
	for (auto const &tri : triangles) {
		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];
		glm::vec3 out = glm::normalize(glm::cross(b-a, c-a));

		float da = glm::dot(out, normals[tri.x]);
		float db = glm::dot(out, normals[tri.y]);
		float dc = glm::dot(out, normals[tri.z]);

		assert(da > 0.1f && db > 0.1f && dc > 0.1f);
	}
}

//project pt to the plane of triangle a,b,c and return the barycentric weights of the projected point:
glm::vec3 barycentric_weights(glm::vec3 const &a, glm::vec3 const &b, glm::vec3 const &c, glm::vec3 const &pt) {

	// I tried to implement on my own by following this tutorial - https://youtu.be/qz3Q3v84k9Y
	// But something was not adding up and it led to the barycentric coordinate being wrongly computed. Hence used the following code.

	// Get barycentric coords - from https://gamedev.stackexchange.com/a/23745
	glm::vec3 v0 = b - a, v1 = c - a, v2 = pt - a;
	float d00 = glm::dot(v0, v0);
	float d01 = glm::dot(v0, v1);
	float d11 = glm::dot(v1, v1);
	float d20 = glm::dot(v2, v0);
	float d21 = glm::dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	float v = (d11 * d20 - d01 * d21) / denom;
	float w = (d00 * d21 - d01 * d20) / denom;
	float u = 1.0f - v - w;

	return glm::vec3(u, v, w);
}

WalkPoint WalkMesh::nearest_walk_point(glm::vec3 const &world_point) const {
	assert(!triangles.empty() && "Cannot start on an empty walkmesh");

	WalkPoint closest;
	float closest_dis2 = std::numeric_limits< float >::infinity();

	for (auto const &tri : triangles) {
		//find closest point on triangle:

		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];

		//get barycentric coordinates of closest point in the plane of (a,b,c):
		glm::vec3 coords = barycentric_weights(a,b,c, world_point);

		//is that point inside the triangle?
		if (coords.x >= 0.0f && coords.y >= 0.0f && coords.z >= 0.0f) {
			//yes, point is inside triangle.
			float dis2 = glm::length2(world_point - to_world_point(WalkPoint(tri, coords)));
			if (dis2 < closest_dis2) {
				closest_dis2 = dis2;
				closest.indices = tri;
				closest.weights = coords;
			}
		} else {
			//check triangle vertices and edges:
			auto check_edge = [&world_point, &closest, &closest_dis2, this](uint32_t ai, uint32_t bi, uint32_t ci) {
				glm::vec3 const &a = vertices[ai];
				glm::vec3 const &b = vertices[bi];

				//find closest point on line segment ab:
				float along = glm::dot(world_point-a, b-a);
				float max = glm::dot(b-a, b-a);
				glm::vec3 pt;
				glm::vec3 coords;
				if (along < 0.0f) {
					pt = a;
					coords = glm::vec3(1.0f, 0.0f, 0.0f);
				} else if (along > max) {
					pt = b;
					coords = glm::vec3(0.0f, 1.0f, 0.0f);
				} else {
					float amt = along / max;
					pt = glm::mix(a, b, amt);
					coords = glm::vec3(1.0f - amt, amt, 0.0f);
				}

				float dis2 = glm::length2(world_point - pt);
				if (dis2 < closest_dis2) {
					closest_dis2 = dis2;
					closest.indices = glm::uvec3(ai, bi, ci);
					closest.weights = coords;
				}
			};
			check_edge(tri.x, tri.y, tri.z);
			check_edge(tri.y, tri.z, tri.x);
			check_edge(tri.z, tri.x, tri.y);
		}
	}
	assert(closest.indices.x < vertices.size());
	assert(closest.indices.y < vertices.size());
	assert(closest.indices.z < vertices.size());
	return closest;
}

// code taken from https://github.com/Dmcdominic/15-466-f20-base5/blob/master/WalkMesh.cpp with permission
void WalkMesh::walk_in_triangle(WalkPoint const &start, glm::vec3 const &step, WalkPoint *end_, float *time_) const {
	assert(end_);
	auto &end = *end_;
	assert(time_);
	auto &time = *time_;

	//if no edge is crossed, event will just be taking the whole step:
	time = 1.0f;
	end = start;

	//project 'step' into a barycentric-coordinates direction:
	glm::vec3 step_weights;
	{ 
		glm::vec3 const& a = vertices[start.indices.x];
		glm::vec3 const& b = vertices[start.indices.y];
		glm::vec3 const& c = vertices[start.indices.z];
		/*glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
		float dot = glm::dot(step, normal);
		step_coords = step - dot * normal;*/

		glm::vec3 start_wrld_pt = WalkMesh::to_world_point(start);
		glm::vec3 end_wrld_pt = start_wrld_pt + step;
		end.weights = barycentric_weights(a, b, c, end_wrld_pt);

		step_weights = end.weights - start.weights;
	}
	
	//figure out which edge (if any) is crossed first.
	// set time and end appropriately.
	float t_x_0 = (step_weights.x == 0) ? (-1.0f) : (-start.weights.x / step_weights.x);
	float t_y_0 = (step_weights.y == 0) ? (-1.0f) : (-start.weights.y / step_weights.y);
	float t_z_0 = (step_weights.z == 0) ? (-1.0f) : (-start.weights.z / step_weights.z);
	float t_min = 1.0f;
	if (t_x_0 > 0.0f && t_x_0 < t_min) t_min = t_x_0;
	if (t_y_0 > 0.0f && t_y_0 < t_min) t_min = t_y_0;
	if (t_z_0 > 0.0f && t_z_0 < t_min) t_min = t_z_0;

	if (t_min < 1.0f && t_min > 0.0f) {
		time = t_min;
		end.weights = start.weights + step_weights * t_min;
		if (t_x_0 == t_min) {
			end.indices = glm::uvec3(end.indices.y, end.indices.z, end.indices.x);
			float weight_sum = end.weights.y + end.weights.z;
			end.weights = glm::vec3(end.weights.y / weight_sum, end.weights.z / weight_sum, 0.0f);
		} else if (t_y_0 == t_min) {
			end.indices = glm::uvec3(end.indices.z, end.indices.x, end.indices.y);
			float weight_sum = end.weights.z + end.weights.x;
			end.weights = glm::vec3(end.weights.z / weight_sum, end.weights.x / weight_sum, 0.0f);
		} else if (t_z_0 == t_min) {
			float weight_sum = end.weights.x + end.weights.y;
			end.weights = glm::vec3(end.weights.x / weight_sum, end.weights.y / weight_sum, 0.0f);
		}
	}
	//Remember: our convention is that when a WalkPoint is on an edge,
	// then wp.weights.z == 0.0f (so will likely need to re-order the indices)
}

// code taken from https://github.com/Dmcdominic/15-466-f20-base5/blob/master/WalkMesh.cpp with permission
bool WalkMesh::cross_edge(WalkPoint const &start, WalkPoint *end_, glm::quat *rotation_) const {
	assert(start.weights.z == 0.0f);
	assert(start.indices.x <= vertices.size() && start.indices.y <= vertices.size() && start.indices.z <= vertices.size());

	assert(end_);
	auto &end = *end_;

	assert(rotation_);
	auto &rotation = *rotation_;

	// check if edge (start.indices.x, start.indices.y) has a triangle on the other side:
	end = start;
	auto f = next_vertex.find(glm::uvec2(start.indices.y, start.indices.x));
	if (f == next_vertex.end()) {
		rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); //identity quat (wxyz init order)
		return false;
	}
	// if there is another triangle, set end's weights and indicies on that triangle:
	end.indices = glm::uvec3(start.indices.y, start.indices.x, f->second);
	end.weights = glm::vec3(start.weights.y, start.weights.x, 0.0f);

	// compute rotation that takes starting triangle's normal to ending triangle's normal:
	// see 'glm::rotation' in the glm/gtx/quaternion.hpp header (line 160)
	glm::vec3 orig = glm::normalize(glm::cross(vertices[start.indices.y] - vertices[start.indices.x], vertices[start.indices.z] - vertices[start.indices.y]));
	glm::vec3 dest = glm::normalize(glm::cross(vertices[end.indices.y] - vertices[end.indices.x], vertices[end.indices.z] - vertices[end.indices.y]));
	rotation = glm::rotation(orig, dest);

	//return 'true' if there was another triangle, 'false' otherwise:
	return true;
}


WalkMeshes::WalkMeshes(std::string const &filename) {
	std::ifstream file(filename, std::ios::binary);

	std::vector< glm::vec3 > vertices;
	read_chunk(file, "p...", &vertices);

	std::vector< glm::vec3 > normals;
	read_chunk(file, "n...", &normals);

	std::vector< glm::uvec3 > triangles;
	read_chunk(file, "tri0", &triangles);

	std::vector< char > names;
	read_chunk(file, "str0", &names);

	struct IndexEntry {
		uint32_t name_begin, name_end;
		uint32_t vertex_begin, vertex_end;
		uint32_t triangle_begin, triangle_end;
	};

	std::vector< IndexEntry > index;
	read_chunk(file, "idxA", &index);

	if (file.peek() != EOF) {
		std::cerr << "WARNING: trailing data in walkmesh file '" << filename << "'" << std::endl;
	}

	//-----------------

	if (vertices.size() != normals.size()) {
		throw std::runtime_error("Mis-matched position and normal sizes in '" + filename + "'");
	}

	for (auto const &e : index) {
		if (!(e.name_begin <= e.name_end && e.name_end <= names.size())) {
			throw std::runtime_error("Invalid name indices in index of '" + filename + "'");
		}
		if (!(e.vertex_begin <= e.vertex_end && e.vertex_end <= vertices.size())) {
			throw std::runtime_error("Invalid vertex indices in index of '" + filename + "'");
		}
		if (!(e.triangle_begin <= e.triangle_end && e.triangle_end <= triangles.size())) {
			throw std::runtime_error("Invalid triangle indices in index of '" + filename + "'");
		}

		//copy vertices/normals:
		std::vector< glm::vec3 > wm_vertices(vertices.begin() + e.vertex_begin, vertices.begin() + e.vertex_end);
		std::vector< glm::vec3 > wm_normals(normals.begin() + e.vertex_begin, normals.begin() + e.vertex_end);

		//remap triangles:
		std::vector< glm::uvec3 > wm_triangles; wm_triangles.reserve(e.triangle_end - e.triangle_begin);
		for (uint32_t ti = e.triangle_begin; ti != e.triangle_end; ++ti) {
			if (!( (e.vertex_begin <= triangles[ti].x && triangles[ti].x < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].y && triangles[ti].y < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].z && triangles[ti].z < e.vertex_end) )) {
				throw std::runtime_error("Invalid triangle in '" + filename + "'");
			}
			wm_triangles.emplace_back(
				triangles[ti].x - e.vertex_begin,
				triangles[ti].y - e.vertex_begin,
				triangles[ti].z - e.vertex_begin
			);
		}
		
		std::string name(names.begin() + e.name_begin, names.begin() + e.name_end);

		auto ret = meshes.emplace(name, WalkMesh(wm_vertices, wm_normals, wm_triangles));
		if (!ret.second) {
			throw std::runtime_error("WalkMesh with duplicated name '" + name + "' in '" + filename + "'");
		}

	}
}

WalkMesh const &WalkMeshes::lookup(std::string const &name) const {
	auto f = meshes.find(name);
	if (f == meshes.end()) {
		throw std::runtime_error("WalkMesh with name '" + name + "' not found.");
	}
	return f->second;
}
