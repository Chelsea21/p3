/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "scene/model.hpp"
#include "scene/material.hpp"
#include "application/opengl.hpp"
#include "scene/triangle.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>

namespace _462 {

Model::Model() :
		mesh(0), material(0) {
}
Model::~Model() {
}

void Model::render() const {
	if (!mesh)
		return;
	if (material)
		material->set_gl_state();
	mesh->render();
	if (material)
		material->reset_gl_state();
}

bool Model::hit(const Ray ray, const real_t start, const real_t end,
		const unsigned int model_index, HitRecord* record_ptr) {
	Ray transformed_ray = ray.transform(this->invMat);
	real_t beta;
	real_t gamma;
	bool hit_result = mesh->hit(transformed_ray, start, end, model_index, record_ptr, beta, gamma);

	if (hit_result && record_ptr != NULL) {
		record_ptr->hit_point = ray.e + record_ptr->time * ray.d;
		record_ptr->material_ptr = this->material;
		record_ptr->tex_coord = record_ptr->material_ptr->clap_texture(record_ptr->tex_coord);
		//std::cout << record_ptr->tex_coord.x << "\t" << record_ptr->tex_coord.y << std::endl;
		record_ptr->normal = normalize(this->normMat * record_ptr->normal);
		record_ptr->shade_factors.ambient = this->material->ambient;
		record_ptr->shade_factors.diffuse = this->material->diffuse;
		record_ptr->shade_factors.refractive_index = this->material->refractive_index;
		record_ptr->shade_factors.shininess = this->material->shininess;
		record_ptr->shade_factors.specular = this->material->specular;
	}

	return hit_result;
}

size_t Model::num_models() const {
	return mesh->num_triangles();
}

Boundingbox* const Model::get_boundingbox() const {
	return &boundingbox;
}

void Model::construct_boundingbox() {
	boundingbox.mat = mat;
	boundingbox.minPoint = mesh->minPoint;
	boundingbox.maxPoint = mesh->maxPoint;
	boundingbox.construct_boundingbox();
}

} /* _462 */
