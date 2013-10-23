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
	for (size_t i = 0; i < boundingbox_ptrs.size(); i++) {
		delete boundingbox_ptrs[i];
	}
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
	/*
	size_t i;
	for (i = 0; i < mesh->num_triangles(); i++) {
		if (boundingbox_ptrs[i]->hit(ray, start, end, model_index, record_ptr))
			break;
	}
	if (i == mesh->num_triangles())
		return false;
		*/
	Ray transformed_ray = ray.transform(this->invMat);
	real_t beta;
	real_t gamma;
	bool hit_result = mesh->hit(transformed_ray, start, end, model_index, record_ptr, beta, gamma);

	if (hit_result && record_ptr != NULL) {
		record_ptr->hit = true;
		record_ptr->hit_point = ray.e + record_ptr->time * ray.d;
		record_ptr->tex_coord = this->material->clap_texture(record_ptr->tex_coord);
		record_ptr->shade_factors.texture = this->material->get_texture_pixel(record_ptr->tex_coord);
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

std::vector<Boundingbox*> Model::get_boundingboxs() const {
	return boundingbox_ptrs;
}

void Model::construct_boundingbox() {
	boundingbox_ptrs.reserve(mesh->num_triangles());
	for (size_t i = 0; i < mesh->num_triangles(); i++) {
		Boundingbox* new_box = new Boundingbox();
		new_box->mat = mat;
		new_box->minPoint = mesh->min_max_points[i].first;
		new_box->maxPoint = mesh->min_max_points[i].second;
		new_box->model_index = i;
		new_box->geometry = this;
		new_box->construct_boundingbox();
		new_box->isLoose = true;
		boundingbox_ptrs.push_back(new_box);
	}
}

} /* _462 */
