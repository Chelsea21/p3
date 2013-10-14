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
	bool hit_result = mesh->hit(transformed_ray, start, end, model_index, record_ptr);

	if (record_ptr != NULL) {
		record_ptr->material_ptr = this->material;
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

} /* _462 */
