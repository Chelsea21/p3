/*
 * boundingbox.cpp
 *
 *  Created on: Oct 20, 2013
 *      Author: Chenxi Liu chenxil
 */

#include "scene/boundingbox.hpp"

namespace _462 {
Boundingbox::~Boundingbox() {
}

void Boundingbox::render() const {
	std::cerr << "Cannot render boundingbox." << std::endl;
}

/**
 * Check whether the ray hits the bounding box.
 * @param ray 			the ray.
 * @param start			the t parameter for the starting point.
 * @param end			the t parameter for the ending point.
 * @param model_index	unused.
 * @param record_ptr	the pointer of record struct.
 */
bool Boundingbox::hit(const Ray ray, const real_t start, const real_t end,
			const unsigned int model_index, HitRecord* record_ptr) {
	(void) model_index;
	if (!isLoose)
		construct_boundingbox();

	// Check the plane perpendicular to x-axis.
	real_t denominator_x = 1 / ray.d.x;
	real_t tmin;
	real_t tmax;
	if (denominator_x >= 0) {
		tmin = denominator_x * (minPoint.x - ray.e.x);
		tmax = denominator_x * (maxPoint.x - ray.e.x);
	}
	else {
		tmin = denominator_x * (maxPoint.x - ray.e.x);
		tmax = denominator_x * (minPoint.x - ray.e.x);
	}

	// Check the plane perpendicular to y-axis.
	real_t denominator_y = 1 / ray.d.y;
	real_t tmin_y;
	real_t tmax_y;
	if (denominator_y >= 0) {
		tmin_y = denominator_y * (minPoint.y - ray.e.y);
		tmax_y = denominator_y * (maxPoint.y - ray.e.y);
	}
	else {
		tmin_y = denominator_y * (maxPoint.y - ray.e.y);
		tmax_y = denominator_y * (minPoint.y - ray.e.y);
	}

	// If ray hits the plane perpendicular to x-axis first, then
	// the ray missed the box.
	if (tmin > tmax_y || tmax < tmin_y)
		return false;
	tmin = (tmin > tmin_y) ? tmin : tmin_y;
	tmax = (tmax < tmax_y) ? tmax : tmax_y;

	// Do the same checking to the plane perpendicular to z-axis.
	real_t denominator_z = 1 / ray.d.z;
	real_t tmin_z;
	real_t tmax_z;
	if (denominator_z >= 0) {
		tmin_z = denominator_z * (minPoint.z - ray.e.z);
		tmax_z = denominator_z * (maxPoint.z - ray.e.z);
	}
	else {
		tmin_z = denominator_z * (maxPoint.z - ray.e.z);
		tmax_z = denominator_z * (minPoint.z - ray.e.z);
	}
	if (tmin > tmax_z || tmax < tmin_z)
		return false;
	tmin = (tmin > tmin_z) ? tmin : tmin_z;
	tmax = (tmax < tmax_z) ? tmax : tmax_z;

	if (tmin > end || tmax < start)
		return false;

	if (record_ptr != NULL) {
		record_ptr->time = tmin;
	}

	return true;
}

/**
 * Returns the number of objects inside.
 */
size_t Boundingbox::num_models() const {
	return 1;
}

/**
 * Returns a list of pointers of bounding boxes. For Boundingbox class,
 * the list contains this pointer.
 */
std::vector<Boundingbox*> Boundingbox::get_boundingboxs() const {
	return std::vector<Boundingbox*>(1, const_cast<Boundingbox*>(this));
}

/**
 * Constructs the axis aligned bounding box according to the
 * bounding box in object coordinate.
 */
void Boundingbox::construct_boundingbox() {
	std::vector<Vector3> vertices(8);
	vertices[0] = Vector3(maxPoint.x, minPoint.y, minPoint.z);
	vertices[1] = Vector3(minPoint.x, maxPoint.y, minPoint.z);
	vertices[2] = Vector3(maxPoint.x, maxPoint.y, minPoint.z);
	vertices[3] = Vector3(minPoint.x, maxPoint.y, maxPoint.z);
	vertices[4] = Vector3(minPoint.x, minPoint.y, maxPoint.z);
	vertices[5] = Vector3(maxPoint.x, minPoint.y, maxPoint.z);
	vertices[6] = minPoint;
	vertices[7] = maxPoint;
	real_t inf = std::numeric_limits<double>::infinity();
	Vector3 newMinPoint(inf, inf, inf);
	Vector3 newMaxPoint(-inf, -inf, -inf);

	for (size_t i = 0; i < vertices.size(); i++) {
		Vector3 transformed = this->mat.transform_point(vertices[i]);
		for (size_t j = 0; j < 3; j++) {
			newMinPoint[j] = (transformed[j] < newMinPoint[j]) ? transformed[j] : newMinPoint[j];
			newMaxPoint[j] = (transformed[j] > newMaxPoint[j]) ? transformed[j] : newMaxPoint[j];
		}
	}

	minPoint = newMinPoint;
	maxPoint = newMaxPoint;
}

bool Boundingbox::is_refractive() const {
	return false;
}

Vector3 Boundingbox::generate_rand_point() const {
	Vector3 result;
	int axis = rand() % 3;
	real_t min_max = ((double) rand() / (RAND_MAX));
	result[axis] = (min_max < 0.5) ? minPoint[axis] : maxPoint[axis];
	Vector3 face_point;
	for (size_t i = 0; i < 3; i++) {
		if (i != axis)
			face_point[i] = (min_max >= 0.5) ? minPoint[i] : maxPoint[i];
		else
			face_point[i] = (min_max < 0.5) ? minPoint[i] : maxPoint[i];
	}
	for (size_t i = 0; i < 3; i++) {
		if (i != axis) {
			real_t inside = 1. * random();
			if (min_max < 0.5)
				result[i] = (face_point[i] - minPoint[i]) * inside + minPoint[i];
			else
				result[i] = (maxPoint[i] - face_point[i]) * inside + face_point[i];
		}
	}

	return result;
}

}

