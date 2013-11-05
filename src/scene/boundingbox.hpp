/*
 * boundingbox.hpp
 *
 *  Created on: Oct 20, 2013
 *      Author: Chenxi Liu chenxil
 */

#ifndef BOUNDINGBOX_HPP_
#define BOUNDINGBOX_HPP_

#include "scene/scene.hpp"
#include "math/vector.hpp"

namespace _462 {

// The axis aligned bounding box.
class Boundingbox: public Geometry {
public:
	Boundingbox()
	: maxPoint(Vector3(0.0, 0.0, 0.0)), minPoint(Vector3(0.0, 0.0, 0.0)),
	  model_index(-1), geometry(NULL), isLoose(false)
	{ }
	Boundingbox(const Vector3 maxPoint, const Vector3 minPoint,
			const size_t model_index, Geometry* geometry)
	: maxPoint(maxPoint), minPoint(minPoint), model_index(model_index), geometry(geometry), isLoose(false)
	{ }
	virtual ~Boundingbox();

	// Two corners that define the bounding box.
	Vector3 maxPoint;
	Vector3 minPoint;

	// Since a model may have multiple bounding box for triangles,
	// we use bounding boxes to find the models and the triangles
	// inside.
	size_t model_index;
	Geometry* geometry;


	// The bounding box is constructed in the object coordinate.
	// We need to add another axis aligned bounding box outside
	// the original one.
	bool isLoose;

	virtual void render() const;

	// Hit function
	virtual bool hit(const Ray ray, const real_t start, const real_t end,
			const unsigned int model_index, HitRecord* record_ptr);

	virtual size_t num_models() const;
	virtual std::vector<Boundingbox*> get_boundingboxs() const;

	virtual void construct_boundingbox();

	virtual bool is_refractive() const;

	Vector3 generate_rand_point() const;

};
}

#endif /* BOUNDINGBOX_HPP_ */
