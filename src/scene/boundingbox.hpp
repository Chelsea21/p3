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

class Boundingbox: public Geometry {
public:
	Boundingbox()
	: maxPoint(Vector3(0.0, 0.0, 0.0)), minPoint(Vector3(0.0, 0.0, 0.0)), isLoose(false)
	{ }
	Boundingbox(const Vector3 maxPoint, const Vector3 minPoint)
	: maxPoint(maxPoint), minPoint(minPoint), isLoose(false)
	{ }
	virtual ~Boundingbox();

	Vector3 maxPoint;
	Vector3 minPoint;

	bool isLoose;

	virtual void render() const;

	// Hit function
	virtual bool hit(const Ray ray, const real_t start, const real_t end,
			const unsigned int model_index, HitRecord* record_ptr);

	virtual size_t num_models() const;
	virtual Boundingbox* get_boundingbox() const;

	virtual void construct_boundingbox();

};
}

#endif /* BOUNDINGBOX_HPP_ */
