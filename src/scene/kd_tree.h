/*
 * kd_tree.h
 *
 *  Created on: Oct 20, 2013
 *      Author: Chenxi Liu chenxil
 */

#ifndef KD_TREE_H_
#define KD_TREE_H_

#include "scene/scene.hpp"

namespace _462 {

typedef std::vector<Geometry*> GeometryList;
typedef std::vector<GeometryList> GeometrySortedList;

struct KdNode {
	KdNode* left;
	KdNode* right;
	GeometryList geometries;
};

class KdTree {
public:
	KdTree() : root(NULL) { }
	KdTree(GeometryList geometries);
	virtual ~KdTree();

	Boundingbox boundingbox;

	const unsigned int THRESHOLD = 2;
	const real_t RESOLUTION = 1e-3;

	virtual void render() const;
	// Hit function
	virtual bool hit(const Ray ray, const real_t start, const real_t end,
			const unsigned int model_index, HitRecord* record_ptr);
	virtual size_t num_models() const;
	virtual Boundingbox* get_boundingbox() const;
	virtual void construct_boundingbox();

	void build_kd_tree();

private:
	enum ClassifiedSide { RIGHT, LEFT };
	GeometrySortedList geometry_sorted_list;
	KdNode* root;
	void build_kd_tree(KdNode* tree, const GeometryList& list);
	bool choose_plane(GeometryList list, size_t& axis, real_t& plane) const;
	void classify(const GeometryList* list_ptr, size_t axis, real_t plane,
			GeometryList& left_list, GeometryList& right_list) const;
};

}

#endif /* KD_TREE_H_ */
