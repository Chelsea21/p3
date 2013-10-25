/*
 * kd_tree.h
 *
 *  Created on: Oct 20, 2013
 *      Author: Chenxi Liu chenxil
 */

#ifndef KD_TREE_H_
#define KD_TREE_H_

#include "scene/scene.hpp"
#include "scene/boundingbox.hpp"

namespace _462 {

#define THRESHOLD 2
#define RESOLUTION 1e-2

typedef std::vector<Geometry*> GeometryList;
typedef std::vector<Boundingbox*> BoundingboxPointers;

struct KdNode {
	KdNode* left;
	KdNode* right;
	real_t plane;
	size_t axis;
	BoundingboxPointers boundingbox_ptrs;
};

class KdTree {
public:
	KdTree() : root(NULL), geometry_boundingbox_ptrs(BoundingboxPointers(1)) { }
	KdTree(GeometryList geometries);
	virtual ~KdTree();

	enum ClassifiedSide { RIGHT, LEFT, RIGHT_LEFT, LEFT_RIGHT, UNKNOWN };

	// Hit function
	virtual bool hit(const Ray ray, const real_t start, const real_t end,
			const unsigned int model_index, HitRecord* record_ptr);

	void destory_kd_tree(KdNode* root);
	void build_kd_tree();

private:
	KdNode* root;
	BoundingboxPointers geometry_boundingbox_ptrs;

	KdNode* build_kd_tree(KdNode* tree, const BoundingboxPointers& list);
	bool choose_plane(BoundingboxPointers list, size_t& axis, real_t& plane) const;
	void classify(const BoundingboxPointers list_ptr, size_t axis, real_t plane,
			BoundingboxPointers& left_list, BoundingboxPointers& right_list, BoundingboxPointers& share_list,
			const bool shared) const;
	void find_min_max(const BoundingboxPointers list, const size_t current_axis, real_t& min,
						real_t& max) const;
	bool traverse(KdNode* root, const Ray ray, const real_t start, const real_t end,
					HitRecord* record_ptr);
	int classify_ray(const Ray ray, const KdNode* root,
					const real_t start, const real_t end, real_t& t_plane) const;
};

}

#endif /* KD_TREE_H_ */
