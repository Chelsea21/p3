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

// TODO add a number field
typedef std::vector<Geometry*> GeometryList;
typedef std::vector<GeometryList> GeometrySortedList;

struct KdNode {
	KdNode* left;
	KdNode* right;
	real_t plane;
	size_t axis;
	GeometryList geometries;
};

class KdTree {
public:
	KdTree() : root(NULL) { }
	KdTree(GeometryList geometries);
	virtual ~KdTree();

	Boundingbox boundingbox;

	enum ClassifiedSide { RIGHT, LEFT, RIGHT_LEFT, LEFT_RIGHT, UNKNOWN };

	virtual void render() const;
	// Hit function
	virtual bool hit(const Ray ray, const real_t start, const real_t end,
			const unsigned int model_index, HitRecord* record_ptr);
	virtual size_t num_models() const;
	virtual Boundingbox* get_boundingbox() const;
	virtual void construct_boundingbox();

	void build_kd_tree();

private:
	GeometrySortedList geometry_sorted_list;
	KdNode* root;

	KdNode* build_kd_tree(KdNode* tree, const GeometryList& list);
	bool choose_plane(GeometryList list, size_t& axis, real_t& plane) const;
	void classify(const GeometryList list_ptr, size_t axis, real_t plane,
			GeometryList& left_list, GeometryList& right_list, GeometryList& share_list,
			const bool shared) const;
	void find_min_max(const GeometryList list, const size_t current_axis, real_t& min,
						real_t& max) const;
	bool traverse(KdNode* root, const Ray ray, const real_t start, const real_t end,
					HitRecord* record_ptr);
	int classify_ray(const Ray ray, const KdNode* root,
					const real_t start, const real_t end, real_t& t_plane) const;
};

}

#endif /* KD_TREE_H_ */
