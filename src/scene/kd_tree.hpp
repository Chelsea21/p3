/*
 * kd_tree.h
 *
 *  Created on: Oct 20, 2013
 *      Author: Chenxi Liu chenxil
 */

#ifndef KD_TREE_H_
#define KD_TREE_H_

#include <iostream>
#include <queue>
#include "math/vector.hpp"
#include "scene/scene.hpp"
#include "scene/boundingbox.hpp"

namespace _462 {

#define THRESHOLD 2
#define RESOLUTION 1e-2

typedef std::vector<Geometry*> GeometryList;
typedef std::vector<Boundingbox*> BoundingboxPointers;
typedef std::vector<Photon*> PhotonPointers;
typedef std::vector<Photon> PhotonList;

struct KdNode {
	KdNode* left;
	KdNode* right;
	real_t plane;
	size_t axis;
	BoundingboxPointers boundingbox_ptrs;
	PhotonPointers photon_ptrs;

	void add(BoundingboxPointers boundingbox_ptrs) {
		this->boundingbox_ptrs = boundingbox_ptrs;
	}

	void add(PhotonPointers photon_ptrs) {
		this->photon_ptrs = photon_ptrs;
	}
};

struct PhotonDistanceComparor {
	Photon* center_ptr;
	Photon* photon_ptr;

	// TODO check
	bool operator() (PhotonDistanceComparor cmp1, PhotonDistanceComparor cmp2) {
		return length(cmp1.photon_ptr->position - cmp1.center_ptr->position) <
				length(cmp2.photon_ptr->position - cmp2.center_ptr->position);
	}
};

class KdTree {
public:
	KdTree() : root(NULL), geometry_boundingbox_ptrs(BoundingboxPointers(0)) { }
	KdTree(GeometryList geometries);
	KdTree(PhotonList photons) : root(NULL), photons(photons) { }
	virtual ~KdTree();

	enum ClassifiedSide { RIGHT, LEFT, RIGHT_LEFT, LEFT_RIGHT, UNKNOWN };

	// Hit function
	virtual bool hit(const Ray ray, const real_t start, const real_t end,
			const unsigned int model_index, HitRecord* record_ptr);

	real_t find_k_nn(const Photon photon, const size_t nn_num, PhotonPointers& knn_ptr_list) const;

	void add_photon(const Photon photon);

	void destory_kd_tree(KdNode* root);
	void build_kd_tree();

private:
	KdNode* root;
	BoundingboxPointers geometry_boundingbox_ptrs;
	PhotonList photons;

	template<typename T> KdNode* build_kd_tree(KdNode* tree, const T& list);
	bool choose_plane(BoundingboxPointers list, size_t& axis, real_t& plane) const;
	bool choose_plane(PhotonPointers list, size_t& axis, real_t& plane) const;
	void find_k_nn_queue(KdNode* root, const Photon* photon_ptr, const size_t nn_num,
			std::priority_queue<PhotonDistanceComparor,
			std::vector<PhotonDistanceComparor>,
			PhotonDistanceComparor>& knn_ptr_queue) const;
	void classify(const PhotonPointers list_ptr, size_t axis, real_t plane,
				PhotonPointers& left_list, PhotonPointers& right_list, PhotonPointers& share_list,
				const bool shared) const;
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
