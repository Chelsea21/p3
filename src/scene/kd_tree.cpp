/*
 * kd_tree.cpp
 *
 *  Created on: Oct 20, 2013
 *      Author: Chenxi Liu chenxil
 */

#include <scene/kd_tree.h>

namespace _462 {

struct sort_pred {
	bool operator() (const std::pair<real_t, Geometry*> &left,
			const std::pair<real_t, Geometry*> &right) {
		return left.first < right.first;
	}
};

KdTree::KdTree(GeometryList geometries)
: geometry_sorted_list(std::vector<GeometryList>(3)) {
	std::vector<std::pair<real_t, Geometry*>> aa_sort_list(geometries.size());

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < geometries.size(); j++) {
			Boundingbox* boundingbox_ptr = geometries[j]->get_boundingbox();
			real_t middle = (boundingbox_ptr->minPoint[i] + boundingbox_ptr->maxPoint[j]) / 2;
			aa_sort_list[j] = std::pair(middle, geometries[j]);
		}

		std::sort(aa_sort_list.begin(), aa_sort_list.end(), sort_pred());
		geometry_sorted_list[i] = std::vector<Geometry*>(geometries.size());
		for (size_t j = 0; j < geometries.size(); j++) {
			geometry_sorted_list[i][j] = aa_sort_list[j].second;
		}
	}

	construct_boundingbox();
}

KdTree::~KdTree() {

}

void KdTree::render() const {
	std::cerr << "Cannot render kd-tree." << std::endl;
}

bool KdTree::hit(const Ray ray, const real_t start, const real_t end,
		const unsigned int model_index, HitRecord* record_ptr) {

}

size_t KdTree::num_models() const {
	return 1;
}

Boundingbox* KdTree::get_boundingbox() const {
	return const_cast<Boundingbox*>(boundingbox);
}

void KdTree::construct_boundingbox() {
	boundingbox.minPoint = Vector3(geometry_sorted_list[0].front()->position.x,
									geometry_sorted_list[1].front()->position.y,
									geometry_sorted_list[2].front()->position.z);
	boundingbox.maxPoint = Vector3(geometry_sorted_list[0].back()->position.x,
									geometry_sorted_list[1].back()->position.y,
									geometry_sorted_list[2].back()->position.z);
	boundingbox.isLoose = true;
}

void KdTree::construct_kd_tree() {
	build_kd_tree(root, geometry_sorted_list[0]);
}

void KdTree::build_kd_tree(KdNode* tree, const GeometryList& list) {
	real_t plane;
	size_t axis;
	bool divisible;
	choose_plane(list, axis, plane, divisible);
	if (divisible) {
		tree = new KdNode();
		tree->left = tree->right = NULL;
		GeometryList left_list;
		GeometryList right_list;
		classify(list, axis, plane, left_list, right_list);
		if (left_list.size() > THRESHOLD)
			build_kd_tree(tree->left, left_list);
		if (tree->left == NULL) {
			KdNode* leaf = new KdNode();
			leaf->left = leaf->right = NULL;
			leaf->geometries = left_list;
			tree->left = leaf;
		}
		if (right_list.size() > THRESHOLD)
			build_kd_tree(tree->right, right_list);
		if (tree->right == NULL) {
			KdNode* leaf = new KdNode();
			leaf->left = leaf->right = NULL;
			leaf->geometries = right_list;
			tree->right = leaf;
		}
	}
}

void KdTree::choose_plane(const GeometryList& list, size_t& axis, real_t& plane, bool& divisible) const {
	size_t current_axis = 0;

}

void KdTree::classify(const GeometryList& list, size_t axis, real_t plane,
		GeometryList& left_list, GeometryList& right_list) const {

}

}

