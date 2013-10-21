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

struct SortBoundingbox {
	size_t axis;
	bool operator()(const Geometry* &left,
			const Geometry* &right) {
		real_t left_middle = (left->get_boundingbox()->minPoint[axis] +
							left->get_boundingbox()->maxPoint[axis]) / 2;
		real_t right_middle = (right->get_boundingbox()->minPoint[axis] +
									right->get_boundingbox()->maxPoint[axis]) / 2;
		return left_middle < right_middle;
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
	return true;
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

void KdTree::build_kd_tree() {
	build_kd_tree(root, geometry_sorted_list[0]);
}

void KdTree::build_kd_tree(KdNode* tree, const GeometryList& list) {
	real_t plane;
	size_t axis;
	bool divisible;
	divisible = choose_plane(list, axis, plane);
	if (divisible) {
		tree = new KdNode();
		tree->left = tree->right = NULL;
		GeometryList left_list;
		GeometryList right_list;
		classify(&list, axis, plane, left_list, right_list);
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

bool KdTree::choose_plane(GeometryList list, size_t& axis, real_t& plane) const {
	size_t current_axis = 0;
	real_t metric;
	real_t best_metric;
	metric = best_metric = std::numeric_limits<real_t>::infinity();

	real_t min = std::numeric_limits<real_t>::infinity();
	real_t max = -std::numeric_limits<real_t>::infinity();
	for (size_t i = 0; i < list.size(); i++) {
		if (list[i]->get_boundingbox()->minPoint[current_axis] < min)
			min = list[i]->get_boundingbox()->minPoint[current_axis];
		if (list[i]->get_boundingbox()->maxPoint[current_axis] > max)
			max = list[i]->get_boundingbox()->maxPoint[current_axis];
	}
	real_t choice = (min + max) / 2;

	GeometryList* current_list_ptr = &list;
	size_t left_count = 0;
	size_t right_count = 0;
	size_t share_count = 0;
	GeometryList left_list;
	GeometryList right_list;

	while (metric > THRESHOLD && current_axis < 3) {
		classify(current_list_ptr, current_axis, choice, left_list, right_list);
		size_t current_left_count = left_list.size();
		size_t current_right_count = right_list.size();
		size_t current_share_count = current_list_ptr->size() - current_left_count - current_right_count;

		if (current_left_count > current_right_count) {
			left_count = current_left_count;
			right_count += current_right_count;
			metric = abs(left_count - right_count) + current_share_count;
			current_list_ptr = &left_list;
			max = choice;
		}
		else if (current_left_count < current_right_count) {
			left_count += current_left_count;
			right_count = current_right_count;
			metric = abs(right_count - left_count) + current_share_count;
			current_list_ptr = &right_list;
			min = choice;
		}
		else {
			current_list_ptr = &list;
			left_count = 0;
			right_count = 0;
			share_count = 0;
			current_axis++;
			break;
		}

		if (metric < best_metric) {
			best_metric = metric;
			plane = choice;
			axis = current_axis;
		}

		if (abs(max - min) < RESOLUTION || metric == current_list_ptr->size())
			current_axis++;
	}

	return metric == current_list_ptr->size();
}

void KdTree::classify(const GeometryList* list_ptr, size_t axis, real_t plane,
		GeometryList& left_list, GeometryList& right_list) const {
	left_list.clear();
	right_list.clear();

	for (size_t i = 0; i < list_ptr->size(); i++) {
		Boundingbox* boundingbox_ptr = (*list_ptr)[i]->get_boundingbox();
		if (boundingbox_ptr->maxPoint[axis] < plane)
			left_list.push_back((*list_ptr)[i]);
		if (boundingbox_ptr->minPoint[axis] > plane)
			right_list.push_back((*list_ptr)[i]);
	}
}

}

