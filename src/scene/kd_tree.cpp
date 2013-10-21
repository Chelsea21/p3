/*
 * kd_tree.cpp
 *
 *  Created on: Oct 20, 2013
 *      Author: Chenxi Liu chenxil
 */

#include "scene/kd_tree.hpp"
#include "scene/ray.hpp"

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
	(void) model_index;
	return traverse(root, ray, start, end, record_ptr);
}

bool KdTree::traverse(KdNode* root, const Ray ray, const real_t start, const real_t end,
					HitRecord* record_ptr) {
	if (root->left == NULL && root->right == NULL) {
		real_t t0 = start;
		real_t t1 = end;
		for (size_t i = 0; i < root->geometries.size(); i++) {
			for (size_t j = 0; j < root->geometries[i]->num_models(); j++) {
				if (root->geometries[i]->hit(ray, t0, t1, j, record_ptr)) {
					if (record_ptr != NULL)
						t1 = record_ptr->time;
					else
						return true;
				}
			}
		}

		return record_ptr->material_ptr != NULL;
	}

	real_t t_plane;
	int path = classify_ray(ray, root, start, end, t_plane);
	switch (path) {
	case LEFT_RIGHT:
		if (traverse(root->left, ray, start, t_plane, record_ptr))
			return true;
		return traverse(root->right, ray, t_plane, end, record_ptr);
	case RIGHT_LEFT:
		if (traverse(root->right, ray, t_plane, end, record_ptr))
			return true;
		return traverse(root->left, ray, start, t_plane, record_ptr);
	case LEFT:
		return traverse(root->left, ray, start, t_plane, record_ptr);
	case RIGHT:
		return traverse(root->right, ray, t_plane, end, record_ptr);
	default:
		break;
	}

	return record_ptr->material_ptr != NULL;
}

int KdTree::classify_ray(const Ray ray, const KdNode* root,
						const real_t start, const real_t end, real_t& t_plane) const {
	real_t start_point = (ray.e + start * ray.d)[root->axis];
	real_t end_point = (ray.e + end * ray.d)[root->axis];

	t_plane = (root->plane - ray.e[root->axis]) / ray.d[root->axis];

	if (start_point <= root->plane && end_point <= root->plane)
		return RIGHT;
	if (start_point <= root->plane && end_point >= root->plane)
		return LEFT_RIGHT;
	if (start_point >= root->plane && end_point >= root->plane)
		return LEFT;
	if (start_point >= root->plane && end_point <= root->plane)
		return RIGHT_LEFT;
	return UNKNOWN;
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
		tree->plane = plane;
		tree->axis = axis;
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

	real_t min;
	real_t max;
	GeometryList* current_list_ptr = &list;
	find_min_max(current_list_ptr, current_axis, min, max);
	real_t choice = (min + max) / 2;

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
			right_count += current_right_count + share_count;
			share_count = current_share_count;
			metric = abs(left_count - right_count) + current_share_count;
			current_list_ptr = &left_list;
			max = choice;
		}
		else if (current_left_count < current_right_count) {
			left_count += current_left_count + share_count;
			right_count = current_right_count;
			share_count = current_share_count;
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
			find_min_max(current_list_ptr, current_axis, min, max);
			choice = (min + max) / 2;
		}

		if (metric < best_metric) {
			best_metric = metric;
			plane = choice;
			axis = current_axis;
		}

		if (abs(max - min) < RESOLUTION || metric == current_list_ptr->size()) {
			current_axis++;
			find_min_max(current_list_ptr, current_axis, min, max);
			choice = (min + max) / 2;
		}
	}

	return metric < current_list_ptr->size();
}

void KdTree::find_min_max(const GeometryList* list_ptr, const size_t current_axis, real_t& min,
						real_t& max) const {
	min = std::numeric_limits<real_t>::infinity();
	max = -std::numeric_limits<real_t>::infinity();
	for (size_t i = 0; i < list_ptr->size(); i++) {
		if ((*list_ptr)[i]->get_boundingbox()->minPoint[current_axis] < min)
			min = (*list_ptr)[i]->get_boundingbox()->minPoint[current_axis];
		if ((*list_ptr)[i]->get_boundingbox()->maxPoint[current_axis] > max)
			max = (*list_ptr)[i]->get_boundingbox()->maxPoint[current_axis];
	}
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

