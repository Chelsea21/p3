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

KdTree::KdTree(GeometryList geometries)
: geometry_sorted_list(std::vector<GeometryList>(3)) {
	// TODO rewrite
	std::vector<std::pair<real_t, Geometry*>> aa_sort_list(geometries.size());

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < geometries.size(); j++) {
			Boundingbox* boundingbox_ptr = geometries[j]->get_boundingbox();
			real_t middle = (boundingbox_ptr->minPoint[i] + boundingbox_ptr->maxPoint[i]) / 2;
			aa_sort_list[j] = std::pair<real_t, Geometry*>(middle, geometries[j]);
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
	// TODO delete
	delete root->right;
	delete root->left;
	delete root;
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

		if (record_ptr == NULL)
			return false;
		return record_ptr->hit;
	}

	real_t t_plane;
	int path = classify_ray(ray, root, start, end, t_plane);
	switch (path) {
	case LEFT_RIGHT:
		if (traverse(root->left, ray, start, t_plane, record_ptr))
			return true;
		return traverse(root->right, ray, t_plane, end, record_ptr);
	case RIGHT_LEFT:
		if (traverse(root->right, ray, start, t_plane, record_ptr))
			return true;
		return traverse(root->left, ray, t_plane, end, record_ptr);
	case LEFT:
		return traverse(root->left, ray, start, end, record_ptr);
	case RIGHT:
		return traverse(root->right, ray, start, end, record_ptr);
	default:
		break;
	}
	std::cout << path << std::endl;
	return record_ptr->hit;
}

int KdTree::classify_ray(const Ray ray, const KdNode* root,
						const real_t start, const real_t end, real_t& t_plane) const {
	real_t start_point = (ray.e + start * ray.d)[root->axis];
	real_t end_point = (ray.e + end * ray.d)[root->axis];

	t_plane = (root->plane - ray.e[root->axis]) / ray.d[root->axis];

	if (start_point >= root->plane && end_point >= root->plane)
		return RIGHT;
	if (start_point <= root->plane && end_point >= root->plane)
		return LEFT_RIGHT;
	if (start_point <= root->plane && end_point <= root->plane)
		return LEFT;
	if (start_point >= root->plane && end_point <= root->plane)
		return RIGHT_LEFT;
	std::cout << "start: " << start_point << "; end: " << end_point << "; plane: " << root->plane << std::endl;
	return UNKNOWN;
}

size_t KdTree::num_models() const {
	return 1;
}

Boundingbox* KdTree::get_boundingbox() const {
	return const_cast<Boundingbox*>(&boundingbox);
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
	root = build_kd_tree(root, geometry_sorted_list[0]);
	if (root == NULL) {
		root = new KdNode();
		root->left = root->right = NULL;
		root->geometries = geometry_sorted_list[0];
	}
	std::cout << root->geometries.size() << std::endl;
	std::cout << root->left->geometries.size() << std::endl;
	std::cout << root->right->geometries.size() << std::endl;
}

KdNode* KdTree::build_kd_tree(KdNode* tree, const GeometryList& list) {
	real_t plane;
	size_t axis;
	bool divisible;
	divisible = choose_plane(list, axis, plane);
	std::cout << divisible << std::endl;
	if (divisible) {
		tree = new KdNode();
		tree->left = tree->right = NULL;
		GeometryList left_list;
		GeometryList right_list;
		GeometryList share_list;
		classify(list, axis, plane, left_list, right_list, share_list, true);
		std::cout << left_list.size() << "\t" << right_list.size() << std::endl;
		tree->plane = plane;
		tree->axis = axis;
		if (left_list.size() > THRESHOLD)
			tree->left = build_kd_tree(tree->left, left_list);
		if (tree->left == NULL) {
			KdNode* leaf = new KdNode();
			leaf->left = leaf->right = NULL;
			leaf->geometries = left_list;
			tree->left = leaf;
		}
		if (right_list.size() > THRESHOLD)
			tree->right = build_kd_tree(tree->right, right_list);
		if (tree->right == NULL) {
			KdNode* leaf = new KdNode();
			leaf->left = leaf->right = NULL;
			leaf->geometries = right_list;
			tree->right = leaf;
		}

		return tree;
	}

	return NULL;
}

bool KdTree::choose_plane(GeometryList list, size_t& axis, real_t& plane) const {
	size_t current_axis = 0;
	real_t metric;
	real_t best_metric;
	metric = best_metric = std::numeric_limits<real_t>::infinity();

	real_t min;
	real_t max;
	GeometryList current_list = list;
	find_min_max(current_list, current_axis, min, max);
	real_t choice;

	size_t left_count = 0;
	size_t right_count = 0;
	size_t share_count = 0;
	GeometryList left_list;
	GeometryList right_list;
	GeometryList share_list;
	int previous_move = UNKNOWN;

	while (metric > THRESHOLD && current_axis < 3) {
		choice = (min + max) / 2;
		classify(current_list, current_axis, choice, left_list, right_list, share_list, false);
		size_t current_left_count = left_list.size();
		size_t current_right_count = right_list.size();
		size_t current_share_count = list.size() - current_left_count - current_right_count;

		std::cout << "axis: " << current_axis << std::endl;
		std::cout << choice << std::endl;
		std::cout << "l: " << left_list.size() << "; " << "r: " << right_list.size() << "; s:" <<
				share_list.size() << std::endl;
		std::cout << "list: " << current_list.size() << std::endl;

		if (previous_move == LEFT) {
			left_count = current_left_count;
			right_count += current_right_count;
			share_count = current_share_count;
		}
		else if (previous_move == RIGHT) {
			left_count += current_left_count;
			right_count = current_right_count;
			share_count = current_share_count;
		}
		else {
			left_count = current_left_count;
			right_count = current_right_count;
			share_count = current_share_count;
		}
		std::cout << left_count << "\t" << right_count << "\t" << share_count << std::endl;
		std::cout << std::endl;
		metric = abs(left_count - right_count) + share_count;
		if (metric < best_metric) {
			best_metric = metric;
			plane = choice;
			axis = current_axis;
		}
		if (left_count > right_count) {
			left_list.insert(left_list.end(), share_list.begin(), share_list.end());
			current_list = left_list;
			current_list.resize(left_list.size());
			max = choice;
			previous_move = LEFT;
		}
		else if (left_count < right_count) {
			right_list.insert(right_list.end(), share_list.begin(), share_list.end());
			current_list = right_list;
			current_list.resize(right_list.size());
			min = choice;
			previous_move = RIGHT;
		}
		else {
			left_count = 0;
			right_count = 0;
			share_count = 0;
			current_axis++;
			if (current_axis > 2)
				break;
			current_list = list;
			find_min_max(current_list, current_axis, min, max);
			choice = (min + max) / 2;
			previous_move = UNKNOWN;
			continue;
		}

		// Use share here, because there's chance that we can still improve it.
		// Example: left: 2, right: 0, share: 1.
		if (std::fabs(max - min) < RESOLUTION || share_count == list.size()) {
			current_axis++;
			if (current_axis > 2)
				break;
			current_list = list;
			find_min_max(current_list, current_axis, min, max);
			choice = (min + max) / 2;
			previous_move = UNKNOWN;
		}
	}

	return best_metric < list.size();
}

void KdTree::find_min_max(const GeometryList list, const size_t current_axis, real_t& min,
						real_t& max) const {
	min = std::numeric_limits<real_t>::infinity();
	max = -std::numeric_limits<real_t>::infinity();
	for (size_t i = 0; i < list.size(); i++) {
		if (list[i]->get_boundingbox()->minPoint[current_axis] < min)
			min = list[i]->get_boundingbox()->minPoint[current_axis];
		if (list[i]->get_boundingbox()->maxPoint[current_axis] > max)
			max = list[i]->get_boundingbox()->maxPoint[current_axis];
	}
}

void KdTree::classify(const GeometryList list, size_t axis, real_t plane,
		GeometryList& left_list, GeometryList& right_list, GeometryList& share_list,
		const bool shared) const {
	left_list.clear();
	right_list.clear();
	share_list.clear();

	for (size_t i = 0; i < list.size(); i++) {
		Boundingbox* boundingbox_ptr = list[i]->get_boundingbox();
		if (boundingbox_ptr->maxPoint[axis] < plane)
			left_list.push_back(list[i]);
		else if (boundingbox_ptr->minPoint[axis] > plane)
			right_list.push_back(list[i]);
		else if (shared) {
			std::cout << "share" << std::endl;
			left_list.push_back(list[i]);
			right_list.push_back(list[i]);
		}
		else
			share_list.push_back(list[i]);
	}
}

}

