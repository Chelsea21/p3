/*
 * kd_tree.cpp
 *
 *  Created on: Oct 20, 2013
 *      Author: Chenxi Liu chenxil
 */

#include "scene/kd_tree.hpp"
#include "scene/ray.hpp"

namespace _462 {

KdTree::KdTree(GeometryList geometries)
: root(NULL) {
	size_t num_models = 0;
	for (size_t i = 0; i < geometries.size(); i++) {
		num_models += geometries[i]->num_models();
	}
	geometry_boundingbox_ptrs = BoundingboxPointers(num_models);

	for (size_t i = 0; i < geometries.size(); i++) {
		for (size_t j = 0; j < geometries[i]->get_boundingboxs().size(); j++)
			geometry_boundingbox_ptrs.list[geometry_boundingbox_ptrs.size++] =
					geometries[i]->get_boundingboxs()[j];
	}
}

KdTree::~KdTree() {
	destory_kd_tree(root);
}

void KdTree::destory_kd_tree(KdNode* root) {
	if (root->left == NULL && root->right == NULL) {
		for (size_t i = 0; i < root->boundingbox_ptrs.size; i++) {
			delete root->boundingbox_ptrs.list[i];
		}
		delete root;

		return ;
	}
	if (root->left != NULL)
		destory_kd_tree(root->left);
	if (root->right != NULL)
		destory_kd_tree(root->right);
	delete root;
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
		for (size_t i = 0; i < root->boundingbox_ptrs.size; i++) {
			size_t model_num = root->boundingbox_ptrs.list[i]->model_index;
			//std::cout << model_num << std::endl;
			//std::cout << i << ": " << root->boundingbox_ptrs.size << ": " <<
				//	model_num << std::endl;
			if (root->boundingbox_ptrs.list[i]->hit(ray, t0, t1, 0, record_ptr)) {
				//std::cout << root->boundingbox_ptrs.list[i]->geometry->num_models()<< std::endl;
				//std::cout << model_num << std::endl;
				//std::cout << record_ptr->hit << std::endl;
				//std::cout << "1" << std::endl;
				bool geo_hit = root->boundingbox_ptrs.list[i]->geometry->hit(ray, t0, t1,
							model_num, record_ptr);
				//std::cout << "2" << std::endl;
				if (geo_hit) {
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
	return UNKNOWN;
}

void KdTree::build_kd_tree() {
	root = build_kd_tree(root, geometry_boundingbox_ptrs);
	if (root == NULL) {
		root = new KdNode();
		root->left = root->right = NULL;
		root->boundingbox_ptrs = geometry_boundingbox_ptrs;
	}
	//std::cout << root->boundingbox_ptrs.size << std::endl;
	//std::cout << root->left->boundingbox_ptrs.size << std::endl;
	//std::cout << root->right->boundingbox_ptrs.size << std::endl;
}

KdNode* KdTree::build_kd_tree(KdNode* tree, const BoundingboxPointers& list) {
	real_t plane;
	size_t axis;
	bool divisible;
	divisible = choose_plane(list, axis, plane);
	if (divisible) {
		tree = new KdNode();
		tree->left = tree->right = NULL;
		BoundingboxPointers left_list(list.size / 3);
		BoundingboxPointers right_list(list.size / 3);
		BoundingboxPointers share_list(list.size / 3);
		classify(list, axis, plane, left_list, right_list, share_list, true);
		tree->plane = plane;
		tree->axis = axis;
		if (left_list.size > THRESHOLD)
			tree->left = build_kd_tree(tree->left, left_list);
		if (tree->left == NULL) {
			KdNode* leaf = new KdNode();
			leaf->left = leaf->right = NULL;
			leaf->boundingbox_ptrs = left_list;
			tree->left = leaf;
		}
		if (right_list.size > THRESHOLD)
			tree->right = build_kd_tree(tree->right, right_list);
		if (tree->right == NULL) {
			KdNode* leaf = new KdNode();
			leaf->left = leaf->right = NULL;
			leaf->boundingbox_ptrs = right_list;
			tree->right = leaf;
		}

		return tree;
	}

	return NULL;
}

bool KdTree::choose_plane(BoundingboxPointers list, size_t& axis, real_t& plane) const {
	size_t current_axis = 0;
	real_t metric;
	real_t best_metric;
	metric = best_metric = std::numeric_limits<real_t>::infinity();

	real_t min;
	real_t max;
	BoundingboxPointers current_list(list.size);
	current_list = list;
	find_min_max(current_list, current_axis, min, max);
	real_t choice;

	size_t left_count = 0;
	size_t right_count = 0;
	size_t share_count = 0;
	BoundingboxPointers left_list(list.size / 3);
	BoundingboxPointers right_list(list.size / 3);
	BoundingboxPointers share_list(list.size / 3);
	int previous_move = UNKNOWN;

	while (metric > THRESHOLD && current_axis < 3) {
		choice = (min + max) / 2;
		classify(current_list, current_axis, choice, left_list, right_list, share_list, false);
		size_t current_left_count = left_list.size;
		size_t current_right_count = right_list.size;
		size_t current_share_count = list.size - current_left_count - current_right_count;

/*
		std::cout << "axis: " << current_axis << std::endl;
		std::cout << choice << std::endl;
		std::cout << "l: " << left_list.size << "; " << "r: " << right_list.size << "; s:" <<
				share_list.size << std::endl;
		std::cout << "list: " << current_list.size << std::endl;
*/
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
		metric = abs(left_count - right_count) + share_count;
		if (metric < best_metric) {
			best_metric = metric;
			plane = choice;
			axis = current_axis;
		}
		if (left_count > right_count) {
			left_list.push_back_list(share_list);
			current_list = left_list;
			max = choice;
			previous_move = LEFT;
		}
		else if (left_count < right_count) {
			right_list.push_back_list(share_list);
			current_list = right_list;
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
		if (std::fabs(max - min) < RESOLUTION || share_count == list.size) {
			current_axis++;
			if (current_axis > 2)
				break;
			current_list = list;
			find_min_max(current_list, current_axis, min, max);
			choice = (min + max) / 2;
			previous_move = UNKNOWN;
		}
	}

	return best_metric < list.size;
}

void KdTree::find_min_max(const BoundingboxPointers ptr_list, const size_t current_axis, real_t& min,
						real_t& max) const {
	min = std::numeric_limits<real_t>::infinity();
	max = -std::numeric_limits<real_t>::infinity();

	for (size_t i = 0; i < ptr_list.size; i++) {
		if (ptr_list.list[i]->minPoint[current_axis] < min)
			min = ptr_list.list[i]->minPoint[current_axis];
		if (ptr_list.list[i]->maxPoint[current_axis] > max)
			max = ptr_list.list[i]->maxPoint[current_axis];
	}
}

void KdTree::classify(const BoundingboxPointers list, size_t axis, real_t plane,
		BoundingboxPointers& left_list, BoundingboxPointers& right_list, BoundingboxPointers& share_list,
		const bool shared) const {
	left_list.list.clear();
	right_list.list.clear();
	share_list.list.clear();
	left_list.size = 0;
	right_list.size = 0;
	share_list.size = 0;

	for (size_t i = 0; i < list.size; i++) {
		Boundingbox* boundingbox_ptr = list.list[i];
		if (boundingbox_ptr->maxPoint[axis] < plane) {
			left_list.push_back(list.list[i]);
		}
		else if (boundingbox_ptr->minPoint[axis] > plane) {
			right_list.push_back(list.list[i]);
		}
		else if (shared) {
			left_list.push_back(list.list[i]);
			right_list.push_back(list.list[i]);
		}
		else {
			share_list.push_back(list.list[i]);
		}
	}
}

}

