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
	// Generate a list of pointers of bounding boxes.
	size_t num_models = 0;
	for (size_t i = 0; i < geometries.size(); i++) {
		num_models += geometries[i]->num_models();
	}
	geometry_boundingbox_ptrs.reserve(num_models);
	for (size_t i = 0; i < geometries.size(); i++) {
		for (size_t j = 0; j < geometries[i]->get_boundingboxs().size(); j++)
			geometry_boundingbox_ptrs.push_back(geometries[i]->get_boundingboxs()[j]);
	}
}

KdTree::~KdTree() {
	destory_kd_tree(root);
}

void KdTree::add_photon(const Photon photon) {
	if (photon.color.r > 1 || photon.color.g > 1 || photon.color.b > 1) {
		std::cout << "adding: " << photon.color.r << "\t" << photon.color.g << "\t" <<
				photon.color.b << std::endl;
	}
	photons.push_back(photon);
}

/**
 * Destories the kd-tree and bounding boxes.
 */
void KdTree::destory_kd_tree(KdNode* root) {
	if (root->left == NULL && root->right == NULL) {
		// Deletes bounding boxes.
		for (size_t i = 0; i < root->boundingbox_ptrs.size(); i++) {
			delete root->boundingbox_ptrs[i];
		}
		for (size_t i = 0; i < root->photon_ptrs.size(); i++) {
			delete root->photon_ptrs[i];
		}
		// Deletes node.
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

/**
 * Traverese the kd-tree.
 * @param root			Current root.
 * @param ray			The ray.
 * @param start			The t parameter of the starting point.
 * @param end			The t parameter of the ending point.
 * @param record_ptr	The pointer of record struct.
 */
bool KdTree::traverse(KdNode* root, const Ray ray, const real_t start, const real_t end,
					HitRecord* record_ptr) {
	// Found the leaf node.
	if (root->left == NULL && root->right == NULL) {
		real_t t0 = start;
		real_t t1 = end;
		// Checks every bounding box in this node.
		for (size_t i = 0; i < root->boundingbox_ptrs.size(); i++) {
			size_t model_num = root->boundingbox_ptrs[i]->model_index;
			if (root->boundingbox_ptrs[i]->hit(ray, t0, t1, 0, record_ptr)) {
				bool geo_hit = root->boundingbox_ptrs[i]->geometry->hit(ray, t0, t1,
							model_num, record_ptr);
				if (geo_hit) {
					// For shadow ray.
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
	// Traverses in different ways for different ray.
	int path = classify_ray(ray, root, start, end, t_plane);
	switch (path) {
	// Ray crosses the plane from left to right.
	case LEFT_RIGHT:
		if (traverse(root->left, ray, start, t_plane, record_ptr))
			return true;
		return traverse(root->right, ray, t_plane, end, record_ptr);
	// Ray crosses the plane from right to left.
	case RIGHT_LEFT:
		if (traverse(root->right, ray, start, t_plane, record_ptr))
			return true;
		return traverse(root->left, ray, t_plane, end, record_ptr);
	// Ray lays on the left side.
	case LEFT:
		return traverse(root->left, ray, start, end, record_ptr);
	// Ray lays on the right side.
	case RIGHT:
		return traverse(root->right, ray, start, end, record_ptr);
	default:
		break;
	}
	return record_ptr->hit;
}

/**
 * Checks the spatial relation between the given ray and plane.
 * @param ray 		The ray.
 * @param root		The node.
 * @param start		The t parameter of the starting point.
 * @param end		The t parameter of the ending point.
 * @param t_plane	The return value. The t parameter of the intersection.
 */
int KdTree::classify_ray(const Ray ray, const KdNode* root,
						const real_t start, const real_t end, real_t& t_plane) const {
	real_t start_point = (ray.e + start * ray.d)[root->axis];
	real_t end_point = (ray.e + end * ray.d)[root->axis];

	// The t parameter of the intersection.
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

// Builds kd-tree.
void KdTree::build_kd_tree() {
	if (geometry_boundingbox_ptrs.size() > 0) {
		root = build_kd_tree(root, geometry_boundingbox_ptrs);
		// The tree only contains root.
		if (root == NULL) {
			root = new KdNode();
			root->left = root->right = NULL;
			root->boundingbox_ptrs = geometry_boundingbox_ptrs;
		}
	}
	else {
		PhotonPointers photon_ptrs;
		for (size_t i = 0; i < photons.size(); i++) {
			photon_ptrs.push_back(&photons[i]);
		}
		if (photon_ptrs.size() > 0)
			root = build_kd_tree(root, photon_ptrs);
		// The tree only contains root.
		if (root == NULL) {
			root = new KdNode();
			root->left = root->right = NULL;
			root->photon_ptrs = photon_ptrs;
		}
		std::cout << photons.size() << std::endl;
	}
}

/**
 * Builds the kd-tree recursively.
 * @param tree		The current node.
 * @param list		The list of bounding boxes contained by the subtree.
 * @return 			The current node.
 */
template<typename T> KdNode* KdTree::build_kd_tree(KdNode* tree, T& list) {
	real_t plane;
	size_t axis;
	bool divisible;

	// Choose a optimized plane to divide the boxes.
	divisible = choose_plane(list, axis, plane);

	if (divisible) {
		tree = new KdNode();
		tree->left = tree->right = NULL;
		T left_list(list.size() / 3);
		T right_list(list.size() / 3);
		T share_list(list.size() / 3);

		// Divides the list into three. Left and right list also contains boxes
		// in shared list.
		classify(list, axis, plane, left_list, right_list, share_list, true);
		tree->plane = plane;
		tree->axis = axis;

		// Too many boxes. Builds subtrees recursively.
		if (left_list.size() > THRESHOLD)
			tree->left = build_kd_tree(tree->left, left_list);
		if (tree->left == NULL) {
			KdNode* leaf = new KdNode();
			leaf->left = leaf->right = NULL;
			leaf->add(left_list);
			tree->left = leaf;
		}
		if (right_list.size() > THRESHOLD)
			tree->right = build_kd_tree(tree->right, right_list);
		if (tree->right == NULL) {
			KdNode* leaf = new KdNode();
			leaf->left = leaf->right = NULL;
			leaf->add(right_list);
			tree->right = leaf;
		}

		return tree;
	}

	return NULL;
}

real_t KdTree::find_k_nn(const Photon photon, const size_t nn_num, PhotonPointers& knn_ptr_list) const {
	std::priority_queue<PhotonDistanceComparor,
						std::vector<PhotonDistanceComparor>,
						PhotonDistanceComparor> queue;
	find_k_nn_queue(root, &photon, nn_num, queue);
	real_t radius = 0;
	radius = length(queue.top().photon_ptr->position - queue.top().center_ptr->position);

	while (queue.size() > 0) {
		Photon* top_ptr = queue.top().photon_ptr;
		knn_ptr_list.push_back(top_ptr);
		queue.pop();
	}

	return radius;
}

void KdTree::find_k_nn_queue(KdNode* root, const Photon* photon_ptr, const size_t nn_num,
			std::priority_queue<PhotonDistanceComparor,
			std::vector<PhotonDistanceComparor>,
			PhotonDistanceComparor>& knn_ptr_queue) const {
	if (root->right == NULL && root->left == NULL) {
		for (size_t i = 0; i < root->photon_ptrs.size(); i++) {
			PhotonDistanceComparor cmp;
			cmp.center_ptr = const_cast<Photon*>(photon_ptr);
			cmp.photon_ptr = root->photon_ptrs[i];
			if (cmp.center_ptr == NULL || cmp.photon_ptr == NULL)
				std::cout << cmp.center_ptr << "\t " << cmp.photon_ptr << std::endl;
			if (std::fabs(photon_ptr->position.x - root->photon_ptrs[i]->position.x) < 1e-3 ||
				std::fabs(photon_ptr->position.y - root->photon_ptrs[i]->position.y) < 1e-3 ||
				std::fabs(photon_ptr->position.z - root->photon_ptrs[i]->position.z) < 1e-3) {
				knn_ptr_queue.push(cmp);
			}
			//std::cout << root->photon_ptrs[i]->color.r << std::endl;

		}

		if (knn_ptr_queue.size() > nn_num)
			for (size_t i = 0; i < knn_ptr_queue.size() - nn_num; i++) {
				knn_ptr_queue.pop();
			}

		return ;
	}

	if (photon_ptr->position[root->axis] < root->plane) {
		find_k_nn_queue(root->left, photon_ptr, nn_num, knn_ptr_queue);
		if (knn_ptr_queue.size() >= nn_num) {
		PhotonDistanceComparor farthest = knn_ptr_queue.top();

			// Compares the distance from the center to the farthest neighbor and to
			// the separating plane.
			if (length(farthest.photon_ptr->position - farthest.center_ptr->position) <
					std::fabs(photon_ptr->position[root->axis] - root->plane)) {
				if (knn_ptr_queue.size() > nn_num)
					for (size_t i = 0; i < knn_ptr_queue.size() - nn_num; i++) {
						knn_ptr_queue.pop();
					}
				return ;
			}
		}
		find_k_nn_queue(root->right, photon_ptr, nn_num, knn_ptr_queue);
	}
	else {
		find_k_nn_queue(root->right, photon_ptr, nn_num, knn_ptr_queue);
		if (knn_ptr_queue.size() >= nn_num) {
				PhotonDistanceComparor farthest = knn_ptr_queue.top();

			// Compares the distance from the center to the farthest neighbor and to
			// the separating plane.
			if (length(farthest.photon_ptr->position - farthest.center_ptr->position) <
					std::fabs(photon_ptr->position[root->axis] - root->plane)) {
				if (knn_ptr_queue.size() > nn_num)
					for (size_t i = 0; i < knn_ptr_queue.size() - nn_num; i++) {
						knn_ptr_queue.pop();
					}
				return ;
			}
		}
		find_k_nn_queue(root->left, photon_ptr, nn_num, knn_ptr_queue);
	}
}

struct PhotonComparor {
	int axis;

	bool operator() (Photon* ptr1, Photon* ptr2) {
		return ptr1->position[axis] < ptr2->position[axis];
	}
};

bool KdTree::choose_plane(PhotonPointers& list, size_t& axis, real_t& plane) const {
	if (list.size() > 0)
		for (size_t current_axis = 0; current_axis < 3; current_axis++) {
			PhotonComparor comparor;
			comparor.axis = current_axis;
			std::sort(list.begin(), list.end(), comparor);

			int middle = list.size() / 2;
			size_t before_middle = (middle - 1 < 0) ? middle : middle - 1;
			plane = (list[middle]->position[current_axis] + list[before_middle]->position[current_axis]) / 2;

			/*
			for (size_t i = 0; i < list.size(); i++) {
				std::cout << list[i]->position[current_axis] << " ";
			}
			std::cout << std::endl;
			std::cout << plane << " - " << list[middle]->position[current_axis] << std::endl;
			*/
			if (std::fabs(plane - list[middle]->position[current_axis]) > 1e-6) {
				//std::cout << "p: " << plane << std::endl;
				axis = current_axis;
				return true;
			}
		}

	return false;
}

void KdTree::classify(const PhotonPointers& list_ptr, size_t axis, real_t plane,
				PhotonPointers& left_list, PhotonPointers& right_list, PhotonPointers& share_list,
				const bool shared) const {
	(void) share_list;
	(void) shared;
	left_list.clear();
	right_list.clear();

	//std::cout << "start " << list_ptr.size() << std::endl;
	for (size_t i = 0; i < list_ptr.size(); i++) {
		if (list_ptr[i]->position[axis] < plane)
			left_list.push_back(list_ptr[i]);
		else
			right_list.push_back(list_ptr[i]);
	}
	//std::cout << left_list.size() << " : " << right_list.size() << std::endl;
	//std::cout << "end" << std::endl;
}

/**
 * Chooses an optimized plane for current list.
 * @param list		Current list.
 * @param axis		Return value. The chosen axis.
 * @param plane		Return value. The chosen plane.
 */
bool KdTree::choose_plane(const BoundingboxPointers& list, size_t& axis, real_t& plane) const {
	size_t current_axis = 0;
	real_t metric;
	real_t best_metric;
	metric = best_metric = std::numeric_limits<real_t>::infinity();

	// Find the volume of the scene.
	real_t min;
	real_t max;
	BoundingboxPointers current_list(list.size());
	current_list = list;
	find_min_max(current_list, current_axis, min, max);
	real_t choice;

	// Initialize.
	size_t left_count = 0;
	size_t right_count = 0;
	size_t share_count = 0;
	BoundingboxPointers left_list(list.size() / 3);
	BoundingboxPointers right_list(list.size() / 3);
	BoundingboxPointers share_list(list.size() / 3);
	int previous_move = UNKNOWN;

	while (metric > THRESHOLD && current_axis < 3) {
		choice = (min + max) / 2;
		// Use the chosen plane to separate boxes.
		classify(current_list, current_axis, choice, left_list, right_list, share_list, false);
		size_t current_left_count = left_list.size();
		size_t current_right_count = right_list.size();
		size_t current_share_count = list.size() - current_left_count - current_right_count;

		// If it's not the first round. We can compute the metric
		// using result from last round.
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

		// Keep record of the min metric.
		if (metric < best_metric) {
			best_metric = metric;
			plane = choice;
			axis = current_axis;
		}

		// Adjust the chose plane to find a better division.
		if (left_count > right_count) {
			left_list.insert(left_list.end(), share_list.begin(), share_list.end());
			current_list = left_list;
			max = choice;
			previous_move = LEFT;
		}
		else if (left_count < right_count) {
			right_list.insert(right_list.end(), share_list.begin(), share_list.end());
			current_list = right_list;
			min = choice;
			previous_move = RIGHT;
		}
		else {
			// All boxes are shared. Cannot find a dividing plane.
			// Go to next axis.
			left_count = 0;
			right_count = 0;
			share_count = 0;
			current_axis++;
			if (current_axis > 2)
				break;

			// Reset.
			current_list = list;
			find_min_max(current_list, current_axis, min, max);
			choice = (min + max) / 2;
			previous_move = UNKNOWN;
			continue;
		}

		// Use the number of shared objects here, because there's chance that we can still improve it.
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

/**
 * Finds the bound of current list of objects.
 * @param ptr_list		A list of pointers of bounding boxes.
 * @param current_axis	Finds bounds along this axis.
 * @param min			Return value. The lower bound.
 * @param max			Return value. The upper bound.
 */
void KdTree::find_min_max(const BoundingboxPointers& ptr_list, const size_t current_axis, real_t& min,
						real_t& max) const {
	min = std::numeric_limits<real_t>::infinity();
	max = -std::numeric_limits<real_t>::infinity();

	for (size_t i = 0; i < ptr_list.size(); i++) {
		if (ptr_list[i]->minPoint[current_axis] < min)
			min = ptr_list[i]->minPoint[current_axis];
		if (ptr_list[i]->maxPoint[current_axis] > max)
			max = ptr_list[i]->maxPoint[current_axis];
	}
}

/**
 * Separates the list into left list, right list, and shared list.
 * @param list		The input.
 * @param axis		The axis perpendicular to the plane.
 * @param plane		Divides using this plane.
 * @param left_list	Return value. Left list.
 * @param right_list Return value. Right list.
 * @param share_list Return value. Shared list.
 * @param shared	Whether adds shared objects to both left and right list.
 */
void KdTree::classify(const BoundingboxPointers& list, size_t axis, real_t plane,
		BoundingboxPointers& left_list, BoundingboxPointers& right_list, BoundingboxPointers& share_list,
		const bool shared) const {
	left_list.clear();
	right_list.clear();
	share_list.clear();

	for (size_t i = 0; i < list.size(); i++) {
		Boundingbox* boundingbox_ptr = list[i];
		if (boundingbox_ptr->maxPoint[axis] < plane) {
			left_list.push_back(list[i]);
		}
		else if (boundingbox_ptr->minPoint[axis] > plane) {
			right_list.push_back(list[i]);
		}
		else if (shared) {
			left_list.push_back(list[i]);
			right_list.push_back(list[i]);
		}
		else {
			share_list.push_back(list[i]);
		}
	}
}

}

