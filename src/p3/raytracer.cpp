/**
 * @file raytacer.cpp
 * @brief Raytracer class
 *
 * Implement these functions for project 4.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#include "raytracer.hpp"
#include "scene/scene.hpp"
#include "scene/kd_tree.hpp"

#include <SDL_timer.h>
#include <iostream>
#include <random>
#include <limits>
#include <cmath>

#ifdef OPENMP // just a defense in case OpenMP is not installed.

#include <omp.h>

#endif
namespace _462 {

// TODO
// max number of threads OpenMP can use. Change this if you like.
#define MAX_THREADS 8

static const unsigned STEP_SIZE = 8;
static const unsigned PHOTON_NUM = 100000;
static const unsigned OBJECT_PHOTON_NUM = 500;
static const unsigned K_NN_NUM = 200;
static const real_t GLOBAL_RATIO = 0.9;

Raytracer::Raytracer() :
		scene(0), width(0), height(0), current_row(0),
		num_samples(1), photon_emission_flag(false) {
}

// random real_t in [0, 1)
static inline real_t random() {
	return real_t(rand()) / RAND_MAX;
}

Raytracer::~Raytracer() {
}

/**
 * Initializes the raytracer for the given scene. Overrides any previous
 * initializations. May be invoked before a previous raytrace completes.
 * @param scene The scene to raytrace.
 * @param width The width of the image being raytraced.
 * @param height The height of the image being raytraced.
 * @return true on success, false on error. The raytrace will abort if
 * false is returned.
 */
bool Raytracer::initialize(Scene* scene, size_t num_samples, size_t width,
		size_t height) {
	/*
	 * omp_set_num_threads sets the maximum number of threads OpenMP will
	 * use at once.
	 */
#ifdef OPENMP
	//omp_set_num_threads(MAX_THREADS);
#endif
	this->scene = scene;
	this->num_samples = num_samples;
	this->width = width;
	this->height = height;

	current_row = 0;

	Ray::init(scene->camera);
	scene->initialize();

	// TODO any initialization or precompuation before the trace

	return true;
}

/**
 * Performs a raytrace on the given pixel on the current scene.
 * The pixel is relative to the bottom-left corner of the image.
 * @param scene The scene to trace.
 * @param x The x-coordinate of the pixel to trace.
 * @param y The y-coordinate of the pixel to trace.
 * @param width The width of the screen in pixels.
 * @param height The height of the screen in pixels.
 * @return The color of that pixel in the final image.
 */
Color3 Raytracer::trace_pixel(const Scene* scene, size_t x, size_t y,
		size_t width, size_t height) {
	assert(x < width);
	assert(y < height);

	real_t dx = real_t(1) / width;
	real_t dy = real_t(1) / height;

	Color3 res = Color3::Black();
	unsigned int iter;
	for (iter = 0; iter < num_samples; iter++) {
		// pick a point within the pixel boundaries to fire our
		// ray through.
		real_t i = real_t(2) * (real_t(x) + random()) * dx - real_t(1);
		real_t j = real_t(2) * (real_t(y) + random()) * dy - real_t(1);
		std::vector<real_t> refractive_indices;
		refractive_indices.resize(1);
		refractive_indices[0] = scene->refractive_index;
		Ray ray(scene->camera.get_position(), Ray::get_pixel_dir(i, j));
		TracingResult result;
		result.is_photon = false;
		trace_point(ray, 1, refractive_indices, result);
		res += result.color;
	}
	return res * (real_t(1) / num_samples);
}

/**
 * Traces a point using the given eye ray.
 * @param scene                                        The scene.
 * @param e                                                Eye position.
 * @param d                                                The direction of eye ray.
 * @param depth                                        The tracing depth for reflection and refraction.
 * @param refractive_indices        The refractive indices stack.
 */
void Raytracer::trace_point(const Ray ray, unsigned int depth,
		std::vector<real_t> refractive_indices, TracingResult& result) const {
	if ((!result.is_photon && depth > MAX_DEPTH) ||
			(result.is_photon && (depth >= PHOTON_DEPTH ||
					power(result.photon.color) < 1e-8))) {
		result.color = Color3::Black();
		return;
	}

	// Depth of scene: choose a random eye position from a square lens.
	/*
	 real_t len_length = 0.05;
	 real_t rand_i = random_gaussian();
	 real_t rand_j = random_gaussian();
	 real_t i = len_length * ((rand_i > 0.5) ? 0.5 : ((rand_i < -0.5) ? -0.5 : rand_i)) + e.x;
	 real_t j = len_length * ((rand_j > 0.5) ? 0.5 : ((rand_j < -0.5) ? -0.5 : rand_j)) + e.y;
	 Vector3 e_rand = Vector3(i, j, e.z);
	 */

	HitRecord record;
	record.hit = false;

	// Set t0 to be a small number for recursive ray cast.
	real_t t0 = (depth > 1) ? 1e-3 : 0;
	real_t t1 = std::numeric_limits<double>::infinity();

	// Uses kd-tree to find hit point.
	scene->get_kd_tree()->hit(ray, t0, t1, 0, &record);
	shade(ray, record, depth, refractive_indices, result);
}

real_t Raytracer::shade_diffuse(const Ray ray, const HitRecord record, unsigned int depth,
		std::vector<real_t> refractive_indices, TracingResult& result) const {
	// The indices of lights hitting this point.
	std::vector<unsigned int> light_number;
	// The shadow rays from the point to the light sources.
	std::vector<Ray> shadow_rays(scene->num_lights());
	real_t diffuse_prob = 0;

	// Checks whether the object is transparent. Only shades ambient and diffuse color
	// for objects with refractive indices equal to zero.
	if (record.shade_factors.refractive_index < 1e-3) {
		if (result.is_photon) {
			diffuse_prob = max_component(result.photon.color * record.shade_factors.diffuse) /
							max_component(result.photon.color);
			real_t rand_diffuse = ((double) rand() / (RAND_MAX));
			// Diffuse reflection.
			if (rand_diffuse < diffuse_prob) {
				real_t incident = -dot(ray.d, record.normal);
				// TODO back side?
				Color3 color_store = result.photon.color;
				//std::cout << diffuse_prob << std::endl;
				//std::cout << "before: " << result.photon.color.r << std::endl;
				result.photon.color = result.photon.color *
								record.shade_factors.diffuse * (1. / diffuse_prob);
				//std::cout << "after: " << result.photon.color.r << std::endl;
				Vector3 rand_dir(random_gaussian(), random_gaussian(), random_gaussian());
				rand_dir = normalize(rand_dir);
				rand_dir = (dot(rand_dir, record.normal) < 0) ? -rand_dir : rand_dir;
				Ray diffuse_ray(record.hit_point, rand_dir);
				trace_point(diffuse_ray, depth + 1, refractive_indices, result);
				if (result.photon.position.x == std::numeric_limits<double>::infinity()) {
					if (depth > 1 && depth < 4) {
						result.photon.cosine_angle = incident;
						result.photon.position = record.hit_point;
						result.photon.color = color_store;
					}
				}
			}
			return diffuse_prob;
		}

		//result.color = record.shade_factors.ambient * scene->ambient_light;

		if (dot(ray.d, record.normal) < 0) {
			for (size_t i = 0; i < scene->num_lights(); i++) {
				// Uses randomness to sample the volumn light source.
				// Random vector generate by the light source is a vector from the center to a random
				// point on its surface.
				Vector3 random_light_point =
						scene->get_lights()[i].generate_random_point();
				Vector3 shadow_d = random_light_point - record.hit_point;
				Ray shadow_ray(record.hit_point, shadow_d);

				shadow_rays[i] = shadow_ray;
				if (!scene->get_kd_tree()->hit(shadow_ray, 1e-3, 1, 0, NULL))
					light_number.push_back(i);
			}
		}

		// The point is not in shadow.
		if (light_number.size() >= 1) {
			// Render diffuse light for each light sources.
			for (std::vector<unsigned int>::iterator itr = light_number.begin();
					itr != light_number.end(); itr++) {
				const SphereLight* light = (scene->get_lights() + *itr);
				real_t dot_n_l = dot(record.normal,
						normalize(shadow_rays[*itr].d));
				real_t diffuse_cos = (dot_n_l > 0) ? dot_n_l : 0;
				result.color += record.shade_factors.diffuse * light->color
						* diffuse_cos;
			}
		}

		result.color += shade_photon_map(record.hit_point);
	} else {
		result.color = Color3::Black();
	}

	return diffuse_prob;
}

Color3 Raytracer::shade_photon_map(const Vector3 hit_position) const {
	std::vector<Photon*> k_nn(K_NN_NUM);
	Photon center;
	real_t weight_k = 5;
	Color3 global_color = Color3::Black();
	center.position = hit_position;
	k_nn.clear();

	real_t radius = scene->get_global_photon_map()->find_k_nn(center, K_NN_NUM, k_nn);
	for (size_t i = 0; i < k_nn.size(); i++) {
		real_t weight = 1 - length(k_nn[i]->position - center.position) / weight_k / radius;
		//std::cout << radius << std::endl;
		global_color += k_nn[i]->color * std::fmax(k_nn[i]->cosine_angle, 0) * weight;
		//std::cout << k_nn[i]->color.r << "\t" << k_nn[i]->cosine_angle << "\t"
				//<< weight << std::endl;
		//std::cout << global_color.r << std::endl;
	}
	//std::cout << std::endl;
	global_color *= 1. / (PI * radius * radius * (1 - 2.f / 3 / weight_k)) * 200;
	global_color = clamp(global_color, 1e-3, 0.75);
	//std::cout << global_color.r << std::endl;

	k_nn.clear();
	Color3 local_color_sum = Color3::Black();
	for (size_t j = 0; j < scene->num_geometries(); j++) {
		if (scene->get_geometry(j)->is_refractive()) {
			Color3 obj_color = Color3::Black();
			radius = scene->get_geometry(j)->photon_map->find_k_nn(center, 10, k_nn);
			//std::cout << radius << std::endl;
			for (size_t i = 0; i < k_nn.size(); i++) {
				//std::cout << k_nn[i]->color.r << "\t"
					//	<< k_nn[i]->color.g << "\t"
						//<< k_nn[i]->color.b << std::endl;
				obj_color += k_nn[i]->color * std::fmax(k_nn[i]->cosine_angle, 0);
			}
			obj_color *= 1. / (PI * radius * radius) * 100;
			local_color_sum += obj_color;
			//std::cout << radius << "\t" << local_color_sum.r << std::endl;
		}
	}
	local_color_sum = clamp(local_color_sum, 1e-3, 1.0f);

	return global_color;// + local_color_sum;
}

/**
 * Shades a hit point using the given eye ray.
 * @param ray                 the eye ray.
 * @param record         the struct keeping the hit information.
 * @param depth                the tracing depth for reflective ray and refractive ray.
 */
void Raytracer::shade(const Ray ray, const HitRecord record, unsigned int depth,
		std::vector<real_t> refractive_indices, TracingResult& result) const {
	real_t diffuse_prob = 0;

	// The eye ray hits nothing.
	if (!record.hit) {
		result.color = scene->background_color;
		return ;
	}

	diffuse_prob = shade_diffuse(ray, record, depth, refractive_indices, result);
	// Photon has been "stored". Position has been set.
	if (result.is_photon &&
			result.photon.position.x < std::numeric_limits<double>::infinity()) {
		return ;
	}

	// Refraction and reflection.
	Vector3 normal = record.normal;
	real_t dot_n_d = dot(ray.d, normal);
	Vector3 t(0, 0, 0);
	real_t R = 1;

	// If the normal vector is not in the same side with d, reverse the normal.
	if (dot_n_d > 0) {
		normal = -normal;
		dot_n_d = -dot_n_d;
	}

	// Refraction happens when refractive index changes on two sides.
	if (record.shade_factors.refractive_index > 1e-3
			&& refractive_indices.back() > 1e-3) {
		real_t n = refractive_indices.back();
		real_t n_t;

		// Checks to see whether the ray is entering object or leaving it.
		if (!is_same_material(n, scene->refractive_index)) {
			n_t = refractive_indices[refractive_indices.size() - 2];
		} else {
			n_t = record.shade_factors.refractive_index;
		}

		real_t n_over_n_t = n / n_t;
		real_t cos_t_square = 1
				- n_over_n_t * n_over_n_t * (1 - dot_n_d * dot_n_d);

		// Total internal refraction will skip this step.
		if (cos_t_square > 0) {
			real_t cos_t = sqrt(cos_t_square);
			t = n_over_n_t * (ray.d - dot_n_d * normal) - normal * cos_t;
			R = (n_t - 1) / (n_t + 1);
			R *= R;
			cos_t = (cos_t < -dot_n_d) ? cos_t : -dot_n_d;

			R = R + (1 - R) * (1 - cos_t) * (1 - cos_t) * (1 - cos_t)
							* (1 - cos_t) * (1 - cos_t);
		}
	}

	// Randomly trace refective ray or refractive ray using Russian roulette.
	real_t rand_r = ((double) rand() / (RAND_MAX));
	// The ray is reflected with probability R.
	if (rand_r < R) {
		// Only traces reflective ray for objects with positive specular color.
		if (record.shade_factors.specular != Color3::Black()) {
			// Ray tracing: specular: 1; diffuse: 0.
			real_t specular_prob = 1;
			real_t total = 1 - diffuse_prob;
			if (result.is_photon) {
				specular_prob = max_component(result.photon.color * record.shade_factors.specular) /
											max_component(result.photon.color);
				specular_prob /= total;
			}
			real_t rand_specular = ((double) rand() / (RAND_MAX));

			if (R < 1 || rand_specular < specular_prob ||
					(result.is_photon && depth < 2)) {
				// Uses randomness to create glossy reflection.
				Vector3 r = ray.d - 2 * dot_n_d * normal;
				Vector3 u = normalize(cross(normal, r));
				Vector3 v = cross(r, v);

				// The parameter is 2e-2.
				real_t rand_u = -record.shade_factors.shininess / 2e2 +
								random_gaussian() * 2e-2;
				real_t rand_v = -record.shade_factors.shininess / 2e2 +
								random_gaussian() * 2e-2;
				TracingResult reflective_result;
				reflective_result.is_photon = result.is_photon;
				reflective_result.photon = result.photon;
				Ray reflective_ray(record.hit_point, r + rand_u * u + rand_v * v);
				trace_point(reflective_ray, depth + 1, refractive_indices, reflective_result);

				if (!result.is_photon) {
					Color3 reflective = record.shade_factors.specular * reflective_result.color;
					result.color += reflective;
				}
				else {
					result.photon = reflective_result.photon;
				}
			}
			else {
				if (depth > 1) {
					result.photon.cosine_angle = -dot(record.normal, ray.d);
					result.photon.position = record.hit_point;
				}
			}
		}
		else if (result.is_photon) {
			result.photon.cosine_angle = -dot(record.normal, ray.d);
			result.photon.position = record.hit_point;
		}
	}
	else {
		// Traces refractive ray with probability (1 - R).
		// Maintains the refractive indices stack.
		if (!is_same_material(refractive_indices.back(),
				scene->refractive_index)) {
			refractive_indices.pop_back();
		} else {
			refractive_indices.push_back(record.shade_factors.refractive_index);
		}

		Ray refractive_ray(record.hit_point, t);
		TracingResult refractive_result;
		refractive_result.is_photon = result.is_photon;
		refractive_result.photon = result.photon;
		trace_point(refractive_ray, depth + 1, refractive_indices, refractive_result);
		if (!result.is_photon)
			result.color += refractive_result.color;

		else
			result.photon = refractive_result.photon;
	}

	// Adds texture.
	if (!result.is_photon)
		result.color *= record.shade_factors.texture;
}

/**
 * Raytraces some portion of the scene. Should raytrace for about
 * max_time duration and then return, even if the raytrace is not copmlete.
 * The results should be placed in the given buffer.
 * @param buffer The buffer into which to place the color data. It is
 * 32-bit RGBA (4 bytes per pixel), in row-major order.
 * @param max_time, If non-null, the maximum suggested time this
 * function raytrace before returning, in seconds. If null, the raytrace
 * should run to completion.
 * @return true if the raytrace is complete, false if there is more
 * work to be done.
 */
bool Raytracer::raytrace(unsigned char* buffer, real_t* max_time) {
	// TODO Add any modifications to this algorithm, if needed.
#pragma omp critical
	{
		if (!photon_emission_flag) {
			std::cout << "emission starts" << std::endl;
			emit_photons();
			photon_emission_flag = true;
		}
	}

	static const size_t PRINT_INTERVAL = 64;

	// the time in milliseconds that we should stop
	unsigned int end_time = 0;
	bool is_done;

	if (max_time) {
		// convert duration to milliseconds
		unsigned int duration = (unsigned int) (*max_time * 1000);
		end_time = SDL_GetTicks() + duration;
	}

	// until time is up, run the raytrace. we render an entire group of
	// rows at once for simplicity and efficiency.
	for (; !max_time || end_time > SDL_GetTicks(); current_row += STEP_SIZE) {
		// we're done if we finish the last row
		is_done = current_row >= height;
		// break if we finish
		if (is_done)
			break;

		int loop_upper = std::min(current_row + STEP_SIZE, height);

		// This tells OpenMP that this loop can be parallelized.
#pragma omp parallel for
		for (int c_row = current_row; c_row < loop_upper; c_row++) {
			//for (int c_row = current_row; c_row < 1; c_row++) {
			/*
			 * This defines a critical region of code that should be
			 * executed sequentially.
			 */
#pragma omp critical
			{
				if (c_row % PRINT_INTERVAL == 0)
					printf("Raytracing (Row %d)\n", c_row);
			}

			for (size_t x = 0; x < width; x++) {
				//for (size_t x = 0; x < 1; x++) {
				// trace a pixel
				Color3 color = trace_pixel(scene, x, c_row, width, height);
				// write the result to the buffer, always use 1.0 as the alpha
				color.to_array(&buffer[4 * (c_row * width + x)]);
			}
		}
	}

	if (is_done)
		printf("Done raytracing!\n");

	return is_done;
}

void Raytracer::emit_photons() const {
	// Global photon map.
	std::vector<real_t> light_powers(scene->num_lights());
	real_t total_power = 0;
	for (size_t i = 0; i < scene->num_lights(); i++) {
		SphereLight* light = const_cast<SphereLight*>(scene->get_light(i));
		Vector3 color_vector = Vector3(light->color.r, light->color.g, light->color.b);
		real_t power = length(color_vector);
		total_power += power;
		light_powers[i] = power;
	}

	// TODO parallel

	for (size_t i = 0; i < scene->num_lights(); i++) {
		size_t num_photons = std::ceil(light_powers[i] * PHOTON_NUM / total_power);
		SphereLight* light = const_cast<SphereLight*>(scene->get_light(i));

		for (size_t j = 0; j < num_photons; j++) {
			Ray photon_ray(light->position, normalize(light->generate_random_point() - light->position));
			TracingResult photon_result;
			photon_result.is_photon = true;
			photon_result.photon.color = light->color * (1. / num_photons) * GLOBAL_RATIO;
			photon_result.photon.position = Vector3(std::numeric_limits<real_t>::infinity(),
													std::numeric_limits<real_t>::infinity(),
													std::numeric_limits<real_t>::infinity());
			std::vector<real_t> refractive_indices;
			refractive_indices.push_back(scene->refractive_index);
			trace_point(photon_ray, 1, refractive_indices, photon_result);
			if (photon_result.photon.position.x < std::numeric_limits<double>::infinity())
				scene->get_global_photon_map()->add_photon(photon_result.photon);
		}
		std::cout << "global emission done" << std::endl;
		for (size_t j = 0; j < scene->num_geometries(); j++) {
			if (scene->get_geometry(j)->is_refractive()) {
				for (size_t k = 0; k < OBJECT_PHOTON_NUM; k++) {
					size_t rand_box = scene->get_geometry(j)->get_boundingboxs().size() * random();
					Vector3 rand_point = scene->get_geometry(j)->get_boundingboxs()[rand_box]->generate_rand_point();
					Ray photon_ray(light->position, normalize(rand_point - light->position));
					TracingResult photon_result;
					photon_result.is_photon = true;
					photon_result.photon.color = light->color * (1 - GLOBAL_RATIO) * (1. / OBJECT_PHOTON_NUM);
					photon_result.photon.position = Vector3(
							std::numeric_limits<real_t>::infinity(),
							std::numeric_limits<real_t>::infinity(),
							std::numeric_limits<real_t>::infinity());
					std::vector<real_t> refractive_indices;
					refractive_indices.push_back(scene->refractive_index);
					trace_point(photon_ray, 1, refractive_indices, photon_result);
					if (photon_result.photon.position.x < std::numeric_limits<double>::infinity())
						scene->get_geometry(j)->photon_map->add_photon(photon_result.photon);
				}
				scene->get_geometry(j)->photon_map->build_kd_tree();
			}
		}
	}
	std::cout << "before global" << std::endl;
	scene->build_global_photon_map();
	std::cout << "emission done" << std::endl;
}

} /* _462 */
