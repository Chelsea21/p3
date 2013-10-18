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
#define MAX_THREADS 1

static const unsigned STEP_SIZE = 8;

Raytracer::Raytracer() :
		scene(0), width(0), height(0), current_row(0), num_samples(1) {
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
 *  false is returned.
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

	//const SphereLight* lights = scene->get_lights();

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
		res += trace_point(scene, scene->camera.get_position(), Ray::get_pixel_dir(i, j), 1,
							refractive_indices);
	}
	return res * (real_t(1) / num_samples);
}

Color3 Raytracer::trace_point(const Scene* scene, Vector3 e, Vector3 d, unsigned int depth,
							std::vector<real_t> refractive_indices) const {
	if (depth > MAX_DEPTH)
		return Color3::Black();

	Ray r(e, d);
	HitRecord record;
	record.material_ptr = NULL;

	// Set t0 to be a small number for recursive ray cast.
	real_t t0 = (depth > 1) ? 1e-3 : 0;
	real_t t1 = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < scene->num_geometries(); i++) {
		for (size_t j = 0; j < scene->get_geometry(i)->num_models(); j++) {
			if (scene->get_geometry(i)->hit(r, t0, t1, j, &record)) {
				if (is_air(refractive_indices.back(), scene->refractive_index) ||
						abs(refractive_indices.back() - record.shade_factors.refractive_index) < 1e-6)
					t1 = record.time;
			}
		}
	}
	Color3 res = shade(r, record, depth, refractive_indices);

	return res;
}

Color3 Raytracer::shade(const Ray ray, const HitRecord record, unsigned int depth,
			std::vector<real_t> refractive_indices) const {
	Color3 result;
	std::vector<unsigned int> light_number;
	std::vector<Ray> light_rays(scene->num_lights());

	if (record.material_ptr == NULL) {
		result = scene->background_color;
		return result;
	}

	if (record.shade_factors.refractive_index < 1e-3) {
		result = record.shade_factors.ambient * scene->ambient_light;
		if (dot(ray.d, record.normal) < 0) {
			for (size_t i = 0; i < scene->num_lights(); i++) {
				// Random vector generate by the light source is a vector from the center to a random
				// point on its surface.
				Vector3 random_light_point =
						scene->get_lights()[i].generate_random_point();
				Vector3 shadow_d = random_light_point - record.hit_point;
				Ray shadow_ray(record.hit_point, shadow_d);

				size_t j = 0;
				for (; j < scene->num_geometries(); j++) {
					// D vector here is the vector pointing from the object surface to the light source.
					for (size_t k = 0; k < scene->get_geometry(j)->num_models(); k++) {
						if (scene->get_geometry(j)->hit(shadow_ray, 1e-3, 1, k, NULL)) {
							goto hit_found;
						} else
							light_rays[i] = shadow_ray;
					}
				}
				hit_found:
				if (j == scene->num_geometries())
					light_number.push_back(i);
			}
		}

		// Non-shadow.
		if (light_number.size() >= 1) {
			// Render diffuse and specular light for each light sources.
			for (std::vector<unsigned int>::iterator itr = light_number.begin();
					itr != light_number.end(); itr++) {
				const SphereLight* light = (scene->get_lights() + *itr);
				Vector3 h = normalize(normalize(light_rays[*itr].d) - ray.d);
				real_t dot_n_l = dot(record.normal,
						normalize(light_rays[*itr].d));
				real_t diffuse_cos = (dot_n_l > 0) ? dot_n_l : 0;

				result += record.shade_factors.diffuse * light->color * diffuse_cos;
			}
		}
	}
	else {
		result = Color3::Black();
	}

	// Refraction and reflection.
	Vector3 normal = record.normal;
	real_t dot_n_d = dot(ray.d, normal);
	Vector3 t(0, 0, 0);
	real_t R = 1;

	if (dot_n_d > 0) {
		normal = -normal;
		dot_n_d = -dot_n_d;
	}

	// Refraction.
	if (record.shade_factors.refractive_index > 1e-3
			&& refractive_indices.back() > 1e-3) {
		real_t n = refractive_indices.back();
		real_t n_t;

		// checked
		if (!is_air(n, scene->refractive_index)) {
			n_t = refractive_indices[refractive_indices.size() - 2];
		} else {
			n_t = record.shade_factors.refractive_index;
		}

		real_t n_over_n_t = n / n_t;
		real_t cos_t_square = 1 - n_over_n_t * n_over_n_t * (1 - dot_n_d * dot_n_d);

		// Total internal refraction will skip this step.
		if (cos_t_square > 0) {
			real_t cos_t = sqrt(cos_t_square);
			t = n_over_n_t * (ray.d - dot_n_d * normal)
					- normal * cos_t;
			R = (n_t - 1) / (n_t + 1);
			R *= R;
			cos_t = (cos_t < -dot_n_d) ? cos_t : -dot_n_d;

			R = R + (1 - R) * (1 - cos_t) * (1 - cos_t) * (1 - cos_t)
							* (1 - cos_t) * (1 - cos_t);
		}
	}

	// Randomly trace refective ray or refractive ray.
	real_t rand_r = 1. * random();
	if (rand_r < R) {
		// TODO glossy reflection.
		if (record.shade_factors.specular != Color3::Black()) {
			Vector3 r = ray.d - 2 * dot_n_d * normal;

			result += record.shade_factors.specular
					* trace_point(scene, record.hit_point, normalize(r), ++depth,
							refractive_indices);
		}
	}
	else {
		if (!is_air(refractive_indices.back(), scene->refractive_index)) {
			refractive_indices.pop_back();
		} else {
			refractive_indices.push_back(record.shade_factors.refractive_index);
		}

		result += trace_point(scene, record.hit_point, normalize(t), ++depth,
					refractive_indices);
	}

	return result;
}

/**
 * Raytraces some portion of the scene. Should raytrace for about
 * max_time duration and then return, even if the raytrace is not copmlete.
 * The results should be placed in the given buffer.
 * @param buffer The buffer into which to place the color data. It is
 *  32-bit RGBA (4 bytes per pixel), in row-major order.
 * @param max_time, If non-null, the maximum suggested time this
 *  function raytrace before returning, in seconds. If null, the raytrace
 *  should run to completion.
 * @return true if the raytrace is complete, false if there is more
 *  work to be done.
 */
bool Raytracer::raytrace(unsigned char* buffer, real_t* max_time) {
	// TODO Add any modifications to this algorithm, if needed.

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

} /* _462 */
