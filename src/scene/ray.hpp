#ifndef _462_SCENE_RAY_HPP_
#define _462_SCENE_RAY_HPP_

#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "math/camera.hpp"
#include "scene/material.hpp"
#include <string>
#include <vector>

namespace _462 {

struct ShadeFactors {
	Color3 ambient;
	Color3 diffuse;
	Color3 specular;
	real_t shininess;
	real_t refractive_index;
};

struct HitRecord {
	real_t time;
	Vector3 hit_point;
	Vector3 normal;
	Vector2 tex_coord;
	const Material* material_ptr;
	ShadeFactors shade_factors;
};

class Ray
{

public:
    Vector3 e;
    Vector3 d;
    Ray();
    Ray(Vector3 e, Vector3 d);

    static Vector3 get_pixel_dir(real_t x, real_t y);
	static void init(const Camera& camera);

	Ray transform(const Matrix4 matrix) const;
};

}
#endif
