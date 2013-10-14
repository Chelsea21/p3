/**
 * @file triangle.hpp
 * @brief Class definition for Triangle.
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_TRIANGLE_HPP_
#define _462_SCENE_TRIANGLE_HPP_

#include "scene/scene.hpp"

namespace _462 {

/**
 * a triangle geometry.
 * Triangles consist of 3 vertices. Each vertex has its own position, normal,
 * texture coordinate, and material. These should all be interpolated to get
 * values in the middle of the triangle.
 * These values are all in local space, so it must still be transformed by
 * the Geometry's position, orientation, and scale.
 */
class Triangle : public Geometry
{
public:

    struct Vertex
    {
        // note that position and normal are in local space
        Vector3 position;
        Vector3 normal;
        Vector2 tex_coord;
        const Material* material;
    };

    // the triangle's vertices, in CCW order
    Vertex vertices[3];

    Triangle();
    virtual ~Triangle();
    virtual void render() const;
    virtual bool hit(const Ray ray, const real_t start, const real_t end,
    				const unsigned int model_index, HitRecord* record_ptr);
    virtual size_t num_models() const;
};

template<typename T> inline T interpolate(const real_t beta, const real_t gamma,
										const T a, const T b, const T c) {
	return a + (b - a) * beta + (c - a) * gamma;
}

} /* _462 */

#endif /* _462_SCENE_TRIANGLE_HPP_ */
