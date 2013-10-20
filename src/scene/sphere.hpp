/**
 * @file sphere.hpp
 * @brief Class defnition for Sphere.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_SPHERE_HPP_
#define _462_SCENE_SPHERE_HPP_

#include "scene/scene.hpp"
#include "scene/boundingbox.hpp"

namespace _462 {

/**
 * A sphere, centered on its position with a certain radius.
 */
class Sphere : public Geometry
{
public:

    real_t radius;
    const Material* material;

    Boundingbox boundingbox;

    Sphere();
    virtual ~Sphere();
    virtual void render() const;
    virtual bool hit(const Ray ray, const real_t start, const real_t end,
        			const unsigned int model_index, HitRecord* record_ptr);
    virtual size_t num_models() const;
    virtual Boundingbox* get_boundingbox() const;
    virtual void construct_boundingbox();
};

} /* _462 */

#endif /* _462_SCENE_SPHERE_HPP_ */

