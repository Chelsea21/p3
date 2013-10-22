/**
 * @file model.hpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_MODEL_HPP_
#define _462_SCENE_MODEL_HPP_

#include "scene/scene.hpp"
#include "scene/mesh.hpp"
#include "scene/boundingbox.hpp"

namespace _462 {

/**
 * A mesh of triangles.
 */
class Model : public Geometry
{
public:

    const Mesh* mesh;
    const Material* material;
    std::vector<Boundingbox*> boundingbox_ptrs;

    Model();
    virtual ~Model();

    virtual void render() const;
    virtual bool hit(const Ray ray, const real_t start, const real_t end,
    			const unsigned int model_index, HitRecord* record_ptr);
    virtual size_t num_models() const;
    virtual std::vector<Boundingbox*> get_boundingbox() const;
    virtual void construct_boundingbox();
};


} /* _462 */

#endif /* _462_SCENE_MODEL_HPP_ */

