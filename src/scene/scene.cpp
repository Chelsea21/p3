/**
 * @file scene.cpp
 * @brief Function definitions for scenes.
 *
 * @author Eric Butler (edbutler)
 * @author Kristin Siu (kasiu)
 */

#include "scene/scene.hpp"
#include "math/random462.hpp"

namespace _462 {


Geometry::Geometry():
    position(Vector3::Zero()),
    orientation(Quaternion::Identity()),
    scale(Vector3::Ones())
{

}

Geometry::~Geometry() { }

bool Geometry::initialize()
{
	make_inverse_transformation_matrix(&invMat, position, orientation, scale);
	make_transformation_matrix(&mat, position, orientation, scale);
	make_normal_matrix(&normMat, mat);

	return true;
}

SphereLight::SphereLight():
    position(Vector3::Zero()),
    color(Color3::White()),
	radius(real_t(0))
{
    attenuation.constant = 1;
    attenuation.linear = 0;
    attenuation.quadratic = 0;
}

Vector3 SphereLight::generate_random_point() const {
	Vector3 rand_dir(random_gaussian(), random_gaussian(), random_gaussian());
	rand_dir = normalize(rand_dir) * radius;
	return position + rand_dir;
}

Scene::Scene()
{
    reset();
}

Scene::~Scene()
{
    reset();
}

bool Scene::initialize()
{
	bool res = true;
	for (unsigned int i = 0; i < num_geometries(); i++)
		res &= geometries[i]->initialize();
	return res;
}


Geometry* const* Scene::get_geometries() const
{
    return geometries.empty() ? NULL : &geometries[0];
}

Geometry* Scene::get_geometry(unsigned int geometry_num) const {
	if (geometry_num < num_geometries())
		return geometries[geometry_num];
	return NULL;
}

size_t Scene::num_geometries() const
{
    return geometries.size();
}

const SphereLight* Scene::get_lights() const
{
    return point_lights.empty() ? NULL : &point_lights[0];
}

size_t Scene::num_lights() const
{
    return point_lights.size();
}

Material* const* Scene::get_materials() const
{
    return materials.empty() ? NULL : &materials[0];
}

size_t Scene::num_materials() const
{
    return materials.size();
}

Mesh* const* Scene::get_meshes() const
{
    return meshes.empty() ? NULL : &meshes[0];
}

size_t Scene::num_meshes() const
{
    return meshes.size();
}

void Scene::reset()
{
    for ( GeometryList::iterator i = geometries.begin(); i != geometries.end(); ++i ) {
        delete *i;
    }
    for ( MaterialList::iterator i = materials.begin(); i != materials.end(); ++i ) {
        delete *i;
    }
    for ( MeshList::iterator i = meshes.begin(); i != meshes.end(); ++i ) {
        delete *i;
    }

    geometries.clear();
    materials.clear();
    meshes.clear();
    point_lights.clear();

    camera = Camera();

    background_color = Color3::Black();
    ambient_light = Color3::Black();
}

void Scene::add_geometry( Geometry* g )
{
    geometries.push_back( g );
}

void Scene::add_material( Material* m )
{
    materials.push_back( m );
}

void Scene::add_mesh( Mesh* m )
{
    meshes.push_back( m );
}

void Scene::add_light( const SphereLight& l )
{
    point_lights.push_back( l );
}

} /* _462 */

