/**
 * @file triangle.cpp
 * @brief Function definitions for the Triangle class.
 *
 * @author Eric Butler (edbutler)
 */

#include "scene/triangle.hpp"
#include "application/opengl.hpp"

namespace _462 {

Triangle::Triangle()
{
    vertices[0].material = 0;
    vertices[1].material = 0;
    vertices[2].material = 0;
}

Triangle::~Triangle() { }

void Triangle::render() const
{
    bool materials_nonnull = true;
    for ( int i = 0; i < 3; ++i )
        materials_nonnull = materials_nonnull && vertices[i].material;

    // this doesn't interpolate materials. Ah well.
    if ( materials_nonnull )
        vertices[0].material->set_gl_state();

    glBegin(GL_TRIANGLES);

    glNormal3dv( &vertices[0].normal.x );
    glTexCoord2dv( &vertices[0].tex_coord.x );
    glVertex3dv( &vertices[0].position.x );

    glNormal3dv( &vertices[1].normal.x );
    glTexCoord2dv( &vertices[1].tex_coord.x );
    glVertex3dv( &vertices[1].position.x);

    glNormal3dv( &vertices[2].normal.x );
    glTexCoord2dv( &vertices[2].tex_coord.x );
    glVertex3dv( &vertices[2].position.x);

    glEnd();

    if ( materials_nonnull )
        vertices[0].material->reset_gl_state();
}

bool Triangle::hit(const Ray ray, const real_t start, const real_t end,
			const unsigned int model_index, HitRecord* record_ptr) {
	(void) model_index;
	Ray transformed_ray = ray.transform(this->invMat);
	Vector3 a_minus_b = vertices[0].position - vertices[1].position;
	Vector3 a_minus_c = vertices[0].position - vertices[2].position;
	Vector3 a_minus_e = vertices[0].position - transformed_ray.e;

	//std::cout << transformed_ray.e.x << "\t" << transformed_ray.e.y << "\t" << transformed_ray.e.z << std::endl;

	real_t a = a_minus_b.x;
	real_t b = a_minus_b.y;
	real_t c = a_minus_b.z;
	real_t d = a_minus_c.x;
	real_t e = a_minus_c.y;
	real_t f = a_minus_c.z;
	real_t g = transformed_ray.d.x;
	real_t h = transformed_ray.d.y;
	real_t i = transformed_ray.d.z;
	real_t j = a_minus_e.x;
	real_t k = a_minus_e.y;
	real_t l = a_minus_e.z;

	real_t ei_minus_hf = e * i - h * f;
	real_t gf_minus_di = g * f - d * i;
	real_t dh_minus_eg = d * h - e * g;
	real_t ak_minus_jb = a * k - j * b;
	real_t jc_minus_al = j * c - a * l;
	real_t bl_minus_kc = b * l - k * c;

	real_t M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg;
	real_t time = (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc) / -M;
	if (time <= start || time >= end) {
		return false;
	}

	real_t beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg) / M;
	if (beta < 0 || beta > 1)
		return false;

	real_t gamma = (i * ak_minus_jb + h * jc_minus_al + g * bl_minus_kc) / M;
	if (gamma < 0 || gamma > 1 - beta)
		return false;

	if (record_ptr == NULL)
		return true;

	record_ptr->time = time;
	record_ptr->material_ptr = vertices[0].material;
	record_ptr->hit_point = ray.e + record_ptr->time * ray.d;
	record_ptr->normal = normalize(this->normMat * vertices[0].normal);

	record_ptr->tex_coord = interpolate<Vector2>(beta, gamma,
			vertices[0].tex_coord, vertices[1].tex_coord, vertices[2].tex_coord);
	record_ptr->tex_coord = record_ptr->material_ptr->clap_texture(record_ptr->tex_coord);

	record_ptr->shade_factors.ambient = interpolate<Color3>(beta, gamma,
			vertices[0].material->ambient, vertices[1].material->ambient, vertices[2].material->ambient);
	record_ptr->shade_factors.diffuse = interpolate<Color3>(beta, gamma,
				vertices[0].material->diffuse, vertices[1].material->diffuse, vertices[2].material->diffuse);
	record_ptr->shade_factors.specular = interpolate<Color3>(beta, gamma,
				vertices[0].material->specular, vertices[1].material->specular, vertices[2].material->specular);
	record_ptr->shade_factors.shininess = interpolate<real_t>(beta, gamma,
			vertices[0].material->shininess, vertices[1].material->shininess, vertices[2].material->shininess);
	record_ptr->shade_factors.refractive_index = interpolate<real_t>(beta, gamma,
			vertices[0].material->refractive_index, vertices[1].material->refractive_index, vertices[2].material->refractive_index);

	return true;
}

size_t Triangle::num_models() const {
	return 1;
}

Boundingbox* const Triangle::get_boundingbox() const {
	return &boundingbox;
}

void Triangle::construct_boundingbox() {
	boundingbox.mat = mat;

	real_t inf = std::numeric_limits<double>::infinity();
	Vector3 newMinPoint(-inf, -inf, -inf);
	Vector3 newMaxPoint(inf, inf, inf);

	for (size_t i = 0; i < 3; i++) {
		Vector3 transformed = this->mat.transform_point(vertices[i].position);
		for (size_t j = 0; j < 3; j++) {
			newMinPoint[j] =
					(transformed[j] < newMinPoint[j]) ?
							transformed[j] : newMinPoint[j];
			newMaxPoint[j] =
					(transformed[j] > newMaxPoint[j]) ?
							transformed[j] : newMaxPoint[j];
		}
	}

	boundingbox.minPoint = newMinPoint;
	boundingbox.maxPoint = newMaxPoint;
	boundingbox.construct_boundingbox();
}

} /* _462 */
