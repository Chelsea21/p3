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
					const bool check_only, HitRecord& record) const {
	Ray transformed_ray = this->invMat * ray;
	real_t a = vertices[0].position.x - vertices[1].position.x;
	real_t b = vertices[0].position.y - vertices[1].position.y;
	real_t c = vertices[0].position.z - vertices[1].position.z;
	real_t d = vertices[0].position.x - vertices[2].position.x;
	real_t e = vertices[0].position.y - vertices[2].position.y;
	real_t f = vertices[0].position.z - vertices[2].position.z;
	real_t g = transformed_ray.d.x;
	real_t h = transformed_ray.d.y;
	real_t i = transformed_ray.d.z;
	real_t j = vertices[0].position.x - transformed_ray.e.x;
	real_t k = vertices[0].position.y - transformed_ray.e.y;
	real_t l = vertices[0].position.z - transformed_ray.e.z;

	real_t ei_minus_hf = e * i - h * f;
	real_t gf_minus_di = g * f - d * i;
	real_t dh_minus_eg = d * h - e * g;
	real_t ak_minus_jb = a * k - j * b;
	real_t jc_minus_al = j * c - a * l;
	real_t bl_minus_kc = b * l - k * c;

	real_t M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg;
	real_t time = (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc) / -M;
	if (time < start || time > end) {
		return false;
	}

	real_t beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg) / M;
	if (beta < 0 || beta > 1)
		return false;

	real_t gamma = (i * ak_minus_jb + h * jc_minus_al + g * bl_minus_kc) / M;
	if (gamma < 0 || gamma > 1 - beta)
		return false;
	record.time = time;
	//record.material_ptr = this->
	record.hit_point = ray.e + record.time * ray.d;
	record.normal = normalize(record.hit_point - position);
}

} /* _462 */
