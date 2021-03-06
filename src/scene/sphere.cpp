/**
 * @file sphere.cpp
 * @brief Function defnitions for the Sphere class.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#include "scene/sphere.hpp"
#include "scene/kd_tree.hpp"
#include "application/opengl.hpp"
#include <cmath>

namespace _462 {

#define SPHERE_NUM_LAT 80
#define SPHERE_NUM_LON 100

#define SPHERE_NUM_VERTICES ( ( SPHERE_NUM_LAT + 1 ) * ( SPHERE_NUM_LON + 1 ) )
#define SPHERE_NUM_INDICES ( 6 * SPHERE_NUM_LAT * SPHERE_NUM_LON )
// index of the x,y sphere where x is lat and y is lon
#define SINDEX(x,y) ((x) * (SPHERE_NUM_LON + 1) + (y))
#define VERTEX_SIZE 8
#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5

static unsigned int Indices[SPHERE_NUM_INDICES];
static float Vertices[VERTEX_SIZE * SPHERE_NUM_VERTICES];

static void init_sphere()
{
    static bool initialized = false;
    if ( initialized )
        return;

    for ( int i = 0; i <= SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j <= SPHERE_NUM_LON; j++ ) {
            real_t lat = real_t( i ) / SPHERE_NUM_LAT;
            real_t lon = real_t( j ) / SPHERE_NUM_LON;
            float* vptr = &Vertices[VERTEX_SIZE * SINDEX(i,j)];

            vptr[TCOORD_OFFSET + 0] = lon;
            vptr[TCOORD_OFFSET + 1] = 1-lat;

            lat *= PI;
            lon *= 2 * PI;
            real_t sinlat = sin( lat );

            vptr[NORMAL_OFFSET + 0] = vptr[VERTEX_OFFSET + 0] = sinlat * sin( lon );
            vptr[NORMAL_OFFSET + 1] = vptr[VERTEX_OFFSET + 1] = cos( lat ),
            vptr[NORMAL_OFFSET + 2] = vptr[VERTEX_OFFSET + 2] = sinlat * cos( lon );
        }
    }

    for ( int i = 0; i < SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j < SPHERE_NUM_LON; j++ ) {
            unsigned int* iptr = &Indices[6 * ( SPHERE_NUM_LON * i + j )];

            unsigned int i00 = SINDEX(i,  j  );
            unsigned int i10 = SINDEX(i+1,j  );
            unsigned int i11 = SINDEX(i+1,j+1);
            unsigned int i01 = SINDEX(i,  j+1);

            iptr[0] = i00;
            iptr[1] = i10;
            iptr[2] = i11;
            iptr[3] = i11;
            iptr[4] = i01;
            iptr[5] = i00;
        }
    }

    initialized = true;
}

Sphere::Sphere()
    : radius(0), material(0) {
	this->photon_map = new KdTree();
}

Sphere::~Sphere() {
	delete this->photon_map;
}

void Sphere::render() const
{
    // create geometry if we haven't already
    init_sphere();

    if ( material )
        material->set_gl_state();

    // just scale by radius and draw unit sphere
    glPushMatrix();
    glScaled( radius, radius, radius );
    glInterleavedArrays( GL_T2F_N3F_V3F, VERTEX_SIZE * sizeof Vertices[0], Vertices );
    glDrawElements( GL_TRIANGLES, SPHERE_NUM_INDICES, GL_UNSIGNED_INT, Indices );
    glPopMatrix();

    if ( material )
        material->reset_gl_state();
}

/**
 * Hit function inherited from Geometry class.
 */
bool Sphere::hit(const Ray ray, const real_t start, const real_t end,
    			const unsigned int model_index, HitRecord* record_ptr) {
	(void) model_index;

	// Transform the ray into object coordinate.
	Ray transformed_ray = ray.transform(this->invMat);

	Vector3 e_minus_c = transformed_ray.e;
	real_t a = dot(transformed_ray.d, transformed_ray.d);
	real_t c = dot(e_minus_c, e_minus_c) - radius * radius;
	real_t b_half = dot(transformed_ray.d, e_minus_c);
	real_t discriminant = b_half * b_half - a * c;

	if (discriminant < 0)
		return false;

	real_t numerator = -b_half;
	real_t t1;
	real_t t2;
	real_t t;

	if (discriminant > 0) {
		real_t sqrt_discriminant = std::sqrt(discriminant);
		t1 = (numerator - sqrt_discriminant) / a;
		t2 = (numerator + sqrt_discriminant) / a;
	}
	else
		t = t1 = t2 = numerator / a;
	if (t1 > end || t2 < start) {
		return false;
	}
	t = (t1 < start) ? t2 : t1;

	if (t == end || t == start)
		return false;

	if (record_ptr != NULL) {
		record_ptr->time = t;
		record_ptr->hit = true;
		Vector3 transformed_point = transformed_ray.e + record_ptr->time * transformed_ray.d;
		Vector3 transformed_normal = normalize(transformed_point - Vector3(0, 0, 0));

		record_ptr->hit_point = ray.e + record_ptr->time * ray.d;
		record_ptr->normal = normalize(this->normMat * transformed_normal);

		// Computes the texture coordinate.
		real_t theta = std::atan2(transformed_normal.z, transformed_normal.x);
		real_t phi = std::asin(transformed_normal.y);

		record_ptr->tex_coord = Vector2(0.5 + theta / PI / 2, 0.5 - phi / PI);
		record_ptr->tex_coord = this->material->clap_texture(record_ptr->tex_coord);
		record_ptr->shade_factors.texture = this->material->get_texture_pixel(record_ptr->tex_coord);

		record_ptr->shade_factors.ambient = this->material->ambient;
		record_ptr->shade_factors.diffuse = this->material->diffuse;
		record_ptr->shade_factors.refractive_index = this->material->refractive_index;
		record_ptr->shade_factors.shininess = this->material->shininess;
		record_ptr->shade_factors.specular = this->material->specular;
	}

	return true;
}

size_t Sphere::num_models() const {
	return 1;
}

std::vector<Boundingbox*> Sphere::get_boundingboxs() const {
	return std::vector<Boundingbox*>(1, const_cast<Boundingbox*>(&boundingbox));
}

void Sphere::construct_boundingbox() {
	boundingbox.mat = mat;
	boundingbox.geometry = this;
	boundingbox.model_index = 0;
	boundingbox.minPoint = Vector3(-radius, -radius, -radius);
	boundingbox.maxPoint = Vector3(radius, radius, radius);
	boundingbox.construct_boundingbox();
	boundingbox.isLoose = true;
}

bool Sphere::is_refractive() const {
	return material->refractive_index > 1e-3;
}

} /* _462 */

