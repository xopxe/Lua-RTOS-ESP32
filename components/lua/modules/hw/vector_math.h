// derived from https://github.com/arkanis/single-header-file-c-libs/

#ifndef __VECTOR_MATH_H_
#define __VECTOR_MATH_H_

#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { float x, y, z; } vec3_t;
static inline vec3_t vec3(float x, float y, float z)        { return (vec3_t){ x, y, z }; }

static inline vec3_t v3_add   (vec3_t a, vec3_t b)          { return (vec3_t){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static inline vec3_t v3_adds  (vec3_t a, float s)           { return (vec3_t){ a.x + s,   a.y + s,   a.z + s   }; }
static inline vec3_t v3_sub   (vec3_t a, vec3_t b)          { return (vec3_t){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static inline vec3_t v3_subs  (vec3_t a, float s)           { return (vec3_t){ a.x - s,   a.y - s,   a.z - s   }; }
static inline vec3_t v3_mul   (vec3_t a, vec3_t b)          { return (vec3_t){ a.x * b.x, a.y * b.y, a.z * b.z }; }
static inline vec3_t v3_muls  (vec3_t a, float s)           { return (vec3_t){ a.x * s,   a.y * s,   a.z * s   }; }
static inline vec3_t v3_div   (vec3_t a, vec3_t b)          { return (vec3_t){ a.x / b.x, a.y / b.y, a.z / b.z }; }
static inline vec3_t v3_divs  (vec3_t a, float s)           { return (vec3_t){ a.x / s,   a.y / s,   a.z / s   }; }
static inline float  v3_length(vec3_t v)                    { return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);          }
static inline vec3_t v3_norm  (vec3_t v);
static inline float  v3_dot   (vec3_t a, vec3_t b)          { return a.x*b.x + a.y*b.y + a.z*b.z;                 }
static inline vec3_t v3_proj  (vec3_t v, vec3_t onto);
static inline vec3_t v3_cross (vec3_t a, vec3_t b);

//
// 3D vector functions header implementation
//

static inline vec3_t v3_norm(vec3_t v) {
	float len = v3_length(v);
	if (len > 0)
		return (vec3_t){ v.x / len, v.y / len, v.z / len };
	else
		return (vec3_t){ 0, 0, 0};
}

static inline vec3_t v3_proj(vec3_t v, vec3_t onto) {
	return v3_muls(onto, v3_dot(v, onto) / v3_dot(onto, onto));
}

static inline vec3_t v3_cross(vec3_t a, vec3_t b) {
	return (vec3_t){
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

//
// Matrix functions header implementation
//

typedef union {
	// The first index is the column index, the second the row index. The memory
	// layout of nested arrays in C matches the memory layout expected by OpenGL.
	float m[3][3];
	// OpenGL expects the first 4 floats to be the first column of the matrix.
	// So we need to define the named members column by column for the names to
	// match the memory locations of the array elements.
	struct {
		float m00, m01, m02;
		float m10, m11, m12;
		float m20, m21, m22;
	};
} mat3_t;


static inline mat3_t mat3(
	float m00, float m10, float m20,
	float m01, float m11, float m21,
	float m02, float m12, float m22
);

vec3_t mat3_mul_vec3(mat3_t matrix, vec3_t position) {
	vec3_t result = vec3(
		matrix.m00 * position.x + matrix.m10 * position.y + matrix.m20 * position.z,
		matrix.m01 * position.x + matrix.m11 * position.y + matrix.m21 * position.z,
		matrix.m02 * position.x + matrix.m12 * position.y + matrix.m22 * position.z
	);


	return result;
}

static inline mat3_t mat3(
	float m00, float m10, float m20,
	float m01, float m11, float m21,
	float m02, float m12, float m22
) {
	return (mat3_t){
		.m[0][0] = m00, .m[1][0] = m10, .m[2][0] = m20,
		.m[0][1] = m01, .m[1][1] = m11, .m[2][1] = m21,
		.m[0][2] = m02, .m[1][2] = m12, .m[2][2] = m22
	};
}


#endif //__VECTOR_MATH_H_
