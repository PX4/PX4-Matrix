/**
 * @file test_marcos.hpp
 *
 * Helps with cmake testing.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */
#pragma once

#include <cstdio>
#include <cfloat>
#include <math.h>

inline float radians(float deg) {
    return M_PI / 180.0f * deg;
}

inline float degrees(float rad) {
    return rad / (M_PI / 180.0f);
}

inline bool is_equal(float a, float b, float e) {
    return fabsf(a - b) < e;
}
inline bool is_equal(float a, float b) {
    return is_equal(a, b, FLT_EPSILON);
}

#define TEST(X) if(!(X)) { fprintf(stderr, "test failed on %s:%d\n", __FILE__, __LINE__); return -1;}
