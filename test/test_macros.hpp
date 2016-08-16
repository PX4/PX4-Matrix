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

template<typename Type>
inline Type radians(Type deg) {
    return Type(M_PI) / Type(180.0) * deg;
}

template<typename Type>
inline Type degrees(Type rad) {
    return rad / (Type(M_PI) / Type(180.0));
}

inline bool is_equal(float a, float b, float e) {
    return fabsf(a - b) < e;
}

inline bool is_equal(float a, float b) {
    return is_equal(a, b, FLT_EPSILON);
}

inline bool is_equal(double a, double b, double e) {
    return fabs(a - b) < e;
}

inline bool is_equal(double a, double b) {
    return is_equal(a, b, DBL_EPSILON);
}


#define TEST(X) if(!(X)) { fprintf(stderr, "test failed on %s:%d\n", __FILE__, __LINE__); return -1;}
