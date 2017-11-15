#include "test_macros.hpp"

#include <matrix/math.hpp>

using namespace matrix;

int main()
{
    float eps = 1e-6f;

    Vector3f a(1, 0, 0);
    Vector3f b(0, 1, 0);
    Vector3f c = a.cross(b);
    TEST(isEqual(c, Vector3f(0,0,1)));
    c = a % b;
    TEST(isEqual(c, Vector3f(0,0,1)));
    Matrix<float, 3, 1> d(c);
    Vector3f e(d);
    TEST(isEqual(e, d));
    float data[] = {4, 5, 6};
    Vector3f f(data);
    TEST(isEqual(f, Vector3f(4, 5, 6)));

    TEST(isEqual(a + b, Vector3f(1, 1, 0)));
    TEST(isEqual(a - b, Vector3f(1, -1, 0)));
    TEST(isEqualF(a * b, 0.0f));
    TEST(isEqual(-a, Vector3f(-1, 0, 0)));
    TEST(isEqual(a.unit(), a));
    TEST(isEqual(a.unit(), a.normalized()));
    TEST(isEqual(a*2.0, Vector3f(2, 0, 0)));

    // Vector3 copyTo
    Vector3f v(1, 2, 3);
    float dst[3] = {};
    v.copyTo(dst);
    TEST(fabs(v(0) - dst[0]) < eps);
    TEST(fabs(v(1) - dst[1]) < eps);
    TEST(fabs(v(2) - dst[2]) < eps);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
