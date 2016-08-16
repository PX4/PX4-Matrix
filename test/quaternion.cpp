#include <stdio.h>
#include <math.h>
#include <cfloat>

#include "../matrix/math.hpp"
#include "test_macros.hpp"

using namespace matrix;

template class Vector<float, 5>;

int main() {
    Quaternion<float> q90x(cos(radians(90.0f/2.0f)), sin(radians(90.0f/2.0f)), 0, 0);
    //Quaternion<float> q90y(cos(radians(90.0f/2.0f)), 0, sin(radians(90.0f/2.0f)), 0);
    Quaternion<float> q90y(Vector3f(0, 1, 0), radians(90.0f));
    Quaternion<float> q90z(cos(radians(90.0f/2.0f)), 0, 0, sin(radians(90.0f/2.0f)));

    // all positive rotations are clockwise

    // test rotations of a vector
    {
        Vector3f vx(1, 0, 0);
        Vector3f rx = q90x * vx;
        TEST(is_equal(rx(0), 1.0f) && is_equal(rx(1), 0.0f) && is_equal(rx(2), 0.0f));
        rx = q90y * vx;
        TEST(is_equal(rx(0), 0.0f) && is_equal(rx(1), 0.0f) && is_equal(rx(2), -1.0f));
        rx = q90z * vx;
        TEST(is_equal(rx(0), 0.0f) && is_equal(rx(1), 1.0f) && is_equal(rx(2), 0.0f));
    }

    // test integration
    // note that precision is going to be off because numerical integration is always off so we use a rather large epsilon!
    {
        size_t it = 100;
        Vector3f w(0, M_PI / 2.0f, 0);
        Quaternion<float> qq;
        for(size_t c = 0; c < it; c++) {
            qq.applyRates(w, 1.0f / it);
        }

        Vector3f v(1, 0, 0);
        Vector3f r = qq * v;
        // q should now be about the same as q90y
        TEST(is_equal(r(0), 0.0f, 0.01f) && is_equal(r(1), 0.0f, 0.01f) && is_equal(r(2), -1.0f, 0.01f));
    }

    {
        size_t it = 100;
        Vector3f w(0, M_PI / 2.0f, 0);
        Quaternion<float> qq = q90x;
        for(size_t j = 0; j < 5; j++) {
            // lets rotate it one full circle and then 90 degrees
            for(size_t c = 0; c < it; c++) {
                qq.applyRates(w, 1.0f / it);
            }
        }
        qq.normalize();

        Vector3f v(0, 1, 0);
        Vector3f r = qq * v;
        // q should now represent rotation of 90 degrees around x and then 90 degrees around y (in the frame of q90x). So resulting vector points down along positive z.
        TEST(is_equal(r(0), 0.0f, 0.001f) && is_equal(r(1), 0.0f, 0.001f) && is_equal(r(2), 1.0f, 0.001f));
    }

    {
        size_t it = 100;
        Vector3f w(M_PI / 2.0f, 0, 0);
        Quaternion<float> qq = q90y;
        for(size_t c = 0; c < it; c++) {
            qq.applyRates(w, 1.0f / it);
        }

        Vector3f vx(0, 0, 1);
        Vector3f rx = qq * vx;
        // vector should now point along negative x axis (above first applies rotations of 90 around y and then 90 around x).
        TEST(is_equal(rx(0), 0.0f, 0.01f) && is_equal(rx(1), -1.0f, 0.01f) && is_equal(rx(2), 0.0f, 0.01f));
    }

    // test euler
    {
        Euler<float> e = eulerAngles(q90x);
        TEST(is_equal(degrees(e.roll()), 90.0f) && is_equal(degrees(e.pitch()), 0.0f) && is_equal(degrees(e.yaw()), 0.0f));
        e = eulerAngles(q90y);
        // pointing in direction of gimbal lock so we use a larger error tolerance
        TEST(is_equal(degrees(e.roll()), 0.0f) && is_equal(degrees(e.pitch()), 90.0f, 0.2f) && is_equal(degrees(e.yaw()), 0.0f));
        e = eulerAngles(q90z);
        TEST(is_equal(degrees(e.roll()), 0.0f) && is_equal(degrees(e.pitch()), 0.0f) && is_equal(degrees(e.yaw()), 90.0f));

    }

	// test lerp and slerp
	{
		Euler<float> ex = eulerAngles(slerp(Quaternion<float>(), q90x, 0.5f, 0.0f)); 
		Euler<float> ey = eulerAngles(lerp(Quaternion<float>(), Quaternion<float>(cos(radians(0.5f/2)), 0, sin(radians(0.5f/2)), 0), 0.5f)); 
		TEST(is_equal(degrees(ex.roll()), 45.0f) && is_equal(ex.pitch(), 0) && is_equal(ex.yaw(), 0)); 
		TEST(is_equal(ey.roll(), 0) && is_equal(degrees(ey.pitch()), 0.25f, 0.02f) && is_equal(ey.yaw(), 0)); 
	}
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
