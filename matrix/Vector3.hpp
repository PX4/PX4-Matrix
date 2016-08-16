/**
 * @file Vector3.hpp
 *
 * 3D vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M>
class Vector;

template<typename Type>
class Dcm;

template<typename Type>
class Vector3 : public Vector<Type, 3>
{
public:

    typedef Matrix<Type, 3, 1> Matrix31;

    virtual ~Vector3() {};

    Vector3() :
        Vector<Type, 3>()
    {
    }

    Vector3(const Matrix31 & other) :
        Vector<Type, 3>(other)
    {
    }

    Vector3(const Type *data_) :
        Vector<Type, 3>(data_)
    {
    }

    Vector3(Type x, Type y, Type z) : Vector<Type, 3>()
    {
        Vector3 &v(*this);
        v(0) = x;
        v(1) = y;
        v(2) = z;
    }

    Vector3 cross(const Matrix31 & b)  const {
        const Vector3 &a(*this);
        Vector3 c;
        c(0) = a(1)*b(2,0) - a(2)*b(1,0);
        c(1) = -a(0)*b(2,0) + a(2)*b(0,0);
        c(2) = a(0)*b(1,0) - a(1)*b(0,0);
        return c;
    }

    Vector3 operator%(const Matrix31 & b) const {
        return (*this).cross(b);
    }

    Dcm<Type> hat() const {    // inverse to Dcm.vee() operation
        const Vector3 &v(*this);
        Dcm<Type> A;
        A(0,0) = 0;
        A(0,1) = -v(2);
        A(0,2) = v(1);
        A(1,0) = v(2);
        A(1,1) = 0;
        A(1,2) = -v(0);
        A(2,0) = -v(1);
        A(2,1) = v(0);
        A(2,2) = 0;
        return A;
    }

};

typedef Vector3<float> Vector3f;

template <typename Type>
class Quaternion;

template<typename Type>
Vector3<Type> operator* (const Quaternion<Type> &q, const Vector3<Type>& v) {
    /*
    // nVidia SDK implementation
    Vector3<Type> uv, uuv;
    const float qdata[3] = {q(1), q(2), q(3)};
    Vector3<Type> qvec(qdata);
    uv = qvec.cross(v);
    uuv = qvec.cross(uv);
    uv *= (2.0f * q(0));
    uuv *= 2.0f;

    return v + uv + uuv;
    */
    Quaternion<Type> vq(0, v(0), v(1), v(2));
    Quaternion<Type> r = q * vq * q.inversed();
    return Vector3f(r(1), r(2), r(3));
}


} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
