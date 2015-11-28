/**
 * @file Quaternion.hpp
 *
 * A quaternion class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"
#include "helper_functions.hpp"

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Euler;

template<typename Type>
class Quaternion : public Vector<Type, 4>
{
public:
    virtual ~Quaternion() {};

    typedef Matrix<Type, 4, 1> Matrix41;
    typedef Matrix<Type, 3, 1> Matrix31;

    Quaternion() :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = 1;
        q(1) = 0;
        q(2) = 0;
        q(3) = 0;
    }

    Quaternion(const Matrix41 & other) :
        Vector<Type, 4>(other)
    {
    }

    Quaternion(const Dcm<Type> & dcm) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = Type(0.5 * sqrt(1 + dcm(0, 0) +
                               dcm(1, 1) + dcm(2, 2)));
        q(1) = Type((dcm(2, 1) - dcm(1, 2)) /
                    (4 * q(0)));
        q(2) = Type((dcm(0, 2) - dcm(2, 0)) /
                    (4 * q(0)));
        q(3) = Type((dcm(1, 0) - dcm(0, 1)) /
                    (4 * q(0)));
    }

    Quaternion(const Euler<Type> & euler) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        Type cosPhi_2 = Type(cos(euler.phi() / 2.0));
        Type cosTheta_2 = Type(cos(euler.theta() / 2.0));
        Type cosPsi_2 = Type(cos(euler.psi() / 2.0));
        Type sinPhi_2 = Type(sin(euler.phi() / 2.0));
        Type sinTheta_2 = Type(sin(euler.theta() / 2.0));
        Type sinPsi_2 = Type(sin(euler.psi() / 2.0));
        q(0) = cosPhi_2 * cosTheta_2 * cosPsi_2 +
               sinPhi_2 * sinTheta_2 * sinPsi_2;
        q(1) = sinPhi_2 * cosTheta_2 * cosPsi_2 -
               cosPhi_2 * sinTheta_2 * sinPsi_2;
        q(2) = cosPhi_2 * sinTheta_2 * cosPsi_2 +
               sinPhi_2 * cosTheta_2 * sinPsi_2;
        q(3) = cosPhi_2 * cosTheta_2 * sinPsi_2 -
               sinPhi_2 * sinTheta_2 * cosPsi_2;
    }

    Quaternion(Type a, Type b, Type c, Type d) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = a;
        q(1) = b;
        q(2) = c;
        q(3) = d;
    }

    Quaternion operator*(const Quaternion &q) const
    {
        const Quaternion &p = *this;
        Quaternion r;
        r(0) = p(0)*q(0) - p(1)*q(1) - p(2)*q(2) - p(3)*q(3);
        r(1) = p(0)*q(1) + p(1)*q(0) - p(2)*q(3) + p(3)*q(2);
        r(2) = p(0)*q(2) + p(1)*q(3) + p(2)*q(0) - p(3)*q(1);
        r(3) = p(0)*q(3) - p(1)*q(2) + p(2)*q(1) + p(3)*q(0);
        return r;
    }

    void operator*=(const Quaternion & other)
    {
        Quaternion &self = *this;
        self = self * other;
    }

    Matrix41 derivative(const Matrix31 & w) const {
        const Quaternion &q = *this;
        Type dataQ[] = {
            q(0), -q(1), -q(2), -q(3),
            q(1),  q(0), -q(3),  q(2),
            q(2),  q(3),  q(0), -q(1),
            q(3), -q(2),  q(1),  q(0)
        };
        Matrix<Type, 4, 4> Q(dataQ);
        Vector<Type, 4> v;
        v(0) = 0;
        v(1) = w(0,0);
        v(2) = w(1,0);
        v(3) = w(2,0);
        return Q * v * Type(0.5);
    }

    void invert() {
        Quaternion &q = *this;
        q(1) *= -1;
        q(2) *= -1;
        q(3) *= -1;
    }

    Quaternion inversed() {
        Quaternion &q = *this;
        Quaternion ret;
        ret(0) = q(0);
        ret(1) = q(1);
        ret(2) = q(2);
        ret(3) = q(3);
        return ret;
    }

    void rotate(const Vector<float, 3> &v) {
        Quaternion<float> r;
        r.from_axis_angle(v);
        (*this) = (*this) * r;
    }

    void from_axis_angle(Vector<float, 3> v) {
        Quaternion<float> &q = *this;
        float theta = v.norm();
        if(theta < 1.0e-12f) {
            q(0) = 1.0f;
            q(1)=q(2)=q(3)=0.0f;
            return;
        }
        v /= theta;
        from_axis_angle(v,theta);
    }

    void from_axis_angle(const Vector<float, 3> &axis, float theta) {
        Quaternion<float> &q = *this;
        if(theta < 1.0e-12f) {
            q(0) = 1.0f;
            q(1)=q(2)=q(3)=0.0f;
        }
        float st2 = sinf(theta/2.0f);

        q(0) = cosf(theta/2.0f);
        q(1) = axis(0) * st2;
        q(2) = axis(1) * st2;
        q(3) = axis(2) * st2;
    }

    Vector<Type, 3> to_axis_angle() {
        Quaternion &q = *this;
        float l = sqrt(q(1) * q(1) + q(2) * q(2) + q(3) * q(3));
        Vector<Type, 3> v;
        v(0) = q(1);
        v(1) = q(2);
        v(2) = q(3);
        if(l >= (Type)1.0e-12) {
            v /= l;
            v *= wrap_pi((Type)2.0 * atan2f(l,q(0)));
        }
        return v;
    }

    Matrix<Type, 3, 3> to_dcm() {
        Quaternion &q = *this;
        Matrix<Type, 3, 3> R;
        Type aSq = q(0) * q(0);
        Type bSq = q(1) * q(1);
        Type cSq = q(2) * q(2);
        Type dSq = q(3) * q(3);
        R._data[0][0] = aSq + bSq - cSq - dSq;
        R._data[0][1] = (Type)2.0 * (q(1) * q(2) - q(0) * q(3));
        R._data[0][2] = (Type)2.0 * (q(0) * q(2) + q(1) * q(3));
        R._data[1][0] = (Type)2.0 * (q(1) * q(2) + q(0) * q(3));
        R._data[1][1] = aSq - bSq + cSq - dSq;
        R._data[1][2] = (Type)2.0 * (q(2) * q(3) - q(0) * q(1));
        R._data[2][0] = (Type)2.0 * (q(1) * q(3) - q(0) * q(2));
        R._data[2][1] = (Type)2.0 * (q(0) * q(1) + q(2) * q(3));
        R._data[2][2] = aSq - bSq - cSq + dSq;
        return R;
    }
};

typedef Quaternion<float> Quatf;

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
