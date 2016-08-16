/**
 * @file Quaternion.hpp
 *
 * All rotations and axis systems follow the right-hand rule.
 *
 * In order to rotate a vector v by a righthand rotation defined by the quaternion q
 * one can use the following operation:
 * v_rotated = q^(-1) * [0;v] * q
 * where q^(-1) represents the inverse of the quaternion q.
 * The product z of two quaternions z = q1 * q2 represents an intrinsic rotation
 * in the order of first q1 followed by q2.
 * The first element of the quaternion
 * represents the real part, thus, a quaternion representing a zero-rotation
 * is defined as (1,0,0,0).
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

template <typename Type>
class AxisAngle;


template<typename Type>
Quaternion<Type> normalize(const Quaternion<Type> &o);

/**
 * Quaternion class
 *
 * The rotation between two coordinate frames is
 * described by this class.
 */
template<typename Type>
class Quaternion : public Vector<Type, 4>
{
public:
    virtual ~Quaternion() {};

    typedef Matrix<Type, 4, 1> Matrix41;
    typedef Matrix<Type, 3, 1> Matrix31;

    /**
     * Constructor from array
     *
     * @param data_ array
     */
    Quaternion(const Type *data_) :
        Vector<Type, 4>(data_)
    {
    }

    /**
     * Standard constructor
     */
    Quaternion() :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = 1;
        q(1) = 0;
        q(2) = 0;
        q(3) = 0;
    }

    /**
     * Constructor from Matrix41
     *
     * @param other Matrix41 to copy
     */
    Quaternion(const Matrix41 &other) :
        Vector<Type, 4>(other)
    {
    }

    /**
     * Constructor from dcm
     *
     * Instance is initialized from a dcm representing coordinate transformation
     * from frame 2 to frame 1.
     *
     * @param dcm dcm to set quaternion to
     */
    Quaternion(const Dcm<Type> &dcm) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = Type(0.5) * Type(sqrt(Type(1) + dcm(0, 0) +
                                     dcm(1, 1) + dcm(2, 2)));
        q(1) = Type((dcm(2, 1) - dcm(1, 2)) /
                    (Type(4) * q(0)));
        q(2) = Type((dcm(0, 2) - dcm(2, 0)) /
                    (Type(4) * q(0)));
        q(3) = Type((dcm(1, 0) - dcm(0, 1)) /
                    (Type(4) * q(0)));
    }

    /**
     * Constructor from euler angles
     *
     * This sets the instance to a quaternion representing coordinate transformation from
     * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
     * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
     *
     * @param euler euler angle instance
     */
    Quaternion(const Euler<Type> &euler) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        Type cosPhi_2 = Type(cos(euler.phi() / (Type)2.0));
        Type cosTheta_2 = Type(cos(euler.theta() / (Type)2.0));
        Type cosPsi_2 = Type(cos(euler.psi() / (Type)2.0));
        Type sinPhi_2 = Type(sin(euler.phi() / (Type)2.0));
        Type sinTheta_2 = Type(sin(euler.theta() / (Type)2.0));
        Type sinPsi_2 = Type(sin(euler.psi() / (Type)2.0));
        q(0) = cosPhi_2 * cosTheta_2 * cosPsi_2 +
               sinPhi_2 * sinTheta_2 * sinPsi_2;
        q(1) = sinPhi_2 * cosTheta_2 * cosPsi_2 -
               cosPhi_2 * sinTheta_2 * sinPsi_2;
        q(2) = cosPhi_2 * sinTheta_2 * cosPsi_2 +
               sinPhi_2 * cosTheta_2 * sinPsi_2;
        q(3) = cosPhi_2 * cosTheta_2 * sinPsi_2 -
               sinPhi_2 * sinTheta_2 * cosPsi_2;
    }

    /**
     * Quaternion from AxisAngle
     *
     * @param aa axis-angle vector
     */
    Quaternion(const AxisAngle<Type> &aa) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        Type angle = aa.norm();
        Vector<Type, 3> axis = aa.unit();
        if (angle < (Type)1e-10) {
            q(0) = (Type)1.0;
            q(1) = q(2) = q(3) = 0;
        } else {
            Type magnitude = sinf(angle / 2.0f);
            q(0) = cosf(angle / 2.0f);
            q(1) = axis(0) * magnitude;
            q(2) = axis(1) * magnitude;
            q(3) = axis(2) * magnitude;
        }
    }


    /**
     * Constructor from quaternion values
     *
     * Instance is initialized from quaternion values representing coordinate
     * transformation from frame 2 to frame 1.
     * A zero-rotation quaternion is represented by (1,0,0,0).
     *
     * @param a set quaternion value 0
     * @param b set quaternion value 1
     * @param c set quaternion value 2
     * @param d set quaternion value 3
     */
    Quaternion(Type a, Type b, Type c, Type d) :
        Vector<Type, 4>()
    {
        Quaternion &q = *this;
        q(0) = a;
        q(1) = b;
        q(2) = c;
        q(3) = d;
    }

    Quaternion(const Vector<Type, 3> &axis, Type angle) {
        from_axis_angle(axis, angle);
    }

    /**
     * Quaternion multiplication operator
     *
     * @param q quaternion to multiply with
     * @return product
     */
    Quaternion operator*(const Quaternion &q) const
    {
        const Quaternion &p = *this;
        Quaternion r;
#if 0
        r(0) = p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3);
        r(1) = p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2);
        r(2) = p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1);
        r(3) = p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
#endif
        // Hamilton notation is like this:
        r(0) = p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3);
        r(1) = p(0) * q(1) + p(1) * q(0) + p(2) * q(3) - p(3) * q(2);
        r(2) = p(0) * q(2) - p(1) * q(3) + p(2) * q(0) + p(3) * q(1);
        r(3) = p(0) * q(3) + p(1) * q(2) - p(2) * q(1) + p(3) * q(0);
        return r;
    }

    Quaternion operator+(const Quaternion &o) const {
        const Quaternion &p = *this;
        return Quaternion(p(0) + o(0), p(1) + o(1), p(2) + o(2), p(3) + o(3));
    }

    /**
     * Self-multiplication operator
     *
     * @param other quaternion to multiply with
     */
    void operator*=(const Quaternion & other)
    {
        Quaternion &self = *this;
        self = self * other;
    }

    /**
     * Scalar multiplication operator
     *
     * @param scalar scalar to multiply with
     * @return product
     */
    Quaternion operator*(Type s) const
    {
        const Quaternion &q = *this;
        return Quaternion(q(0) * s, q(1) * s, q(2) * s, q(3) * s);
    }

    /**
     * Scalar self-multiplication operator
     *
     * @param scalar scalar to multiply with
     */
    void operator*=(Type scalar)
    {
        Quaternion &q = *this;
        q = q * scalar;
    }

    /**
     * Computes the derivative
     *
     * @param w direction
     */
    Matrix41 derivative(const Matrix31 &w) const
    {
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
        v(1) = w(0, 0);
        v(2) = w(1, 0);
        v(3) = w(2, 0);
        return Q * v * Type(0.5);
    }

    /**
    * Apply an angular velocity vector to this quaternion and normalize result
    */
    void applyRates(const Matrix31 &w, float dt) {
        Quaternion<Type> &p = *this;
        p = p + derivative(w) * dt;

        // automatically normalize if error is larger than some constant (normalize before use for best result)
        // NOTE: if normalization is done always then it creates horrible precision because of floating point roundoffs
        if(fabs(p.dot(p) - 1.0f) > 0.005) {
            p.normalize();
        }
    }

    /**
     * Invert quaternion
     */
    void invert() {
        Quaternion &q = *this;
        q(1) *= -1;
        q(2) *= -1;
        q(3) *= -1;
    }

    Type dot(const Quaternion<Type> &o) const {
        const Quaternion &p = *this;
        return p(0) * o(0) + p(1) * o(1) + p(2) * o(2) + p(3) * o(3);
    }

    /**
     * Invert quaternion
     *
     * @return inverted quaternion
     */
    Quaternion inversed() const {
        const Quaternion &q = *this;
        Quaternion ret;
        ret(0) = q(0);
        ret(1) = -q(1);
        ret(2) = -q(2);
        ret(3) = -q(3);
        return ret;
    }

    /**
     * Rotate quaternion from rotation vector
     * TODO replace with AxisAngle call
     *
     * @param vec rotation vector
     */
    void rotate(const Vector<Type, 3> &vec)
    {
        Quaternion res;
        res.from_axis_angle(vec);
        (*this) = (*this) * res;
    }

    /**
     * Rotation quaternion from vector
     *
     * The axis of rotation is given by vector direction and
     * the angle is given by the norm.
     *
     * @param vec rotation vector
     * @return quaternion representing the rotation
     */
    void from_axis_angle(Vector<Type, 3> vec)
    {
        Quaternion &q = *this;
        Type theta = vec.norm();

        if (theta < (Type)1e-10) {
            q(0) = (Type)1.0;
            q(1) = q(2) = q(3) = 0;
            return;
        }

        vec /= theta;
        from_axis_angle(vec, theta);
    }

    /**
     * Rotation quaternion from axis and angle
     * XXX DEPRECATED, use AxisAngle class
     *
     * @param axis axis of rotation
     * @param theta scalar describing angle of rotation
     * @return quaternion representing the rotation
     */
    void from_axis_angle(const Vector<Type, 3> &axis, Type theta)
    {
        Quaternion &q = *this;

        if (theta < (Type)1e-10) {
            q(0) = (Type)1.0;
            q(1) = q(2) = q(3) = 0;
        }

        Type magnitude = sinf(theta / 2.0f);

        q(0) = cosf(theta / 2.0f);
        q(1) = axis(0) * magnitude;
        q(2) = axis(1) * magnitude;
        q(3) = axis(2) * magnitude;
    }


    /**
     * Rotation vector from quaternion
     * XXX DEPRECATED, use AxisAngle class
     *
     * The axis of rotation is given by vector direction and
     * the angle is given by the norm.
     *
     * @return vector, direction representing rotation axis and norm representing angle
     */
    Vector<Type, 3> to_axis_angle()
    {
        Quaternion &q = *this;
        Type axis_magnitude = Type(sqrt(q(1) * q(1) + q(2) * q(2) + q(3) * q(3)));
        Vector<Type, 3> vec;
        vec(0) = q(1);
        vec(1) = q(2);
        vec(2) = q(3);

        if (axis_magnitude >= (Type)1e-10) {
            vec = vec / axis_magnitude;
            vec = vec * wrap_pi((Type)2.0 * atan2f(axis_magnitude, q(0)));
        }

        return vec;
    }
};

typedef Quaternion<float> Quatf;
typedef Quaternion<float> Quaternionf;

// set this quaternion to the result of the linear interpolation between two quaternions
template<typename Type>
Quaternion<Type> lerp(const Quaternion<Type> &q1, const Quaternion<Type> &q2, Type time)
{
    const Type scale = 1.0f - time;
    return (q1*scale) + (q2*time);
}


// set this quaternion to the result of the interpolation between two quaternions
template<typename Type>
Quaternion<Type> slerp(const Quaternion<Type> &_q1, const Quaternion<Type> &q2, float time, float threshold) {
    Quaternion<Type> q1 = _q1;
    Type angle = q1.dot(q2);
    // make sure we use the short rotation
    if (angle < 0.0f)
    {
        q1 *= -1.0f;
        angle *= -1.0f;
    }

    // spherical interpolation
    if (angle <= (1-threshold)) {
        const Type theta = acosf(angle);
        const Type invsintheta = 1.0f / sinf(theta); //reciprocal(sinf(theta));
        const Type scale = sinf(theta * (1.0f-time)) * invsintheta;
        const Type invscale = sinf(theta * time) * invsintheta;
        return (q1*scale) + (q2*invscale);
    } else // linear interploation
        return lerp(q1,q2,time);
}

// for scalar * quaternion cases
template<typename Type>
Quaternion<Type> operator*(const Type s, const Quaternion<Type> &q) {
    return q.operator*(s);
}

template<typename Type>
Quaternion<Type> normalize(const Quaternion<Type> &o) {
    return o.normalized();
}

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
