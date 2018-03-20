/**
 * @file Scalar.hpp
 *
 * Defines conversion of matrix to scalar.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template<typename Type>
class Scalar
{
public:
    Scalar() : _value()
    {
    }

    Scalar(const Matrix<Type, 1, 1> & other)
    {
        _value = other(0,0);
    }

    Scalar(Type other)
    {
        _value = other;
    }

    operator Type &()
    {
        return _value;
    }

    operator Matrix<Type, 1, 1>() const {
        Matrix<Type, 1, 1> m;
        m(0, 0) = _value;
        return m;
    }

    operator Vector<Type, 1>() const {
        Vector<Type, 1> m;
        m(0) = _value;
        return m;
    }

private:
    Type _value;

};

typedef Scalar<float> Scalarf;

template<typename Type, size_t  M, size_t N>
Scalar<Type> operator*(const Matrix<Type, 1, N> &A, const Matrix<Type, N, 1> &B)
{
    return Scalar<Type> {A * B};
}

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
