/**
 * @file Slice.hpp
 *
 * A simple matrix template library.
 *
 * @author Julian Kent < julian@auterion.com >
 */

#pragma once

#include "math.hpp"


namespace matrix {

template<typename Type, size_t M, size_t N>
class Matrix;

template<typename Type, size_t M>
class Vector;

template<typename M1, typename M2>
Matrix<typename M1::Scalar,M1::Rows,M2::Cols> mult(const M1& left, const M2& right);

template <typename Type, size_t P, size_t Q, size_t M, size_t N>
class Slice {
public:

    using TypeDefinition = Slice<Type, P, Q, M, N>;
    using Scalar = Type;
    static constexpr size_t Rows = P;
    static constexpr size_t Cols = Q;


    Slice(size_t x0, size_t y0, const Matrix<Type, M, N>* data) :
        _x0(x0),
        _y0(y0),
        _data(const_cast<Matrix<Type, M, N>*>(data)) {
        static_assert(P <= M, "Slice rows bigger than backing matrix");
        static_assert(Q <= N, "Slice cols bigger than backing matrix");
    }

    Type operator()(size_t i, size_t j) const
    {
        return (*_data)(_x0 + i, _y0 + j);
    }

    Type &operator()(size_t i, size_t j)
    {
        return (*_data)(_x0 + i, _y0 + j);
    }

    template<size_t MM, size_t NN>
    Slice<Type, P, Q, M, N>& operator=(const Slice<Type, P, Q, MM, NN>& in_matrix)
    {
        Slice<Type, P, Q, M, N>& self = *this;
        for (size_t i = 0; i < P; i++) {
            for (size_t j = 0; j < Q; j++) {
                self(i, j) = in_matrix(i, j);
            }
        }
        return self;
    }

    Slice<Type, P, Q, M, N>& operator=(const Matrix<Type, P, Q>& in_matrix)
    {
        Slice<Type, P, Q, M, N>& self = *this;
        for (size_t i = 0; i < P; i++) {
            for (size_t j = 0; j < Q; j++) {
                self(i, j) = in_matrix(i, j);
            }
        }
        return self;
    }

    // allow assigning vectors to rows, even though vectors are column major
    Slice<Type, 1, Q, M, N>& operator=(const Vector<Type, Q>& in_vector)
    {
        Slice<Type, 1, Q, M, N>& self = *this;
        for (size_t j = 0; j < Q; j++) {
            self(0, j) = in_vector(j);
        }
        return self;
    }

    void copyTo(Type dst [M*N]) const
    {
        Slice<Type, P, Q, M, N>& self = *this;
        for (size_t i = 0; i < P; i++) {
            for (size_t j = 0; j < Q; j++) {
                dst[i*M + j] = self(i, j);
            }
        }
    }

    template<size_t R, size_t S, size_t T>
    Matrix<Type, P, R> operator*(const Slice<Type, Q, R, S, T> &other) const
    {
        return mult(*this, other);
    }

    template<size_t R, size_t S>
    const Slice<Type, R, S, M, N> slice(size_t x0, size_t y0) const
    {
        return Slice<Type, R, S, M, N>(x0 + _x0, y0 + _y0, _data);
    }

    template<size_t R, size_t S>
    Slice<Type, R, S, M, N> slice(size_t x0, size_t y0)
    {
        return Slice<Type, R, S, M, N>(x0 + _x0, y0 + _y0, _data);
    }

private:
    size_t _x0, _y0;
    Matrix<Type,M,N>* _data;
};

}
