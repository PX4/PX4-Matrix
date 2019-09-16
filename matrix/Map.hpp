/**
 * @file Map.hpp
 *
 * Map class, for wrapping chunks of memory without taking ownership or copying
 *
 * @author Julian Kent <julian@auterion.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template<typename Type, size_t M, size_t N>
class Matrix;

template<typename Type, size_t M, size_t N, size_t P, size_t Q>
class Slice;


template<typename Type, size_t M, size_t N>
class Map
{
    Type* _data;
public:

    Map(Type data[M*N])
    {
        _data = data;
    }

    Map(Type data[M][N])
    {
        _data = data;
    }

    Map<Type, M, N> & operator=(const Matrix<Type, M, N> &other)
    {
        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                _data[i*M + j] = other(i, j);
            }
        }
        return (*this);
    }

    template<size_t P, size_t Q>
    Map<Type, M, N> & operator=(const Slice<Type, M, N, P, Q> &other)
    {
        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                _data[i*M + j] = other(i, j);
            }
        }
        return (*this);
    }

    inline Type operator()(size_t i, size_t j) const
    {
        return _data[i*M + j];
    }

    inline Type &operator()(size_t i, size_t j)
    {
        return _data[i*M + j];
    }

};

}
