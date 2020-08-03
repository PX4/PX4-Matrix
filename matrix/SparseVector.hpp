#include "Vector.hpp"
#include <cassert>

template<int N> struct force_constexpr_eval {
    static const int value = N;
};

// Vector that only store nonzero elements,
// which indices are specified as parameter pack
template<typename Type, int... Idxs>
class SparseVector {
private:
    static constexpr size_t N = sizeof...(Idxs);
    static constexpr int _indices[N] {Idxs...};
    matrix::Vector<Type, N> _data {};

    static constexpr int findCompressedIndex(int index, int range = N) {
        const int last_elem = range - 1;
        return (last_elem < 0 || _indices[last_elem] == index) ? last_elem : findCompressedIndex(index, range - 1);
    }

public:
    static constexpr int non_zeros() {
        return N;
    }
    constexpr int index(int i) const {
        return SparseVector::_indices[i];
    }

    SparseVector() = default;

    SparseVector(const matrix::Vector<Type, N>& data) : _data(data) {}

    template <int i>
    inline Type at() const {
        static constexpr int compressed_index = force_constexpr_eval<findCompressedIndex(i)>::value;
        static_assert(compressed_index >= 0, "cannot access unpopulated indices");
        return _data(static_cast<size_t>(compressed_index));
    }

    template <int i>
    inline Type& at() {
        static constexpr int compressed_index = force_constexpr_eval<findCompressedIndex(i)>::value;
        static_assert(compressed_index >= 0, "cannot access unpopulated indices");
        return _data(static_cast<size_t>(compressed_index));
    }

    void setZero() {
        _data.setZero();
    }
};

template<typename Type, int... Idxs>
constexpr int SparseVector<Type, Idxs...>::_indices[SparseVector<Type, Idxs...>::N];

template<int ... Idxs>
using SparseVectorf = SparseVector<float, Idxs...>;
