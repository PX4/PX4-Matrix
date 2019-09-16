#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
    {
        float data[9] = {0, 2, 3,
                         4, 5, 6,
                         7, 8, 10
                        };
        Matrix<float, 3, 3> A(data);

        float map_data[9] {};
        Map<float, 3, 3> A_map(map_data);
        A_map = A;

        Matrix<float, 3, 3> map_check(map_data);
        TEST(isEqual(map_check, A));
    }
    {
        float data[9] = {0, 2, 3,
                         4, 5, 6,
                         7, 8, 10
                        };
        Matrix<float, 3, 3> A(data);

        float map_data[9] {};
        (Map<float, 3, 3>(map_data)) = A;

        Matrix<float, 3, 3> map_check(map_data);
        TEST(isEqual(map_check, A));
    }
    {
        float data[3][3] = {{0, 2, 3},
            {4, 5, 6},
            {7, 8, 10}
        };
        Matrix<float, 3, 3> A(data);

        float map_data[9] {};
        (Map<float, 3, 3>(map_data)) = A;

        Matrix<float, 3, 3> map_check(map_data);
        TEST(isEqual(map_check, A));
    }
    {
        float data[9] = {0, 2, 3,
                         4, 5, 6,
                         7, 8, 10
                        };
        Matrix<float, 3, 3> A(data);

        float map_data[9] {};
        Map<float, 3, 3> A_map(map_data);
        A_map = A.slice<3,3>(0,0);

        Matrix<float, 3, 3> map_check(map_data);
        TEST(isEqual(map_check, A));
    }
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
