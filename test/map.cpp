#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
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



    float map_data2[9] {};
    (Map<float, 3, 3>(map_data2)) = A;

    Matrix<float, 3, 3> map_check2(map_data2);
    TEST(isEqual(map_check2, A));


    float map_data3[3][3] {};
    (Map<float, 3, 3>(map_data3)) = A;

    Matrix<float, 3, 3> map_check3(map_data3);
    TEST(isEqual(map_check3, A));

    float map_data4[4] {};
    Map<float, 2, 2> A_map4(map_data4);
    A_map4 = A.slice<2,2>(0,0);

    float map4_check[4] = {0, 2,
                           4, 5
                          };
    Matrix<float, 2, 2> A_check4(map4_check);
    Matrix<float, 2, 2> map_check4(map_data4);
    TEST(isEqual(map_check4, A_check4));

    const Map<float, 2, 2> cmap(map_data4);
    Matrix<float, 2, 2> write_cmap = cmap;
    TEST(isEqual(write_cmap, A_check4));

    Matrix<float, 3, 3> write_slice;
    write_slice.slice<2,2>(0,0) = cmap;
    TEST(isEqual(Matrix<float,2,2>(write_slice.slice<2,2>(0,0)), A_check4));

    Matrix<float, 2, 2> write_map = A_map4;
    TEST(isEqual(write_map, Matrix<float,2,2>(A_map4)));
    A_map4(1,1) = 42;
    TEST(!isEqual(write_map, Matrix<float,2,2>(A_map4)));
    TEST((map_data4[3] - 42) < 1e-5);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
