#include <matrix/math.hpp>
#include <gtest/gtest.h>

using namespace matrix;

TEST(sparseVectorTest, initialization) {
    SparseVectorf<24, 4, 6> a;
    EXPECT_EQ(a.non_zeros(), 2);
    EXPECT_EQ(a.index(0), 4);
    EXPECT_EQ(a.index(1), 6);
    a.at<4>() = 1.f;
    a.at<6>() = 2.f;
}

TEST(sparseVectorTest, initializationWithVector) {
    const Vector3f vec(1.f, 2.f, 3.f);
    SparseVectorf<24, 4, 6, 22> a(vec);
    EXPECT_EQ(a.non_zeros(), 3);
    EXPECT_EQ(a.index(0), 4);
    EXPECT_EQ(a.index(1), 6);
    EXPECT_EQ(a.index(2), 22);
    EXPECT_FLOAT_EQ(a.at<4>(), vec(0));
    EXPECT_FLOAT_EQ(a.at<6>(), vec(1));
    EXPECT_FLOAT_EQ(a.at<22>(), vec(2));
}

TEST(sparseVectorTest, fromDenseVector) {
    const Vector3f vec(1.f, 2.f, 3.f);
    SparseVectorf<24, 0, 2> a;
    a.fromDenseVector<3>(vec);
    EXPECT_FLOAT_EQ(a.at<0>(), 1.f);
    EXPECT_FLOAT_EQ(a.at<2>(), 3.f);
}

TEST(sparseVectorTest, setZero) {
    const Vector3f vec(1.f, 2.f, 3.f);
    SparseVectorf<24, 4, 6, 22> a(vec);
    a.setZero();
    EXPECT_FLOAT_EQ(a.at<4>(), 0.f);
    EXPECT_FLOAT_EQ(a.at<6>(), 0.f);
    EXPECT_FLOAT_EQ(a.at<22>(), 0.f);
}

TEST(sparseVectorTest, additionWithDenseVector) {
    Vector<float, 4> dense_vec;
    dense_vec.setAll(1.f);
    const Vector3f vec(1.f, 2.f, 3.f);
    const SparseVectorf<4, 1, 2, 3> sparse_vec(vec);
    const Vector<float, 4> res = sparse_vec + dense_vec;
    EXPECT_FLOAT_EQ(res(0), 1.f);
    EXPECT_FLOAT_EQ(res(1), 2.f);
    EXPECT_FLOAT_EQ(res(2), 3.f);
    EXPECT_FLOAT_EQ(res(3), 4.f);
}

TEST(sparseVectorTest, addScalar) {
    const Vector3f vec(1.f, 2.f, 3.f);
    SparseVectorf<4, 1, 2, 3> sparse_vec(vec);
    sparse_vec += 2.f;
    EXPECT_FLOAT_EQ(sparse_vec.at<1>(), 3.f);
    EXPECT_FLOAT_EQ(sparse_vec.at<2>(), 4.f);
    EXPECT_FLOAT_EQ(sparse_vec.at<3>(), 5.f);
}

TEST(sparseVectorTest, dotProductWithDenseVector) {
    Vector<float, 4> dense_vec;
    dense_vec.setAll(3.f);
    const Vector3f vec(1.f, 2.f, 3.f);
    const SparseVectorf<4, 1, 2, 3> sparse_vec(vec);
    float res = sparse_vec.dot(dense_vec);
    EXPECT_FLOAT_EQ(res, 18.f);
}

TEST(sparseVectorTest, multiplicationWithDenseMatrix) {
    Matrix<float, 2, 3> dense_matrix;
    dense_matrix.setAll(2.f);
    dense_matrix(1, 1) = 3.f;
    const Vector2f vec(1.f, 5.f);
    const SparseVectorf<3, 1, 2> sparse_vec(vec);
    const Vector<float, 2> res = dense_matrix * sparse_vec;
    EXPECT_FLOAT_EQ(res(0), 12.f);
    EXPECT_FLOAT_EQ(res(1), 13.f);
}

TEST(sparseVectorTest, norms) {
    const Vector2f vec(3.f, -4.f);
    const SparseVectorf<4, 1, 3> sparse_vec(vec);
    EXPECT_FLOAT_EQ(sparse_vec.norm_squared(), 25.f);
    EXPECT_FLOAT_EQ(sparse_vec.norm(), 5.f);
    EXPECT_TRUE(sparse_vec.longerThan(4.5f));
    EXPECT_FALSE(sparse_vec.longerThan(5.5f));
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    std::cout << "Run SparseVector tests" << std::endl;
    return RUN_ALL_TESTS();
}
