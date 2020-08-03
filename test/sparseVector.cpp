#include <matrix/math.hpp>
#include <gtest/gtest.h>

using namespace matrix;

TEST(sparseVectorTest, initialization) {
    SparseVectorf<4, 6> a;
    EXPECT_EQ(a.non_zeros(), 2);
    EXPECT_EQ(a.index(0), 4);
    EXPECT_EQ(a.index(1), 6);
    a.at<4>() = 1.f;
    a.at<6>() = 2.f;
}

TEST(sparseVectorTest, initializationWithVector) {
    Vector3f vec(1.f, 2.f, 3.f);
    SparseVectorf<4, 6, 22> a(vec);
    EXPECT_EQ(a.non_zeros(), 3);
    EXPECT_EQ(a.index(0), 4);
    EXPECT_EQ(a.index(1), 6);
    EXPECT_EQ(a.index(2), 22);
    EXPECT_FLOAT_EQ(a.at<4>(), vec(0));
    EXPECT_FLOAT_EQ(a.at<6>(), vec(1));
    EXPECT_FLOAT_EQ(a.at<22>(), vec(2));
    a.setZero();
    EXPECT_FLOAT_EQ(a.at<4>(), 0.f);
    EXPECT_FLOAT_EQ(a.at<6>(), 0.f);
    EXPECT_FLOAT_EQ(a.at<22>(), 0.f);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    std::cout << "Run SparseVector tests" << std::endl;
    return RUN_ALL_TESTS();
}
