#include <gtest/gtest.h>
#include <mat.hpp>

EMIRO::Mat mat_class;

bool quaternion_check(Eigen::Quaternionf q1, Eigen::Quaternionf q2)
{
    return (q1.x() == q2.x() && q1.y() == q2.y() && q1.z() == q2.z() && q1.w() == q2.w());
}

TEST(Conversion, eu2quat)
{
    // Act
    Eigen::Quaternionf result = mat_class.euler_to_quaternion({0, 0, 0});
    // Assert
    EXPECT_EQ(quaternion_check(result, Eigen::Quaternionf(1., .00, .00, .00)), true);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}