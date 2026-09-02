#include <cfenv>
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "common/math/math_helper.h"

namespace
{

using rmp::common::geometry::Vec2d;
using rmp::common::math::circleSegmentIntersection;

void expectFinite(const std::vector<Vec2d> & points)
{
  for (const auto & point : points) {
    EXPECT_TRUE(std::isfinite(point.x()));
    EXPECT_TRUE(std::isfinite(point.y()));
  }
}

TEST(CircleSegmentIntersection, keepsFiniteLegacyIntersectionGeometry)
{
  struct Case
  {
    Vec2d start;
    Vec2d end;
    double radius;
    std::vector<Vec2d> expected;
  };

  const std::vector<Case> cases{
    {{-1.0, 1.0}, {1.0, 1.0}, 1.0, {{0.0, 1.0}}},
    {{-2.0, 0.0}, {2.0, 0.0}, 1.0, {{1.0, 0.0}, {-1.0, 0.0}}},
    {{-1.0, 2.0}, {1.0, 2.0}, 1.0, {}},
  };

  for (const auto & test_case : cases) {
    const auto intersections = circleSegmentIntersection(
      test_case.start, test_case.end, test_case.radius);
    ASSERT_EQ(intersections.size(), test_case.expected.size());
    expectFinite(intersections);
    for (std::size_t index = 0; index < intersections.size(); ++index) {
      EXPECT_NEAR(intersections[index].x(), test_case.expected[index].x(), 1e-12);
      EXPECT_NEAR(intersections[index].y(), test_case.expected[index].y(), 1e-12);
    }
  }
}

TEST(CircleSegmentIntersection, zeroLengthSegmentReturnsNoPoints)
{
  const Vec2d point(0.5, 0.0);
  const auto intersections = circleSegmentIntersection(point, point, 1.0);

  EXPECT_TRUE(intersections.empty());
}

TEST(CircleSegmentIntersection, rejectedGeometryDoesNotRaiseInvalidFloatingPoint)
{
  std::feclearexcept(FE_ALL_EXCEPT);
  const auto intersections = circleSegmentIntersection(
    {-1.0, 2.0}, {1.0, 2.0}, 1.0);

  EXPECT_TRUE(intersections.empty());
  EXPECT_EQ(std::fetestexcept(FE_INVALID), 0);
}

TEST(CircleSegmentIntersection, tinyNegativeDiscriminantIsTreatedAsTangent)
{
  const double radius = std::nextafter(1.0, 0.0);
  const auto intersections = circleSegmentIntersection(
    {-1.0, 1.0}, {1.0, 1.0}, radius);

  ASSERT_EQ(intersections.size(), 1u);
  expectFinite(intersections);
}

}  // namespace
