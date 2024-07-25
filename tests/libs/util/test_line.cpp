#include <gtest/gtest.h>

extern "C" {
#include "math/line.h"
#include "arm_math.h"
}

TEST(Line2f, IntersectionTest)
{
	// from Direction
	auto l1 = Line2fFromDirection(Vector2fFromXY(0, 0), Vector2fFromXY(1, 1));
	auto l2 = Line2fFromDirection(Vector2fFromXY(0, 1), Vector2fFromXY(1, -1));
	auto result = Line2fIntersectLine2f(l1, l2);
	EXPECT_FLOAT_EQ(result.x, 0.5);
	EXPECT_FLOAT_EQ(result.y, 0.5);

	l1 = Line2fFromDirection(Vector2fFromXY(4, 4), Vector2fFromXY(453, 13));
	l2 = Line2fFromDirection(Vector2fFromXY(4, 4), Vector2fFromXY(-45, 18));
	result = Line2fIntersectLine2f(l1, l2);
	EXPECT_FLOAT_EQ(result.x, 4);
	EXPECT_FLOAT_EQ(result.y, 4);

	// parallel lines
	l1 = Line2fFromDirection(Vector2fFromXY(0, 0), Vector2fFromXY(1, 0));
	l2 = Line2fFromDirection(Vector2fFromXY(0, 1), Vector2fFromXY(1, 0));
	result = Line2fIntersectLine2f(l1, l2);
	EXPECT_TRUE(isnanf(result.x));
	EXPECT_TRUE(isnanf(result.y));

	// equal lines
	l1 = Line2fFromDirection(Vector2fFromXY(-1, 0), Vector2fFromXY(1, 0));
	l2 = Line2fFromDirection(Vector2fFromXY(5, 0), Vector2fFromXY(1, 0));
	result = Line2fIntersectLine2f(l1, l2);
	EXPECT_TRUE(isnanf(result.x));
	EXPECT_TRUE(isnanf(result.y));

	// from points
	l1 = Line2fFromPoints(Vector2fFromXY(0, 0), Vector2fFromXY(1, 1));
	l2 = Line2fFromPoints(Vector2fFromXY(0, 1), Vector2fFromXY(1, 0));
	result = Line2fIntersectLine2f(l1, l2);
	EXPECT_FLOAT_EQ(result.x, 0.5);
	EXPECT_FLOAT_EQ(result.y, 0.5);
}

