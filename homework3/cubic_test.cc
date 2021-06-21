#include <gtest/gtest.h>

#include "homework3/cubic_spline.h"

#include <cmath>

namespace homework3
{

    TEST(CubicSpline, Interpolate)
    {
        int n = 100;
        std::vector<double> xs;
        std::vector<double> ys;
        for (int i = 0; i < n; i++)
        {
            xs.push_back(i * acos(-1) / n);
            ys.push_back(sin(xs[i]));
        }
        CubicSpline cs;
        cs.SetPoints(xs, ys);
        cs.Interpolate();

        for (int i = 0; i < 2 * n - 1; i++)
        {
            EXPECT_NEAR(sin(i * acos(-1) / 2 / n), cs.GetInterpolatedY(i * acos(-1) / 2 / n), 1e-6);
        }
    }
}