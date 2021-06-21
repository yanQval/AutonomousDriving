// Copyright @2020 Pony AI Inc. All rights reserved.

#include "homework3/cubic_spline.h"

#include <algorithm>
#include <Eigen/LU>
#include <Eigen/Dense>

void CubicSpline::SetPoints(const std::vector<double> &xs, const std::vector<double> &ys)
{
    // TODO: Implement
    int n = xs.size();
    for (int i = 0; i < n; i++)
        points.push_back(std::make_pair(xs[i], ys[i]));
    std::sort(points.begin(), points.end());
}

void CubicSpline::Interpolate()
{
    // TODO: Implement
    int n = points.size() - 1;
    Eigen::MatrixXd Ma = Eigen::MatrixXd::Zero(3 * n, 3 * n);
    Eigen::MatrixXd Mb = Eigen::VectorXd::Zero(3 * n);
    for (int i = 0; i < n; i++)
    {
        double d = points[i + 1].first - points[i].first;
        Ma(i, 3 * i) = d * d * d;
        Ma(i, 3 * i + 1) = d * d;
        Ma(i, 3 * i + 2) = d;
        Mb(i) = points[i + 1].second - points[i].second;
    }
    for (int i = 0; i < n - 1; i++)
    {
        double d = points[i + 1].first - points[i].first;
        Ma(i + n, 3 * i) = 3 * d * d;
        Ma(i + n, 3 * i + 1) = 2 * d;
        Ma(i + n, 3 * i + 2) = 1;
        Ma(i + n, 3 * i + 5) = -1;
        Ma(i + 2 * n - 1, 3 * i) = 3 * d;
        Ma(i + 2 * n - 1, 3 * i + 1) = 1;
        Ma(i + 2 * n - 1, 3 * i + 4) = -1;
    }
    Ma(3 * n - 2, 0) = 1;
    Ma(3 * n - 2, 3) = -1;
    Ma(3 * n - 1, 3 * n - 6) = 1;
    Ma(3 * n - 1, 3 * n - 3) = -1;
    Eigen::VectorXd x = Ma.inverse() * Mb;
    for (int i = 0; i < n; i++)
        polys.push_back((CubicPoly){x(3 * i), x(3 * i + 1), x(3 * i + 2)});
}

double CubicSpline::GetInterpolatedY(double x) const
{
    // TODO: Implement.
    int i = lower_bound(points.begin(), points.end(), std::make_pair(x, 1e30)) - points.begin() - 1;
    if (i < 0)
        i = 0;
    if (i > points.size() - 2)
        i = points.size() - 2;
    double d = x - points[i].first;
    return polys[i].a * d * d * d + polys[i].b * d * d + polys[i].c * d + points[i].second;
}
