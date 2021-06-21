// Copyright @2020 Pony AI Inc. All rights reserved.

#include "homework3/cubic_spline.h"

#include <fstream>
#include <iostream>
#include <algorithm>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(input_file_path, "", "Absolute path for input data file");

// TODO: Feel free to modify this file as you need.
int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    if (FLAGS_input_file_path.empty())
    {
        LOG(FATAL) << "Please provide absolute path for your input data file with --input_file_path ";
    }

    std::ifstream data_file(FLAGS_input_file_path);

    std::vector<double> xs;
    std::vector<double> ys;

    double x = 0.0;
    double y = 0.0;
    while (data_file >> x >> y)
    {
        //std::cout << x << " " << y << std::endl;
        xs.push_back(x);
        ys.push_back(y);
    }

    std::cout << xs.size() + 1000 << std::endl;
    for (int i = 0; i < xs.size(); i++)
        std::cout << xs[i] << "\n"
                  << ys[i] << "\n";
    bool xsorted = false;
    if (std::is_sorted(xs.begin(), xs.end()) || std::is_sorted(xs.begin(), xs.end(), std::greater<double>()))
        xsorted = true;
    bool ysorted = false;
    if (std::is_sorted(ys.begin(), ys.end()) || std::is_sorted(ys.begin(), ys.end(), std::greater<double>()))
        ysorted = true;

    if (!xsorted)
        std::swap(xs, ys);

    CubicSpline cs;
    cs.SetPoints(xs, ys);
    cs.Interpolate();

    double minX = *min_element(xs.begin(), xs.end());
    double maxX = *max_element(xs.begin(), xs.end());

    for (int i = 0; i < 1000; i++)
    {
        double x = minX + (maxX - minX) * i / 1000;
        double y = cs.GetInterpolatedY(x);
        if (xsorted)
            std::cout << x << "\n"
                      << y << "\n";
        else
            std::cout << y << "\n"
                      << x << "\n";
    }

    return 0;
}
