// Copyright @2019 Pony AI Inc. All rights reserved.

#include "homework2/single_frame_detector.h"

#include "glog/logging.h"

#include <map>
#include <vector>
#include <queue>

int dx[4] = {1, -1, 0, 0};
int dy[4] = {0, 0, 1, -1};

namespace
{

    // If you want to put all your implementation inside this file, you can add some functions here.
    void ASampleFunction()
    {
        LOG(ERROR) << "Ground points have been detected.";
    }

} // namespace

SingleFrameDetector::SingleFrameDetector()
{
    // TODO(you): optional, if you need to do anything to initialize the detector, you can do it here.
    // For example, you can load offline computed ground information here.
}

void SingleFrameDetector::GetGroundAndObstacles(
    const PointCloud &point_cloud,
    std::vector<Eigen::Vector3d> *ground_points,
    std::vector<Obstacle> *obstacles)
{
    CHECK(ground_points != nullptr);
    CHECK(obstacles != nullptr);
    // TODO(you): add some code to detect the ground and put all the points insidethe ground_points.
    //  I provide some trivial code here. Please replace them with a better implementation.
    std::map<std::pair<int, int>, double> minZ;
    std::map<std::pair<int, int>, double> minZ_2;
    std::map<std::pair<int, int>, int> obst_count;
    std::vector<double> dataZ;
    for (const Eigen::Vector3d &point : point_cloud.points)
    {
        Eigen::Vector3d realPoint = point_cloud.rotation * point + point_cloud.translation;
        int rx = int(realPoint.x() * 2), ry = int(realPoint.y() * 2);
        std::pair<int, int> pos = std::make_pair(rx, ry);
        dataZ.push_back(realPoint.z());
        if (minZ.find(pos) != minZ.end())
            minZ[pos] = std::min(minZ[pos], realPoint.z());
        else
            minZ[pos] = realPoint.z();
        int rx_2 = int(realPoint.x() / 5), ry_2 = int(realPoint.y() / 5);
        std::pair<int, int> pos_2 = std::make_pair(rx_2, ry_2);
        if (minZ_2.find(pos_2) != minZ_2.end())
            minZ_2[pos_2] = std::min(minZ_2[pos_2], realPoint.z());
        else
            minZ_2[pos_2] = realPoint.z();
    }
    sort(dataZ.begin(), dataZ.end());
    double lowZ = dataZ[0];
    for (const Eigen::Vector3d &point : point_cloud.points)
    {
        Eigen::Vector3d realPoint = point_cloud.rotation * point + point_cloud.translation;
        int rx = int(realPoint.x() * 2), ry = int(realPoint.y() * 2);
        std::pair<int, int> pos = std::make_pair(rx, ry);
        int rx_2 = int(realPoint.x() / 5), ry_2 = int(realPoint.y() / 5);
        std::pair<int, int> pos_2 = std::make_pair(rx_2, ry_2);
        if (realPoint.z() < minZ[pos] + 0.1 && realPoint.z() < minZ_2[pos_2] + 0.5 && realPoint.z() < lowZ + 100)
            ground_points->push_back(realPoint);
        else
            obst_count[std::make_pair(int(realPoint.x()), int(realPoint.y()))]++;
    }
    // TODO(you): Run flood fill or other algorithms to get the polygons for the obstacles.
    // Still, I provide a fake implementation, please replace it.
    std::map<std::pair<int, int>, int>::iterator it;
    for (it = obst_count.begin(); it != obst_count.end(); ++it)
        if (it->second < 20)
            obst_count.erase(it);
    int cnt = 0;
    for (it = obst_count.begin(); it != obst_count.end(); ++it)
        it->second = cnt++;
    std::vector<int> adjPoint[cnt];
    for (it = obst_count.begin(); it != obst_count.end(); ++it)
    {
        int x = it->first.first, y = it->first.second;
        //LOG(ERROR) << x << y;
        for (int k = 0; k < 4; k++)
        {
            int tx = x + dx[k], ty = y + dy[k];
            //LOG(ERROR) << x << y << tx << ty;
            std::pair<int, int> pos = std::make_pair(tx, ty);
            if (obst_count.find(pos) != obst_count.end())
            {
                adjPoint[it->second].push_back(obst_count[pos]);
                adjPoint[obst_count[pos]].push_back(it->second);
            }
        }
    }
    //LOG(ERROR) << "build edge end" << cnt;
    int obst_cnt = 0;
    int obst_id[cnt];
    for (int i = 0; i < cnt; i++)
        obst_id[i] = -1;
    for (int i = 0; i < cnt; i++)
        if (obst_id[i] == -1)
        {
            std::queue<int> Q;
            obst_id[i] = obst_cnt;
            Q.push(i);
            while (!Q.empty())
            {
                int u = Q.front();
                LOG(ERROR) << u;
                Q.pop();
                for (auto v : adjPoint[u])
                    if (obst_id[v] == -1)
                    {
                        obst_id[v] = obst_cnt;
                        Q.push(v);
                    }
            }
            obst_cnt++;
        }
    //LOG(ERROR) << "bfs end" << obst_cnt;
    double obstMinZ[obst_cnt], obstMaxZ[obst_cnt];
    double obstMinX[obst_cnt], obstMaxX[obst_cnt];
    double obstMinY[obst_cnt], obstMaxY[obst_cnt];
    for (int i = 0; i < obst_cnt; i++)
    {
        obstMinZ[i] = 1e30;
        obstMaxZ[i] = -1e30;
        obstMinX[i] = 1e30;
        obstMaxX[i] = -1e30;
        obstMinY[i] = 1e30;
        obstMaxY[i] = -1e30;
    }
    for (const Eigen::Vector3d &point : point_cloud.points)
    {
        Eigen::Vector3d realPoint = point_cloud.rotation * point + point_cloud.translation;
        int rx = int(realPoint.x()), ry = int(realPoint.y());
        std::pair<int, int> pos = std::make_pair(rx, ry);
        if (obst_count.find(pos) != obst_count.end())
        {
            obstMinZ[obst_id[obst_count[pos]]] = std::min(obstMinZ[obst_id[obst_count[pos]]], realPoint.z());
            obstMaxZ[obst_id[obst_count[pos]]] = std::max(obstMaxZ[obst_id[obst_count[pos]]], realPoint.z());
            obstMinX[obst_id[obst_count[pos]]] = std::min(obstMinX[obst_id[obst_count[pos]]], realPoint.x());
            obstMaxX[obst_id[obst_count[pos]]] = std::max(obstMaxX[obst_id[obst_count[pos]]], realPoint.x());
            obstMinY[obst_id[obst_count[pos]]] = std::min(obstMinY[obst_id[obst_count[pos]]], realPoint.y());
            obstMaxY[obst_id[obst_count[pos]]] = std::max(obstMaxY[obst_id[obst_count[pos]]], realPoint.y());
        }
    }
    for (int i = 0; i < obst_cnt; i++)
    {
        if (obstMaxZ[i] - obstMinZ[i] < 0.5)
            continue;
        //LOG(ERROR) << obstMinX[i] << ' ' << obstMaxX[i] << ' ' << obstMinY[i] << ' ' << obstMaxY[i] << ' ';

        obstacles->emplace_back();
        obstacles->back().polygon.emplace_back(obstMinX[i], obstMinY[i]);
        obstacles->back().polygon.emplace_back(obstMinX[i], obstMaxY[i]);
        obstacles->back().polygon.emplace_back(obstMaxX[i], obstMaxY[i]);
        obstacles->back().polygon.emplace_back(obstMaxX[i], obstMinY[i]);
        obstacles->back().floor = obstMinZ[i];
        obstacles->back().ceiling = obstMaxZ[i];
        obstacles->back().id = std::to_string(i);
    }
}
