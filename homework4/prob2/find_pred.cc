#include "common/utils/file/file.h"
#include "homework4/display/main_window.h"
#include "homework4/map/map_lib.h"
#include "glog/logging.h"
#include <bits/stdc++.h>

double sqr(double x)
{
    return x * x;
}

int main()
{
    interface::map::Map _map;
    file::ReadTextFileToProto("/home/ponyai/homework/PublicCourse/pnc/map/grid3/map_proto.txt", &_map);

    interface::map::Map res = _map;

    res.clear_lane();

    int num_lane = _map.lane_size();
    std::vector<int> pred[num_lane], succ[num_lane];
    for (int i = 0; i < num_lane; i++)
    {
        interface::map::Lane l = _map.lane(i);
        int num_points = l.central_line().point_size();
        interface::geometry::Point3D p = l.central_line().point(num_points - 1);

        for (int j = 0; j < num_lane; j++)
        {
            interface::geometry::Point3D q = _map.lane(j).central_line().point(0);
            double x_1 = p.x(), y_1 = p.y(), z_1 = p.z();
            double x_2 = q.x(), y_2 = q.y(), z_2 = q.z();
            //std::cout << sqr(x_1 - x_2) + sqr(y_1 - y_2) + sqr(z_1 - z_2) << std::endl;
            if (sqr(x_1 - x_2) + sqr(y_1 - y_2) + sqr(z_1 - z_2) < 1)
            {
                //std::cout << "!!!!!!\n";
                succ[i].push_back(j);
                pred[j].push_back(i);
            }
        }
    }

    for (int i = 0; i < num_lane; i++)
    {
        interface::map::Lane *l = res.add_lane();
        l->CopyFrom(_map.lane(i));
        for (auto j : succ[i])
        {
            interface::map::Id *id = l->add_successor();
            id->CopyFrom(_map.lane(j).id());
        }
        for (auto j : pred[i])
        {
            interface::map::Id *id = l->add_predecessor();
            id->CopyFrom(_map.lane(j).id());
        }
    }

    file::WriteProtoToTextFile(res, "/home/ponyai/homework/PublicCourse/pnc/processed_map_proto.txt");
    return 0;
}