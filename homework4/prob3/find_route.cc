#include "common/utils/file/file.h"
#include "homework4/display/main_window.h"
#include "glog/logging.h"
#include <bits/stdc++.h>

const int testcase = 5;

double sqr(double x)
{
    return x * x;
}

double distance(interface::geometry::Point3D p, interface::geometry::Point3D q)
{
    return std::sqrt(sqr(p.x() - q.x()) + sqr(p.y() - q.y()) + sqr(p.z() - q.z()));
}

int main()
{
    interface::map::Map map;
    file::ReadTextFileToProto("/home/ponyai/homework/PublicCourse/homework4/prob2/processed_map_proto.txt", &map);
    interface::route::Route route;
    file::ReadTextFileToProto("/home/ponyai/homework/PublicCourse/homework4/data/routes/route_request_" + std::to_string(testcase) + ".txt", &route);

    double x_st = route.start_point().x(), y_st = route.start_point().y();
    double x_ed = route.end_point().x(), y_ed = route.end_point().y();

    int num_lane = map.lane_size();
    std::map<std::string, int> lane_name;
    std::map<int, std::pair<int, int>> point_position;
    std::map<std::pair<int, int>, int> point_id;
    int tot_point = 0;

    int st_lane_id, st_point_id, ed_lane_id, ed_point_id;
    double st_t = 1e30, ed_t = 1e30;
    for (int i = 0; i < num_lane; i++)
    {
        lane_name[map.lane(i).id().id()] = i;
        int num_point = map.lane(i).central_line().point_size();
        for (int j = 0; j < num_point; j++)
        {
            point_id[std::make_pair(i, j)] = tot_point++;
            point_position[tot_point - 1] = std::make_pair(i, j);
            double x = map.lane(i).central_line().point(j).x(), y = map.lane(i).central_line().point(j).y();
            double dst = sqr(x - x_st) + sqr(y - y_st);
            double ded = sqr(x - x_ed) + sqr(y - y_ed);
            if (dst < st_t)
            {
                st_t = dst;
                st_lane_id = i;
                st_point_id = j;
            }
            if (ded < ed_t)
            {
                ed_t = ded;
                ed_lane_id = i;
                ed_point_id = j;
            }
        }
    }

    if (st_lane_id == ed_lane_id)
    {
        for (int j = st_point_id; j <= ed_lane_id; j++)
        {
            interface::geometry::Point2D *route_point = route.add_route_point();
            route_point->set_x(map.lane(st_lane_id).central_line().point(j).x());
            route_point->set_y(map.lane(st_lane_id).central_line().point(j).y());
        }
        file::WriteProtoToTextFile(route, "/home/ponyai/homework/PublicCourse/homework4/prob3/route_result_" + std::to_string(testcase) + ".txt");
        return 0;
    }

    std::vector<std::pair<int, double>> edge[tot_point];
    for (int i = 0; i < num_lane; i++)
    {
        interface::map::Lane l = map.lane(i);
        int num_point = l.central_line().point_size();
        for (int j = 0; j < num_point - 1; j++)
            edge[point_id[std::make_pair(i, j)]].push_back(std::make_pair(point_id[std::make_pair(i, j + 1)],
                                                                          distance(l.central_line().point(j), l.central_line().point(j + 1))));
        for (auto succ : l.successor())
        {
            int lane_id = lane_name[succ.id()];
            edge[point_id[std::make_pair(i, num_point - 1)]].push_back(std::make_pair(point_id[std::make_pair(lane_id, 0)],
                                                                                      distance(l.central_line().point(num_point - 1), map.lane(lane_id).central_line().point(0))));
        }
    }

    puts("succsess");
    std::cout << tot_point << std::endl;

    for (int i = 0; i < 100; i++)
    {
        std::cout << "edge from " << i << ":";
        for (auto v : edge[i])
        {
            std::cout << " " << v.first;
        }
        std::cout << std::endl;
    }

    int start = point_id[std::make_pair(st_lane_id, st_point_id)], end = point_id[std::make_pair(ed_lane_id, ed_point_id)];
    double dis[tot_point];
    int from_point[tot_point];
    std::priority_queue<std::pair<double, int>> Q;
    for (int i = 0; i < tot_point; i++)
        dis[i] = 1e30;
    dis[start] = 0;
    Q.push(std::make_pair(0, start));
    while (!Q.empty())
    {
        double d = -Q.top().first;
        int u = Q.top().second;
        //std::cout << u << ' ' << d << std::endl;
        Q.pop();
        if (std::fabs(dis[u] - d) > 1e-6)
            continue;
        for (auto v : edge[u])
        {
            if (d + v.second < dis[v.first])
            {
                dis[v.first] = d + v.second;
                from_point[v.first] = u;
                Q.push(std::make_pair(-dis[v.first], v.first));
            }
        }
    }

    puts("succsess");

    std::vector<int> path;
    int pos = end;
    while (true)
    {
        path.push_back(pos);
        if (pos == start)
            break;
        pos = from_point[pos];
    }

    std::reverse(path.begin(), path.end());

    for (auto x : path)
    {
        int i = point_position[x].first, j = point_position[x].second;
        interface::geometry::Point2D *route_point = route.add_route_point();
        route_point->set_x(map.lane(i).central_line().point(j).x());
        route_point->set_y(map.lane(i).central_line().point(j).y());
    }

    file::WriteProtoToTextFile(route, "/home/ponyai/homework/PublicCourse/homework4/prob3/route_result_" + std::to_string(testcase) + ".txt");
    return 0;
}