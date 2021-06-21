// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework5/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/proto/route.pb.h"
#include "common/proto/map.pb.h"
#include "common/proto/map_lane.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework5/simulation/vehicle_agent_factory.h"

#include <bits/stdc++.h>

namespace nbt
{

    class PIDController
    {
    public:
        void init(double tar, double kp = 0.5, double ki = 0.00002, double kd = 0.3)
        {
            last_error = tar, sum_error = 0, target = 0.;
            setTarget(tar, 0.);
            Kp = kp, Ki = ki, Kd = kd;
        }
        void setTarget(double tar, double current)
        {
            last_error += tar - target;
            if (sum_error * (tar - current) > 0)
                sum_error = 0;
            target = tar;
        }
        double solve(double current)
        {
            double current_error = target - current;
            sum_error += current_error;
            double output = Kp * current_error + Ki * sum_error + Kd * (current_error - last_error);
            last_error = current_error;
            return output;
        }

    private:
        double last_error, sum_error, target;
        double Kp = 0.5, Ki = 0.00002, Kd = 0.3;
    };

    class DirectionController
    {
    public:
        void init(double tar, double kp = 0.5, double ki = 0.00002, double kd = 0.3)
        {
            last_error = tar, sum_error = 0, target = 0.;
            setTarget(tar, 0.);
            Kp = kp, Ki = ki, Kd = kd;
        }
        double _minus(double a, double b)
        {
            double c = a - b;
            if (c > pi)
                c -= pi * 2;
            if (c < -pi)
                c += pi * 2;
            return c;
        }
        void setTarget(double tar, double current)
        {
            last_error += tar - target;
            if (sum_error * _minus(tar, current) < 0)
                sum_error = 0;
            target = tar;
        }
        double solve(double current)
        {
            double current_error = _minus(target, current);
            sum_error += current_error;
            double output = Kp * current_error + Ki * sum_error + Kd * (current_error - last_error);
            last_error = current_error;
            return output;
        }

    private:
        double last_error, sum_error, target;
        double Kp = 0.5, Ki = 0.00002, Kd = 0.3;
        const double pi = acos(-1);
    };

    namespace RouteFinder
    {

        double sqr(double x)
        {
            return x * x;
        }

        double distance(interface::geometry::Point3D p, interface::geometry::Point3D q)
        {
            return std::sqrt(sqr(p.x() - q.x()) + sqr(p.y() - q.y()) + sqr(p.z() - q.z()));
        }
        void findRoute(interface::route::Route &route, interface::geometry::Vector3d start_point, interface::geometry::Point3D end_point)
        {
            interface::map::Map map;
            file::ReadTextFileToProto("/home/ponyai/homework/PublicCourse/homework4/prob2/processed_map_proto.txt", &map);

            double x_st = start_point.x(), y_st = start_point.y();
            double x_ed = end_point.x(), y_ed = end_point.y();

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
                return;
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
        }
    }

    // A sample vehicle agent for route 1
    // This agent will always run in straight. It will accelerate to the speed slightly over 5m/s,
    // and keeps running with the speed over 5m/s until reaches destination.

    class nbtVehicleAgent : public simulation::VehicleAgent
    {
    public:
        explicit nbtVehicleAgent(const std::string &name) : VehicleAgent(name) {}

        virtual void Initialize(const interface::agent::AgentStatus &agent_status)
        {
            velocityController.init(5.);
            angleController.init(0., 9., 0.002, 1);
            RouteFinder::findRoute(route, agent_status.vehicle_status().position(), agent_status.route_status().destination());
        }

        virtual interface::control::ControlCommand RunOneIteration(
            const interface::agent::AgentStatus &agent_status)
        {
            interface::control::ControlCommand command;
            double distance, current_v;
            distance = CalcDistance(agent_status.vehicle_status().position(), agent_status.route_status().destination());
            current_v = CalcVelocity(agent_status.vehicle_status().velocity());
            if (distance < 15.)
                velocityController.setTarget(std::max(distance / 3 - 0.5, 0.), current_v);
            double control_v = velocityController.solve(current_v);
            std::cout << current_v << std::endl;
            //std::cout << control_v << std::endl;
            //control_v = std::max(-1., std::min(control_v, 1.));
            if (control_v >= 0)
                command.set_throttle_ratio(control_v);
            else
                command.set_brake_ratio(-control_v);

            //interface::route::Route route;
            //RouteFinder::findRoute(route, agent_status.vehicle_status().position(), agent_status.route_status().destination());
            for (int i = position_pointer; i < route.route_point_size() && i <= position_pointer + 5; i++)
                if (CalcDistance(agent_status.vehicle_status().position(), route.route_point(i)) <= CalcDistance(agent_status.vehicle_status().position(), route.route_point(position_pointer)))
                    position_pointer = i;
            //while (position_pointer < route.route_point_size() - 1 && CalcDistance(agent_status.vehicle_status().position(), route.route_point(position_pointer + 1)) <= CalcDistance(agent_status.vehicle_status().position(), route.route_point(position_pointer)))
            //position_pointer++;
            //std::cout << route.route_point_size() << ' ' << position_pointer << std::endl;
            interface::geometry::Point2D forward_point = route.route_point(std::min(route.route_point_size() - 1, position_pointer + 10));
            //std::cout << agent_status.vehicle_status().position().x() << ' ' << agent_status.vehicle_status().position().y() << std::endl;
            //std::cout << forward_point.x() << ' ' << forward_point.y() << std::endl;
            double current_direction = atan2(agent_status.vehicle_status().velocity().y(), agent_status.vehicle_status().velocity().x());
            double target_direction = atan2(forward_point.y() - agent_status.vehicle_status().position().y(), forward_point.x() - agent_status.vehicle_status().position().x());
            angleController.setTarget(target_direction, current_direction);
            double control_direction = angleController.solve(current_direction);
            command.set_steering_angle(control_direction);
            //std::cout << route.route_point_size() << std::endl;

            return command;
        }

    private:
        double CalcDistance(const interface::geometry::Vector3d &position,
                            const interface::geometry::Point3D &destination)
        {
            double sqr_sum =
                math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
            ;
            return std::sqrt(sqr_sum);
        }

        double CalcDistance(const interface::geometry::Vector3d &position,
                            const interface::geometry::Point2D &destination)
        {
            double sqr_sum =
                math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
            ;
            return std::sqrt(sqr_sum);
        }

        double CalcVelocity(const interface::geometry::Vector3d &velocity)
        {
            double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
            ;
            return std::sqrt(sqr_sum);
        }

        PIDController velocityController;
        DirectionController angleController;
        interface::route::Route route;
        int position_pointer = 0;
    };

} // namespace sample
