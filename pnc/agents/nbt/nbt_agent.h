// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "pnc/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/proto/route.pb.h"
#include "common/proto/map.pb.h"
#include "common/proto/map_lane.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/simulation/vehicle_agent_factory.h"

#include <bits/stdc++.h>

namespace nbt
{

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

    class PID
    {
    public:
        void init(double P, double I, double D)
        {
            Kp = P, Ki = I, Kd = D;
            is_first_iter = true;
            error_integral = 0;
            for (int i = 0; i < 500; i++)
                error_his[i] = 0;
            pos = 0;
        }
        double solve(double error, double &ie, double &de)
        {
            if (error * error_integral < 0)
                error_integral = 0;
            error_integral += error * 0.01;
            //error_his[pos] = error;
            pos = (pos + 1) % 500;
            if (is_first_iter)
            {
                error_last = error;
                is_first_iter = false;
            }
            double output = Kp * error + Ki * error_integral + (error - error_last) * Kd;
            de = error - error_last;
            error_last = error;
            ie = error_integral;
            return output;
        }
        void clear_integral()
        {
            error_integral = 0;
        }

    private:
        double Kp, Ki, Kd;
        double error_last, error_integral;
        bool is_first_iter;
        double error_his[500];
        int pos = 0;
    };

    namespace RouteFinder
    {

        interface::map::Map map;
        void initRouteFinder()
        {
            std::string filepath = file::path::Join(file::path::GetProjectRootPath(), "pnc", "agents", "nbt", "processed_map_proto.txt");
            file::ReadTextFileToProto(filepath, &map);
        }

        double sqr(double x)
        {
            return x * x;
        }

        double distance(interface::geometry::Point3D p, interface::geometry::Point3D q)
        {
            return std::sqrt(sqr(p.x() - q.x()) + sqr(p.y() - q.y()) + sqr(p.z() - q.z()));
        }
        void findRoute(interface::route::Route &route, interface::geometry::Vector3d start_point, interface::geometry::Point3D end_point, std::vector<int> &route_lane_id)
        {

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

            route.clear_route_point();
            route_lane_id.clear();

            for (auto x : path)
            {
                int i = point_position[x].first, j = point_position[x].second;
                interface::geometry::Point2D *route_point = route.add_route_point();
                route_point->set_x(map.lane(i).central_line().point(j).x());
                route_point->set_y(map.lane(i).central_line().point(j).y());
                route_lane_id.push_back(i);
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
            RouteFinder::initRouteFinder();
        }

        virtual interface::control::ControlCommand RunOneIteration(
            const interface::agent::AgentStatus &agent_status)
        {
            if (agent_status.route_status().is_new_request())
            {
                phase = 1;
                last_target_v = 0.;
                angleController.init(0., 9., 0.002, 1);
                velocityPID.init(0.5, 0.01, 0.05);
                RouteFinder::findRoute(route, agent_status.vehicle_status().position(), agent_status.route_status().destination(), route_lane_id);
                position_pointer = 0;
            }

            /*find types of each light*/
            double cur_time = agent_status.simulation_status().simulation_time();
            if (3 < cur_time && cur_time < 3.02)
            {
                light_type.resize(RouteFinder::map.lane_size());
                for (int i = 0; i < RouteFinder::map.lane_size(); i++)
                    light_type[i] = -1;
                for (int i = 0; i < agent_status.perception_status().traffic_light_size(); i++)
                {
                    interface::perception::PerceptionTrafficLightStatus lights = agent_status.perception_status().traffic_light(i);
                    for (int j = 0; j < lights.single_traffic_light_status_size(); j++)
                    {
                        interface::perception::SingleTrafficLightStatus light = lights.single_traffic_light_status(j);
                        for (int k = 0; k < RouteFinder::map.lane_size(); k++)
                            if ((light.id().id()).substr(2, (light.id().id()).length() - 5) == RouteFinder::map.lane(k).id().id())
                            {
                                if (light.color() == interface::map::Bulb::RED)
                                    light_type[k] = 0;
                                if (light.color() == interface::map::Bulb::GREEN)
                                    light_type[k] = 2;
                            }
                    }
                }
            }

            interface::control::ControlCommand command;

            /* control the direction */
            //interface::route::Route route;
            //RouteFinder::findRoute(route, agent_status.vehicle_status().position(), agent_status.route_status().destination());
            for (int i = position_pointer; i < route.route_point_size() && i <= position_pointer + 5; i++)
                if (CalcDistance(agent_status.vehicle_status().position(), route.route_point(i)) <= CalcDistance(agent_status.vehicle_status().position(), route.route_point(position_pointer)))
                    position_pointer = i;
            //std::cout << route.route_point_size() << ' ' << position_pointer << std::endl;
            interface::geometry::Point2D forward_point = route.route_point(std::min(route.route_point_size() - 1, position_pointer + 10));
            //std::cout << agent_status.vehicle_status().position().x() << ' ' << agent_status.vehicle_status().position().y() << std::endl;
            //std::cout << forward_point.x() << ' ' << forward_point.y() << std::endl;
            double current_direction = atan2(agent_status.vehicle_status().velocity().y(), agent_status.vehicle_status().velocity().x());
            double target_direction = atan2(forward_point.y() - agent_status.vehicle_status().position().y(), forward_point.x() - agent_status.vehicle_status().position().x());
            angleController.setTarget(target_direction, current_direction);
            double control_direction = angleController.solve(current_direction);
            command.set_steering_angle(control_direction);

            /* control speed */
            double distance, current_v, target_v;
            bool need_to_stop = false;
            target_v = 10;
            distance = CalcDistance(agent_status.vehicle_status().position(), agent_status.route_status().destination());
            current_v = CalcVelocity(agent_status.vehicle_status().velocity());
            double v0 = 10., d0 = 30.;
            if (distance < d0 && phase == 1)
            {
                phase = 0;
                velocityPID.clear_integral();
            }
            if (distance < d0)
                target_v = std::min(target_v, std::max(v0 * sqrt(std::max(distance - 2, 0.) / (d0 - 2)), 0.0));
            //target_v = distance < d0 ? std::max(v0 * sqrt(std::max(distance - 2, 0.) / (d0 - 2)), 0.05) : v0;
            // target_v = distance < d0 ? std::max(distance / 3 - 1.0, 0.05) : v0;

            if (cur_time > 4 && position_pointer < route.route_point_size() - 1)
            {
                int cur_lane_id = route_lane_id[position_pointer];
                if (light_type[cur_lane_id] > -1)
                {
                    int cur_color;
                    int full_cycle = floor((cur_time - 3) / 46);
                    double now_time = cur_time - 3 - 46 * full_cycle;
                    double res_time;
                    if (now_time < 20)
                    {
                        cur_color = light_type[cur_lane_id];
                        res_time = 20 - now_time;
                    }
                    else if (now_time < 23)
                    {
                        cur_color = light_type[cur_lane_id] ^ 1;
                        res_time = 23 - now_time;
                    }
                    else if (now_time < 43)
                    {
                        cur_color = light_type[cur_lane_id] ^ 2;
                        res_time = 43 - now_time;
                    }
                    else
                    {
                        cur_color = light_type[cur_lane_id] ^ 3;
                        res_time = 46 - now_time;
                    }
                    double dis_to_light = 0;
                    for (int i = position_pointer + 1; i < route.route_point_size() && route_lane_id[i] == route_lane_id[position_pointer]; i++)
                        dis_to_light += CalcDistance(route.route_point(i - 1), route.route_point(i));
                    PublishVariable("dis_to_light", std::to_string(dis_to_light));
                    if (dis_to_light < 30 && route_lane_id[route.route_point_size() - 1] != route_lane_id[position_pointer])
                        if (((cur_color == 0 || cur_color == 1) && std::min(dis_to_light / std::max(current_v, 1.0), 3.0) <= res_time + 3) || (cur_color != 0 && cur_color != 1 && dis_to_light / 10. + 1 >= res_time + 3.0 * (cur_color == 2)))
                        {
                            if (phase == 1)
                            {
                                phase = 0;
                                velocityPID.clear_integral();
                            }
                            // target_v = std::min(target_v, dis_to_light < 5 ? -(dis_to_light - 5) * (dis_to_light - 5) : v0 * sqrt(std::max(dis_to_light - 5, 0.) / (d0 - 5)));
                            target_v = std::min(target_v, v0 * sqrt(std::max(dis_to_light - 5, 0.) / (d0 - 5)));
                            //if (current_v < 0.5 || dis_to_light < 2)
                            //    need_to_stop = true;
                        }
                }
            }

            /* search pedestrians & cars */
            if (position_pointer < route.route_point_size() - 1)
            {
                double safe_dis = CalcDistance(route.route_point(position_pointer + 1), route.route_point(position_pointer));
                int now_point = position_pointer + 1;
                while (safe_dis < 30 && now_point < route.route_point_size())
                {
                    bool safe = true;
                    for (int i = 0; i < agent_status.perception_status().obstacle_size(); i++)
                    {
                        if (!safe)
                            break;
                        interface::perception::PerceptionObstacle obstacle = agent_status.perception_status().obstacle(i);
                        double threshold;
                        if (obstacle.type() == interface::perception::ObjectType::CAR || obstacle.type() == interface::perception::ObjectType::PEDESTRIAN)
                            threshold = 3.;
                        else
                            continue;
                        for (int j = 0; j < obstacle.polygon_point_size(); j++)
                            if (CalcDistance(obstacle.polygon_point(j), route.route_point(now_point)) < threshold)
                            {
                                safe = false;
                                break;
                            }
                    }
                    if (safe)
                    {
                        if (now_point != position_pointer)
                            safe_dis += CalcDistance(route.route_point(now_point), route.route_point(now_point - 1));
                        now_point++;
                    }
                    else
                    {
                        break;
                    }
                }
                PublishVariable("safe_dis", std::to_string(safe_dis));
                if (now_point != route.route_point_size() && safe_dis < 30)
                {
                    if (phase == 1)
                    {
                        phase = 0;
                        velocityPID.clear_integral();
                    }
                    target_v = std::min(target_v, v0 * sqrt(std::max(safe_dis - 6, 0.) / (d0 - 6)));
                    if (safe_dis < 3)
                        target_v = std::min(target_v, -0.2);
                }
            }

            if (target_v > last_target_v + 1.0 && phase == 0)
            // if (target_v > 9.9 && phase == 0)
            {
                phase = 1;
                velocityPID.clear_integral();
            }

            double ie, de;
            double control_v = velocityPID.solve(target_v - current_v, ie, de);
            //std::cout << control_v << std::endl;
            control_v = std::max(-.3, std::min(control_v, .2));
            PublishVariable("distance", std::to_string(distance));
            PublishVariable("error_i", std::to_string(ie));
            PublishVariable("error_d", std::to_string(de));
            PublishVariable("control_velocity", std::to_string(control_v));
            PublishVariable("target_velocity", std::to_string(target_v));
            PublishVariable("last_target_velocity", std::to_string(last_target_v));
            PublishVariable("phase", std::to_string(phase));
            last_target_v = target_v;
            if (need_to_stop)
                command.set_brake_ratio(0.3);
            else if (control_v >= 0)
                command.set_throttle_ratio(control_v);
            else
                command.set_brake_ratio(-control_v + 0.15);

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

        double CalcDistance(const interface::geometry::Point2D &position,
                            const interface::geometry::Point2D &destination)
        {
            double sqr_sum =
                math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
            ;
            return std::sqrt(sqr_sum);
        }

        double CalcDistance(const interface::geometry::Point3D &position,
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

        DirectionController angleController;
        interface::route::Route route;
        std::vector<int> route_lane_id;
        int position_pointer = 0;
        PID velocityPID;
        double last_target_v;
        int phase = 0;
        /*
        0 : brake
        1 : accelate / running
        */
        std::vector<int> light_type;
    };

} // namespace sample
