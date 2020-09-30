#include "collision_methods/sat_check/sat_check.h"
#include <cfloat>

bool SatCheck::Init() {
    return true;
}

bool SatCheck::CheckCollision() {
//    return satObstacle();
    return satOgm();
}


bool SatCheck::satObstacle() {
    if (path_.empty() || obstacles_.empty()) {
        return false;
    }
    for (size_t i = 0; i < path_.size(); ++i) {
        PathPoint pose = path_.at(i);
        vehicle_model_.rectangleRep(pose);
        for (auto& obstacle : obstacles_) {
            if (sat(obstacle.contour, vehicle_model_.rect_contour)) {
                index_ = i;
//                return true;
            }
//            if (simpleSat(vehicle_model_, obstacle)) {
//                index_ = i;
//                return true;
//            }
        }
    }
    return false;
}

bool SatCheck::satOgm() {
    if (path_.empty() || ogm_.empty()) {
        return false;
    }

    for (size_t i = 0; i < path_.size(); ++i) {
        PathPoint pose = path_.at(i);
        vehicle_model_.rectangleRep(pose);
        for (auto& ogm_point : ogm_) {
            if (ogm_point.pt.x < vehicle_model_.min_x ||
                ogm_point.pt.x > vehicle_model_.max_x ||
                ogm_point.pt.y < vehicle_model_.min_y ||
                ogm_point.pt.y > vehicle_model_.max_y) {
                continue;
            } else {
                bool collision = true;
                std::vector<EdgeVector> edges =
                        {{vehicle_model_.rect_contour.at(0).x - vehicle_model_.rect_contour.at(1).x,
                          vehicle_model_.rect_contour.at(0).y - vehicle_model_.rect_contour.at(0).y},
                         {vehicle_model_.rect_contour.at(3).x - vehicle_model_.rect_contour.at(0).x,
                                 vehicle_model_.rect_contour.at(3).y - vehicle_model_.rect_contour.at(0).y}};
                for (size_t k = 0; k < 2; ++k) {
                    double min = DBL_MAX;
                    double max = -min;
                    EdgeVector edge = edges[k];
                    Axis axis = edge.vert();
//                    if (vehicle_model_.rect_contour.at(k).dot(axis) > edge.dot(vehicle_model_.rect_contour.at(k+1))) {
//                        min = vehicle_model_.rect_contour.at(k+1).dot(axis);
//                        max = vehicle_model_.rect_contour.at(k).dot(axis);
//                    } else {
//                        min = vehicle_model_.rect_contour.at(k).dot(axis);
//                        max = vehicle_model_.rect_contour.at(k+1).dot(axis);
//                    }
                    for (auto& pt : vehicle_model_.rect_contour) {
//                        std::cout << "min = " << min << ", max = " << max << std::endl;
                        double proj = pt.dot(axis);
//                        std::cout << "proj = " << proj << std::endl;
                        if (proj <= min) {
//                            std::cout << "min" << std::endl;
                            min = proj;
                        }
//                        std::cout << (proj > max) << std::endl;
                        if (proj >= max) {
//                            std::cout << "max" << std::endl;
                            max = proj;
                        }
                    }
//                    std::cout << "min = " << min << ", max = " << max << std::endl;
                    double ogm_proj = ogm_point.pt.dot(axis);
//                    std::cout << "ogm_proj = " << ogm_proj << std::endl;
                    if (ogm_proj > max || ogm_proj < min) {
                        collision = false;
//                        break;

                    }
                }
                if (collision) {
                    index_ = i;
//                    return true;
                }
            }

        }
    }
    return false;
}

bool SatCheck::sat(const std::vector<Point>& polygon1,
                   const std::vector<Point>& polygon2) {
    if (polygon1.size() < 4 || polygon2.size() < 4) {
        return false;
    }

    // check with polygon1
    std::vector<EdgeVector> edges = calEdgeVector(polygon1);
    for (auto& edge : edges) {
        Axis axis = edge.vert();
        double polygon1_proj_min = polygon1.at(0).dot(axis);
        double polygon1_proj_max = polygon1.at(0).dot(axis);
        double polygon2_proj_min = polygon2.at(0).dot(axis);
        double polygon2_proj_max = polygon2.at(0).dot(axis);
        for (size_t i = 1; i < polygon1.size(); ++i) {
            double project = polygon1.at(i).dot(axis);
            if (project > polygon1_proj_max) {
                polygon1_proj_max = project;
            }
            if (project < polygon1_proj_min) {
                polygon1_proj_min = project;
            }
        }

        for (size_t i = 1; i < polygon2.size(); ++i) {
            double project = polygon2.at(i).dot(axis);
            if (project > polygon2_proj_max) {
                polygon2_proj_max = project;
            }
            if (project < polygon2_proj_min) {
                polygon2_proj_min = project;
            }
        }

        if (polygon1_proj_min > polygon2_proj_max || polygon1_proj_max < polygon2_proj_min) {
            return false;
        }
    }

    // check with polygon2
    edges = calEdgeVector(polygon2);
    for (auto& edge : edges) {
        Axis axis = edge.vert();
        double polygon1_proj_min = polygon1.at(0).dot(axis);
        double polygon1_proj_max = polygon1.at(0).dot(axis);
        double polygon2_proj_min = polygon2.at(0).dot(axis);
        double polygon2_proj_max = polygon2.at(0).dot(axis);
        for (size_t i = 1; i < polygon1.size(); ++i) {
            double project = polygon1.at(i).dot(axis);
            if (project > polygon1_proj_max) {
                polygon1_proj_max = project;
            }
            if (project < polygon1_proj_min) {
                polygon1_proj_min = project;
            }
        }

        for (size_t i = 1; i < polygon2.size(); ++i) {
            double project = polygon2.at(i).dot(axis);
            if (project > polygon2_proj_max) {
                polygon2_proj_max = project;
            }
            if (project < polygon2_proj_min) {
                polygon2_proj_min = project;
            }
        }

        if (polygon1_proj_min > polygon2_proj_max || polygon1_proj_max < polygon2_proj_min) {
            return false;
        }
    }
    return true;
}

std::vector<EdgeVector>
SatCheck::calEdgeVector(const std::vector<Point> &polygon) {
    std::vector<EdgeVector> edges;
    EdgeVector edge;
    for (size_t i = 1; i < polygon.size(); ++i) {
        edge.x = polygon[i].x - polygon[i-1].x;
        edge.y = polygon[i].y - polygon[i-1].y;
        edges.push_back(edge);
    }
    edge.x = polygon[0].x - polygon[polygon.size()-1].x;
    edge.y = polygon[0].y - polygon[polygon.size()-1].y;
    edges.push_back(edge);
    return edges;
}

bool SatCheck::simpleSat(const VehicleModel& veh,
                         const Obstacle& obstacle) {
    if (veh.min_x > obstacle.max_x || veh.max_x < obstacle.min_x ||
        veh.min_y > obstacle.max_y || veh.max_y < obstacle.min_y) {
        return false;
    }

    const double shift_x = obstacle.center.x - veh.center.x;
    const double shift_y = obstacle.center.y - veh.center.y;

    const double dx1 = std::cos(veh.theta) * (veh.base2tail + veh.base2front) / 2.0;
    const double dy1 = std::sin(veh.theta) * (veh.base2tail + veh.base2front) / 2.0;
    const double dx2 = std::sin(veh.theta) * veh.width / 2.0;
    const double dy2 = -std::cos(veh.theta) * veh.width / 2.0;
    const double dx3 = std::cos(obstacle.theta) * obstacle.height / 2.0;
    const double dy3 = std::sin(obstacle.theta) * obstacle.height / 2.0;
    const double dx4 = std::sin(obstacle.theta) * obstacle.width / 2.0;
    const double dy4 = -std::cos(obstacle.theta) * obstacle.width / 2.0;

    return std::abs(shift_x * std::cos(veh.theta) + shift_y * std::sin(veh.theta)) <=
           std::abs(dx3 * std::cos(veh.theta) + dy3 * std::sin(veh.theta)) +
           std::abs(dx4 * std::cos(veh.theta) + dy4 * std::sin(veh.theta)) +
                   (veh.base2tail + veh.base2front) / 2.0 &&
           std::abs(shift_x * std::sin(veh.theta) - shift_y * std::cos(veh.theta)) <=
           std::abs(dx3 * std::sin(veh.theta) - dy3 * std::cos(veh.theta)) +
           std::abs(dx4 * std::sin(veh.theta) - dy4 * std::cos(veh.theta)) +
                   veh.width / 2.0 &&
           std::abs(shift_x * std::cos(obstacle.theta) + shift_y * std::sin(obstacle.theta)) <=
           std::abs(dx1 * std::cos(obstacle.theta) + dy1 * std::sin(obstacle.theta)) +
           std::abs(dx2 * std::cos(obstacle.theta) + dy2 * std::sin(obstacle.theta)) +
                   obstacle.height / 2.0 &&
           std::abs(shift_x * std::sin(obstacle.theta) - shift_y * std::cos(obstacle.theta)) <=
           std::abs(dx1 * std::sin(obstacle.theta) - dy1 * std::cos(obstacle.theta)) +
           std::abs(dx2 * std::sin(obstacle.theta) - dy2 * std::cos(obstacle.theta)) +
                   obstacle.width / 2.0;
}


