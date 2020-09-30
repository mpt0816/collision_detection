/**********************************************************************
* Copyright (C) 2015-2020
*
* NodeName: ChechPolygon
* FileName: ChechPolygon.cpp
*
* Description: 
    
*
* History:
* guohui zhang         2020/08/24    1.0.0    build this module.
************************************************************************/
#include "collision_methods/check_polygon/check_polygon.h"

bool ChechPolygon::Init() {
  return true;
}

bool ChechPolygon::CheckCollision() {
  // std::cout << "------into ChechPolygon------" << std::endl;
  // std::cout << "path_.size(): " << path_.size() << std::endl;
  // std::cout << "ogm_.size(): " << ogm_.size() << std::endl;
  // std::cout << "vehicle_model_ length: " << vehicle_model_.length << std::endl;

  if (ogm_.empty()) {
    return false;
  }

  double head  = vehicle_model_.base2front;
  double tail  = vehicle_model_.base2tail;
  double width = vehicle_model_.width / 2.0;

  PathPoint ego_pos;

  for (int i = 0; i < path_.size(); ++i) {
      ego_pos = path_.at(i);
    // std::cout << "ego_pos.pt.x: " << ego_pos.pt.x << std::endl;
    // std::cout << "ego_pos.pt.y: " << ego_pos.pt.y << std::endl;
    if (CheckFootPrintCollosion(ego_pos, head, tail, width)) {
        index_ = i;
//      return true;
    }
  }
  return false;
}

bool ChechPolygon::CheckFootPrintCollosion(const PathPoint &ego_pos, double head, double tail, float width) {
  geometry_msgs::Polygon ego_polygon;
  GetTrailerHeadModel(head, tail, width, ego_pos, ego_polygon);
  vehicle_model_.rectangleRep(ego_pos);
  for (const auto& ogm_point : ogm_) {
    geometry_msgs::Point32 temp_ogm;
    temp_ogm.x = ogm_point.pt.x;
    temp_ogm.y = ogm_point.pt.y;
    if (temp_ogm.x < vehicle_model_.min_x ||
        temp_ogm.x > vehicle_model_.max_x ||
        temp_ogm.y < vehicle_model_.min_y ||
        temp_ogm.y > vehicle_model_.max_y) {
        continue;
    }
    // std::cout << "ogm_x: " << temp_ogm.x << std::endl;
    // std::cout << "ogm_y: " << temp_ogm.y << std::endl;
    // std::cout << "ego_polygon: " << ego_polygon << std::endl;
    if (CheckPointInPolygon(temp_ogm, ego_polygon)) {
//      return true;
    }
  }
  return false;
}


bool ChechPolygon::ChechPolygon::CheckPointInPolygon(const geometry_msgs::Point32 &point, 
  const geometry_msgs::Polygon &bounding_polygon) {
  if (bounding_polygon.points.empty()) return false;
  int counter = 0;
  double xinters;
  geometry_msgs::Point32 p1, p2;
  int N = bounding_polygon.points.size();
  p1 = bounding_polygon.points.at(0);
  for (int i = 1; i <= N; i++) {
    p2 = bounding_polygon.points.at(i % N);
    if (point.y > std::min<double>(p1.y, p2.y)) {
      if (point.y <= std::max<double>(p1.y, p2.y)) {
        if (point.x <= std::max<double>(p1.x, p2.x)) {
          if (p1.y != p2.y) {
            xinters = (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
            if (p1.x == p2.x || point.x <= xinters) counter++;
          }
        }
      }
    }
    p1 = p2;
  }
  if (0 == counter % 2)
    return false;
  else
    return true;
}

bool ChechPolygon::GetTrailerHeadModel(double head, double tail, double width,
                                       const PathPoint &ego, geometry_msgs::Polygon &polygon) {
  polygon.points.clear();
  geometry_msgs::Point32 pt, ld, ru;
  std::set<double> cmpx, cmpy;
  double vyaw = ego.theta + M_PI / 2.0;
  pt.x = ego.pt.x + head * cos(ego.theta) - width * cos(vyaw);
  pt.y = ego.pt.y + head * sin(ego.theta) - width * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.pt.x + head * cos(ego.theta) + width * cos(vyaw);
  pt.y = ego.pt.y + head * sin(ego.theta) + width * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.pt.x - tail * cos(ego.theta) + width * cos(vyaw);
  pt.y = ego.pt.y - tail * sin(ego.theta) + width * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.pt.x - tail * cos(ego.theta) - width * cos(vyaw);
  pt.y = ego.pt.y - tail * sin(ego.theta) - width * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  // ld.x = *(cmpx.begin());
  // ld.y = *(cmpy.begin());
  // ru.x = *(cmpx.rbegin());
  // ru.y = *(cmpy.rbegin());
  // auto rtn = std::forward_as_tuple(ld, ru);
  return true;
}