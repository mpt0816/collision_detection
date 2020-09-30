#ifndef COLLISION_METHODS_CHECK_POLYGON_H_
#define COLLISION_METHODS_CHECK_POLYGON_H_
#include <geometry_msgs/Polygon.h>
#include "collision_methods/base/collision_base.h"

class ChechPolygon : public CollisionBase {
public:
  ChechPolygon() {
  	is_init_ = true;
  }
  ~ChechPolygon() {};

private:
  bool Init();
  bool CheckCollision();

private:
	bool GetTrailerHeadModel(double head, double tail, double width,
                             const PathPoint &ego, geometry_msgs::Polygon &polygon);
	bool CheckPointInPolygon(const geometry_msgs::Point32 &point, const geometry_msgs::Polygon &bounding_polygon);
	bool CheckFootPrintCollosion(const PathPoint &ego_pos, double head, double tail, float width);
};
#endif  // COLLISION_METHODS_CHECK_POLYGON_H_