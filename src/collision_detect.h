#ifndef COLLISION_DTETCT_H_
#define COLLISION_DTETCT_H_
#include "collision_methods/check_polygon/check_polygon.h"
#include "collision_methods/CV_picture/CV_picture.h"
#include "collision_methods/check_circle/check_circle.h"
#include "collision_methods/sat_check/sat_check.h"
#include "common/common_define.h"

const int map_max = 100;

class CollisionDetect {
public:
  CollisionDetect();
  ~CollisionDetect();

  bool ComputeCollision();
  void printResult();

  Path getPath() {
      return path_;
  };

  Ogm getOgm() {
      return ogm_;
  }

  Obstacles getObstacles() {
      return obstacles_;
  }

  size_t getCollionIndex() {
      if (collision) {
          return collosion_methods_->index_;
      } else {
          return SIZE_MAX;
      }
  }

  VehicleModel getVehModel() {
      return vehicle_model_;
  };

private:
  bool SimPathPoints();
  bool SimObstacles();
  bool SimVehicleModel();
  void generateOgm(int num);
  void generateObstacle(int num);


private:
  std::shared_ptr<CollisionBase> collosion_methods_;
  Path path_;
  Ogm ogm_;
  Obstacles obstacles_;
  VehicleModel vehicle_model_;
  bool collision;

};
#endif  // COLLISION_DTETCT_H_