#ifndef COLLISION_METHODS_BASE_COLLISION_BASE_H_
#define COLLISION_METHODS_BASE_COLLISION_BASE_H_
#include "common/common_define.h"

class CollisionBase {
public:
  CollisionBase() {
    is_init_ = false;
    path_.clear();
    ogm_.clear();
    obstacles_.clear();
    index_ = SIZE_MAX;
  }
  virtual ~CollisionBase() {};

public:
  void SetCollisionData(const Path &path,
                        const Ogm &ogm,
                        const Obstacles &obstacles,
                        const VehicleModel &vehicle_model) {
    path_ = path;
    ogm_  = ogm;
    obstacles_ = obstacles;
    vehicle_model_= vehicle_model;
  }

  bool IsInit() {
    return is_init_;
  }

  virtual bool Init() = 0;
  virtual bool CheckCollision() = 0;

public:
  bool is_init_;
  Path path_;
  Ogm  ogm_;
  Obstacles obstacles_;
  VehicleModel vehicle_model_;
  size_t index_;
};

#endif  // COLLISION_METHODS_BASE_COLLISION_BASE_H_