
#ifndef COLLISION_DETECT_SAT_CHECK_H
#define COLLISION_DETECT_SAT_CHECK_H

#include "collision_methods/base/collision_base.h"

typedef Point EdgeVector;
typedef Point Axis;

class SatCheck : public CollisionBase {
public:
    SatCheck () {
        is_init_ = true;
    }
    ~SatCheck() = default;

private:
    bool Init() override;
    bool CheckCollision() override;
    bool satObstacle();
    bool satOgm();
    bool sat(const std::vector<Point>& polygon1,
             const std::vector<Point>& polygon2);

    bool simpleSat(const VehicleModel& veh,
                   const Obstacle& obstacle);

    std::vector<EdgeVector> calEdgeVector(const std::vector<Point>& polygon);

};
#endif //COLLISION_DETECT_SAT_CHECK_H
