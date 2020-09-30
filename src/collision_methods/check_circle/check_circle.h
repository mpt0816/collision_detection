#ifndef COLLISION_DETECT_CHECK_CIRCLE_H
#define COLLISION_DETECT_CHECK_CIRCLE_H

#include "collision_methods/base/collision_base.h"

class CheckCircle : public CollisionBase {
public:
    CheckCircle() {
        is_init_ = true;
    }

    ~CheckCircle() = default;

private:
    bool Init();
    bool CheckCollision() override;

    double calDisBetweenPoint(const Point &p1, const Point &p2);
};


#endif //COLLISION_DETECT_CHECK_CIRCLE_H
