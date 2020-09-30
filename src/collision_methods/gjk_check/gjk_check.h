
#ifndef COLLISION_DETECT_GJK_CHECK_H
#define COLLISION_DETECT_GJK_CHECK_H

#include "collision_methods/base/collision_base.h"

typedef Point Vector;

struct Simplex {
    std::vector<Point> pts;
    Simplex() : pts({}) {}

    void add(const Point& pt) {
        pts.push_back(pt);
    }

    Point getLast() const{
        return pts.back();
    }

    void remove(size_t index) {
        pts.erase(pts.begin() + index);
    }
};

class GjkCheck : public CollisionBase {
public:
    GjkCheck() {
        is_init_ = true;
    }

    ~GjkCheck() = default;

private:
    bool Init() override ;
    bool CheckCollision() override;
    bool gjk(const std::vector<Point>& polygon1,
             const std::vector<Point>& polygon2);

    Point support(const std::vector<Point>& polygon1,
                  const std::vector<Point>& polygon2,
                  const Vector& v);

    Point getFarthestPointInDirection(const std::vector<Point>& polygon,
            const Vector& v);

    bool containsOrigin(Simplex& simplex, Vector& v);

    Vector tripleProduce(const Vector& v1, const Vector& v2, const Vector& v3);
};
#endif //COLLISION_DETECT_GJK_CHECK_H
