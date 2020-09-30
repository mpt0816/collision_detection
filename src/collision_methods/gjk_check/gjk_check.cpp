
#include "collision_methods/gjk_check/gjk_check.h"
#include "cfloat"

bool GjkCheck::Init() {
    return true;
}

bool GjkCheck::CheckCollision() {
    if (path_.empty() || obstacles_.empty()) {
        return false;
    }

    for (size_t i = 0; i < path_.size(); ++i) {
        PathPoint pose = path_.at(i);
        vehicle_model_.rectangleRep(pose);
        for (auto& obstacle : obstacles_) {
            if (gjk(vehicle_model_.rect_contour, obstacle.contour)) {
                index_ = i;
//                return true;
            }
        }
    }
    return false;
}

bool GjkCheck::gjk(const std::vector<Point> &polygon1,
                   const std::vector<Point> &polygon2) {
    Simplex simplex;

    Point center_1, center_2;
    for (size_t i = 0; i < polygon1.size(); ++i) {
        center_1.x += polygon1[i].x;
        center_1.y += polygon1[i].y;
    }

    for (size_t i = 0; i < polygon2.size(); ++i) {
        center_2.x += polygon2[i].x;
        center_2.y += polygon2[i].y;
    }

    center_1.x = center_1.x / polygon1.size();
    center_1.y = center_1.y / polygon1.size();
    center_2.x = center_2.x / polygon2.size();
    center_2.y = center_2.y / polygon2.size();

    Vector v = {center_1.x - center_2.x, center_1.y - center_2.y};

    simplex.add(support(polygon1, polygon2, v));
    v = v.negate();
    while (true) {
        simplex.add(support(polygon1, polygon2, v));
        if (simplex.getLast().dot(v) <= 0) {
            return false;
        } else {
            if (containsOrigin(simplex, v)) {
                return true;
            }
        }
    }
}

Point GjkCheck::support(const std::vector<Point> &polygon1,
        const std::vector<Point> &polygon2,
        const Vector &v) {
    Point pt1 = getFarthestPointInDirection(polygon1, v);
    Point pt2 = getFarthestPointInDirection(polygon2, v.negate());
    return {pt1.x - pt2.x, pt1.y - pt2.y};
}

Point GjkCheck::getFarthestPointInDirection(
        const std::vector<Point> &polygon, const Vector &v) {
    size_t index = SIZE_MAX;
    double max = -DBL_MAX;
    for (size_t i = 0; i < polygon.size(); ++i) {
        double project = polygon[i].dot(v);
        if (project > max) {
            max = project;
            index = i;
        }
    }
    return polygon[index];
}

bool GjkCheck::containsOrigin(Simplex &simplex, Vector &v) {
    if (simplex.pts.empty()) {
        return false;
    }
    // get the last point added to the simplex
    Point a = simplex.getLast();
    // compute AO (same thing as -A)
    Vector ao = a.negate();
    if (simplex.pts.size() == 3) {
        // triangle case
        Point b = simplex.pts[1];
        Point c = simplex.pts[0];
        // compute the edges
        Vector ab = {b.x - a.x, b.y - a.y};
        Vector ac = {c.x - a.x, c.y - a.y};
        // compute the normal vector
        Vector abPerp = tripleProduce(ac, ab, ab);
        Vector acPerp = tripleProduce(ab, ac, ac);

        if (abPerp.dot(ao) > 0) {
            simplex.remove(0);
            v.set(abPerp);
            return false;
        } else {
            if (acPerp.dot(ao) > 0) {
                simplex.remove(1);
                v.set(acPerp);
                return false;
            } else {
                return true;
            }
        }
    } else {
        // line segment case
        Point b = simplex.pts.front();
        // compute AB
        Vector ab = {b.x - a.x, b.y - a.y};
        // get the perp to AB in the direction of the origin
        Vector abPerp = tripleProduce(ab, ao, ab);
        v.set(abPerp);
        return false;
    }
}

Vector GjkCheck::tripleProduce(const Vector &v1,
        const Vector &v2, const Vector &v3) {
    double v3ProjTov1 = v3.dot(v1);
    double v3ProjTov2 = v3.dot(v2);
    return {v3ProjTov1 * v2.x - v3ProjTov2 * v1.x, v3ProjTov1 * v2.y - v3ProjTov2 * v1.y};
}

