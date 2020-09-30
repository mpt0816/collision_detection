#ifndef COMMON_COMMON_DEFINE_H_
#define COMMON_COMMON_DEFINE_H_
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

struct Point {
    double x;
    double y;

    Point() : x(0.), y(0.) {}

    Point(double x, double y) : x(x), y(y) {};

    Point vert() const {
        return Point(-y, x);
    }

    double dot(const Point& pt) const {
        return pt.x * x + pt.y * y;
    }

    Point negate() const {
        return {-x, -y};
    }

    bool operator==(const Point& rv) const {
        return (x == rv.x && y == rv.y);
    }

    void set(const Point& pt) {
        x = pt.x;
        y = pt.y;
    }

};

struct PathPoint {
  Point pt;
  double theta;

  PathPoint() : pt(), theta(0.) {};

  PathPoint(double x, double y, double theta) : pt(x, y), theta(theta) {};

  void reset() {
    pt.x = 0.0;
    pt.y = 0.0;
    theta = 0.0;
  }
};

struct OgmPoint {
    Point pt;
    double p;

    OgmPoint() : pt(), p(1.0) {}

    OgmPoint(double x, double y) : pt(x, y), p(1.) {}
};

struct Obstacle {
    double theta;
    Point center;
    double height;
    double width;
    std::vector<Point> contour;

    double min_x;
    double max_x;
    double min_y;
    double max_y;

    Obstacle() : center(), contour({}), theta(0.0) {};

    void calMinMaxBounding() {
        min_x = std::numeric_limits<double>::max();
        max_x = std::numeric_limits<double>::lowest();
        min_y = std::numeric_limits<double>::max();
        max_y = std::numeric_limits<double>::lowest();
        for (auto& corner : contour) {
            min_x = std::fmin(corner.x, min_x);
            max_x = std::fmax(corner.x, max_x);
            min_y = std::fmin(corner.y, min_y);
            max_y = std::fmax(corner.y, max_y);
        }
    }
};


struct Circle {
    Point center;
    double radius;

    Circle() : center(), radius(0.) {}
    Circle(Point pt, double r) : center(pt), radius(r) {}
};

struct VehicleModel {
    Point center;
    double theta;
    double base2front;
    double base2tail;
    double width;
    std::vector<Point> rect_contour;
    Circle circle_bounding;
    std::vector<Circle> circle_square;
    std::vector<Circle> circle_rectangle;

    double min_x;
    double max_x;
    double min_y;
    double max_y;

    void reset() {
        base2front = 5.38;
        base2tail = 1.45;
        width = 2.54;

        min_x = std::numeric_limits<double>::max();
        max_x = std::numeric_limits<double>::lowest();
        min_y = std::numeric_limits<double>::max();
        max_y = std::numeric_limits<double>::lowest();
    }

    void rectangleRep(const PathPoint& pose) {

        rect_contour.clear();
        Point pt;
        pt.x = pose.pt.x + base2front * std::cos(pose.theta) - width /2.0 * std::sin(pose.theta);
        pt.y = pose.pt.y + base2front * std::sin(pose.theta) + width /2.0 * std::cos(pose.theta);
        rect_contour.emplace_back(pt);

        pt.x = pose.pt.x + base2front * std::cos(pose.theta) + width /2.0 * std::sin(pose.theta);
        pt.y = pose.pt.y + base2front * std::sin(pose.theta) - width /2.0 * std::cos(pose.theta);
        rect_contour.emplace_back(pt);

        pt.x = pose.pt.x - base2tail * std::cos(pose.theta) + width /2.0 * std::sin(pose.theta);
        pt.y = pose.pt.y - base2tail * std::sin(pose.theta) - width /2.0 * std::cos(pose.theta);
        rect_contour.emplace_back(pt);

        pt.x = pose.pt.x - base2tail * std::cos(pose.theta) - width /2.0 * std::sin(pose.theta);
        pt.y = pose.pt.y - base2tail * std::sin(pose.theta) + width /2.0 * std::cos(pose.theta);
        rect_contour.emplace_back(pt);

        center.x = (rect_contour[0].x + rect_contour[2].x) / 2.0;
        center.y = (rect_contour[0].y + rect_contour[2].y) / 2.0;
        theta    = pose.theta;

        min_x = std::numeric_limits<double>::max();
        max_x = std::numeric_limits<double>::lowest();
        min_y = std::numeric_limits<double>::max();
        max_y = std::numeric_limits<double>::lowest();
        for (auto& contour : rect_contour) {
            min_x = std::fmin(contour.x, min_x);
            max_x = std::fmax(contour.x, max_x);
            min_y = std::fmin(contour.y, min_y);
            max_y = std::fmax(contour.y, max_y);
        }
    }

    void circleRep(const PathPoint& pose) {
        double height = base2tail + base2front;
        // cal circle bouding
        circle_bounding.center.x = pose.pt.x + (height / 2.0 - base2tail) * std::cos(pose.theta);
        circle_bounding.center.y = pose.pt.y + (height / 2.0 - base2tail) * std::sin(pose.theta);
        circle_bounding.radius   = std::sqrt(height * height + width * width) / 2.0;
        // cal circle square
        circle_square.clear();
        double radius_square = std::sqrt(2.0) / 4.0 * width;
        Point pt;
        pt.x = pose.pt.x + (base2front - width / 4.0) * std::cos(pose.theta) - width /4.0 * std::sin(pose.theta);
        pt.y = pose.pt.y + (base2front - width / 4.0) * std::sin(pose.theta) + width /4.0 * std::cos(pose.theta);
        circle_square.emplace_back(pt, radius_square);

        pt.x = pose.pt.x + (base2front - width / 4.0) * std::cos(pose.theta) + width /4.0 * std::sin(pose.theta);
        pt.y = pose.pt.y + (base2front - width / 4.0) * std::sin(pose.theta) - width /4.0 * std::cos(pose.theta);
        circle_square.emplace_back(pt, radius_square);

        pt.x = pose.pt.x - (base2tail - width / 4.0) * std::cos(pose.theta) + width /4.0 * std::sin(pose.theta);
        pt.y = pose.pt.y - (base2tail - width / 4.0) * std::sin(pose.theta) - width /4.0 * std::cos(pose.theta);
        circle_square.emplace_back(pt, radius_square);

        pt.x = pose.pt.x - (base2tail - width / 4.0) * std::cos(pose.theta) - width /4.0 * std::sin(pose.theta);
        pt.y = pose.pt.y - (base2tail - width / 4.0) * std::sin(pose.theta) + width /4.0 * std::cos(pose.theta);
        circle_square.emplace_back(pt, radius_square);
        // cal circle rectangle
        circle_rectangle.clear();
        double radius_rect = std::sqrt(width * width + std::pow((height - width) / 2.0, 2.0)) / 2.0;
        pt.x = pose.pt.x + (height - base2tail - width / 2.0 - (height - width) / 4.0) * std::cos(pose.theta);
        pt.y = pose.pt.y + (height - base2tail - width / 2.0 - (height - width) / 4.0) * std::sin(pose.theta);
        circle_rectangle.emplace_back(pt, radius_rect);
        pt.x = pose.pt.x + (height - base2tail - width / 2.0 - 3.0 * (height - width) / 4.0) * std::cos(pose.theta);
        pt.y = pose.pt.y + (height - base2tail - width / 2.0 - 3.0 * (height - width) / 4.0) * std::sin(pose.theta);
        circle_rectangle.emplace_back(pt, radius_rect);
    }
};

typedef std::vector<PathPoint> Path;
typedef std::vector<OgmPoint>  Ogm;
typedef std::vector<Obstacle>  Obstacles;

#endif  // COMMON_COMMON_DEFINE_H_