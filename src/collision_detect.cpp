#include "collision_detect.h"
#include <cstdlib>
#include <collision_methods/gjk_check/gjk_check.h>

CollisionDetect::CollisionDetect() {

  collosion_methods_ = std::make_shared<ChechPolygon>();
//  collosion_methods_ = std::make_shared<CVPicture>();
//  collosion_methods_ = std::make_shared<CheckCircle>();
//    collosion_methods_ = std::make_shared<SatCheck>();
//    collosion_methods_ = std::make_shared<GjkCheck>();

    SimPathPoints();
//    SimObstacles();
    SimVehicleModel();
    generateOgm(1000);
//    generateObstacle(10000);
    collosion_methods_ -> SetCollisionData(path_, ogm_, obstacles_, vehicle_model_);
}
CollisionDetect::~CollisionDetect() {}

bool CollisionDetect::ComputeCollision() {
   collision = collosion_methods_ -> CheckCollision();
}

void CollisionDetect::printResult() {
    if (collision) {
        std::cout << "Collision generate at the " << collosion_methods_->index_ + 1 << " th of the path!" << std::endl;
    } else {
        std::cout << "NO Collision." << std::endl;
    }
}

bool CollisionDetect::SimPathPoints() {
    path_.clear();
    const int path_size = map_max * 2 + 1;
    for (size_t i = 0; i < path_size; ++i) {
        PathPoint path_point;
        path_point.pt = {i * 0.5, i * 0.5};
        path_point.theta = 0.785;
        path_.push_back(path_point);
    }
//  path_.clear();
//  path_ = {{0, 0, 0.785},     {0.5, 0.5, 0.785}, {1.0, 1.0, 0.785}, {1.5, 1.5, 0.785}, {2.0, 2.0, 0.785},
//           {2.5, 2.5, 0.785}, {3.0, 3.0, 0.785}, {3.5, 3.5, 0.785}, {4.0, 4.0, 0.785}, {4.5, 4.5, 0.785},
//           {5.0, 5.0, 0.785}, {5.5, 5.5, 0.785}, {6.0, 6.0, 0.785}, {6.5, 6.5, 0.785}, {7.0, 7.0, 0.785},
//           {7.5, 7.5, 0.785}, {8.0, 8.0, 0.785}, {8.5, 8.5, 0.785}, {9.0, 9.0, 0.785}, {9.5, 9.5, 0.785},
//           {10.0, 10.0, 0.785}};
}

bool CollisionDetect::SimObstacles() {
  ogm_.clear();
  ogm_ = {{8.5, 8.1}, {3.1,4.5}, {5.8, 1.2}, {2.2, 2.5}, {6.8, 4.5}};
}

bool CollisionDetect::SimVehicleModel() {
  vehicle_model_.reset();
}

void CollisionDetect::generateOgm(int num) {
    ogm_.clear();
    OgmPoint ogm_pt;
//    srand((int)time(0));    // 产生随机种子
    for (int i = 0; i < num; ++i) {
        ogm_pt.pt.x = rand() / double(RAND_MAX) * map_max;
        ogm_pt.pt.y = rand() / double(RAND_MAX) * map_max;
        ogm_pt.p    = 1.0;
        ogm_.emplace_back(ogm_pt);
    }
}

void CollisionDetect::generateObstacle(int num) {
    double height = 1.0;
    double width  = 1.0;
    obstacles_.clear();

//    srand((int)time(0));
    for (int i = 0; i < num; ++i) {
        Obstacle obj;
        obj.height = height;
        obj.width  = width;
        obj.center.x = rand() / double(RAND_MAX) * map_max;
        obj.center.y = rand() / double(RAND_MAX) * map_max;
        double theta = rand() / double(RAND_MAX) * M_PI_2;
        obj.theta = theta;
        Point pt;
        pt.x = obj.center.x + height / 2.0 * std::cos(theta) - width / 2.0 * std::sin(theta);
        pt.y = obj.center.y + height / 2.0 * std::sin(theta) + width / 2.0 * std::cos(theta);
        obj.contour.emplace_back(pt);

//        // add one point
//        pt.x = obj.center.x + height * std::cos(theta);
//        pt.y = obj.center.y + height * std::sin(theta);
//        obj.contour.emplace_back(pt);

        pt.x = obj.center.x + height / 2.0 * std::cos(theta) + width / 2.0 * std::sin(theta);
        pt.y = obj.center.y + height / 2.0 * std::sin(theta) - width / 2.0 * std::cos(theta);
        obj.contour.emplace_back(pt);

        pt.x = obj.center.x - height / 2.0 * std::cos(theta) + width / 2.0 * std::sin(theta);
        pt.y = obj.center.y - height / 2.0 * std::sin(theta) - width / 2.0 * std::cos(theta);
        obj.contour.emplace_back(pt);

//        // add one point
//        pt.x = obj.center.x - height * std::cos(theta);
//        pt.y = obj.center.y - height * std::sin(theta);
//        obj.contour.emplace_back(pt);

        pt.x = obj.center.x - height / 2.0 * std::cos(theta) - width / 2.0 * std::sin(theta);
        pt.y = obj.center.y - height / 2.0 * std::sin(theta) + width / 2.0 * std::cos(theta);
        obj.contour.emplace_back(pt);
        obj.calMinMaxBounding();
        obstacles_.push_back(obj);
    }
}
