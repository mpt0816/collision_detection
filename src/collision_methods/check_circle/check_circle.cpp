#include <src/toolbox/k-d-tree.h>
#include "collision_methods/check_circle/check_circle.h"

bool CheckCircle::Init() {
    return true;
}

bool CheckCircle::CheckCollision() {
    if (ogm_.empty() || path_.empty()) {
        return false;
    }
    for (size_t i = 0; i < path_.size(); ++i) {
        vehicle_model_.circleRep(path_.at(i));
        for (auto& ogm_pt : ogm_) {
            double dis = calDisBetweenPoint(ogm_pt.pt, vehicle_model_.circle_bounding.center);
            if (dis > vehicle_model_.circle_bounding.radius) {
                continue;
            } else {
                for (size_t j = 0; j < vehicle_model_.circle_square.size(); ++j) {
                    dis = calDisBetweenPoint(ogm_pt.pt, vehicle_model_.circle_square[j].center);
                    if (dis <= vehicle_model_.circle_square[j].radius) {
                        index_ = i;
//                        return true;
                        continue;
                    }
                }
                for (size_t j = 0; j < vehicle_model_.circle_rectangle.size(); ++j) {
                    dis = calDisBetweenPoint(ogm_pt.pt, vehicle_model_.circle_rectangle[j].center);
                    if (dis <= vehicle_model_.circle_rectangle[j].radius) {
                        index_ = i;
//                        return true;
                        continue;
                    }
                }
            }
        }
    }
    return false;
}

//bool CheckCircle::CheckCollision() {
//    if (ogm_.empty() || path_.empty()) {
//        return false;
//    }
//
//    alglib::kdtree kdt;
//    alglib::real_2d_array ogm_pts;
//    alglib::ae_int_t nx = 2;
//    alglib::ae_int_t ny = 0;
//    alglib::ae_int_t normtype = 2;
//    double pt[ogm_.size() * 2];
//    for (size_t i = 0; i < ogm_.size(); ++i) {
//        double x = ogm_[i].pt.x;
//        double y = ogm_.at(i).pt.y;
//        pt[i * 2] = x;
//        pt[i * 2 + 1] = y;
//    }
//    ogm_pts.setcontent(ogm_.size(), 2, pt);
//    alglib::kdtreebuild(ogm_pts, nx, ny, normtype, kdt);
//    for (size_t i = 0; i < path_.size(); ++i) {
//        vehicle_model_.circleRep(path_.at(i));
//        double pose[2] = {path_.at(i).pt.x, path_.at(i).pt.y};
//        alglib::real_1d_array x;
//        x.setcontent(2, pose);
//        alglib::ae_int_t num = alglib::kdtreequeryrnn(kdt, x, vehicle_model_.circle_bounding.radius + 1.0);
//        if (num > 0) {
//            alglib::real_2d_array point;
//            alglib::kdtreequeryresultsx(kdt, point);
//            for (size_t j = 0; j < point.rows(); ++j) {
//                Point pt = {point(j, 1), point(j, 0)};
//                for (size_t j = 0; j < vehicle_model_.circle_square.size(); ++j) {
//                    double dis = calDisBetweenPoint(pt, vehicle_model_.circle_square[j].center);
//                    if (dis <= vehicle_model_.circle_square[j].radius) {
//                        index_ = i;
//                        return true;
//                    }
//                }
//                for (size_t j = 0; j < vehicle_model_.circle_rectangle.size(); ++j) {
//                    double dis = calDisBetweenPoint(pt, vehicle_model_.circle_rectangle[j].center);
//                    if (dis <= vehicle_model_.circle_rectangle[j].radius) {
//                        index_ = i;
//                        return true;
//                    }
//                }
//            }
//        }
//    }
//    return false;
//}

double CheckCircle::calDisBetweenPoint(const Point &p1, const Point &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}
