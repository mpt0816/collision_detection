/**********************************************************************
* Copyright (C) 2015-2020
*
* NodeName: ChechPolygon
* FileName: ChechPolygon.cpp
*
* Description: 
    
*
* History:
* guohui zhang         2020/08/24    1.0.0    build this module.
************************************************************************/
#include "collision_methods/CV_picture/CV_picture.h"

bool CVPicture::Init() {
  return true;
}

bool CVPicture::CheckCollision() {
//  std::cout << "------into CV_picture------" << std::endl;
    return CheckOgmCollision();
//    auto now = ros::Time::now();
//    CheckObstacleCollision();
//    auto dura = (ros::Time::now() - now).toNSec();
//    std::cout << "time = " << dura << std::endl;
}

void CVPicture::transOgmPointToOgm() {
    auto height = static_cast<int>(map_max / resolution_);
    auto width  = static_cast<int>(map_max / resolution_);
    base_x_ = static_cast<int>(0.0 / resolution_);
    base_y_ = static_cast<int>(0.0 / resolution_);
    data_ = cv::Mat::zeros(height, width, CV_8UC1);
    for (auto& ogm_pt : ogm_) {
        long x = (ogm_pt.pt.x / resolution_) + base_x_;
        long y = (ogm_pt.pt.y / resolution_) + base_y_;
        if (x < 0 || x > width - 1 || y < 0 || y > height - 1) {
            continue;
        }
        data_.data[y * width + x] = 100;
    }
}

bool CVPicture::CheckOgmCollision() {
    if (ogm_.empty()) {
        return false;
    }
    if (resolution_ <= 0.0 || resolution_ >= 10.0) {
        return false;
    }

    transOgmPointToOgm();

    for (size_t i = 0; i < path_.size(); ++i) {
        vehicle_model_.rectangleRep(path_[i]);
        auto veh_contour = vehicle_model_.rect_contour;
        cv::Mat veh_data = cv::Mat::zeros(data_.rows, data_.cols, CV_8UC1);



        auto pts = new cv::Point2i[veh_contour.size()];
        for (size_t j = 0; j < veh_contour.size(); ++j) {
            pts[j].x = base_x_ + static_cast<int>(veh_contour[j].x / resolution_);
            pts[j].y = base_x_ + static_cast<int>(veh_contour[j].y / resolution_);
        }
        const cv::Point2i* ppts[1] = {pts};
        auto npt = (int)veh_contour.size();
        cv::fillPoly(veh_data, ppts, &npt, 1, cv::Scalar(100));
        delete[] pts;
        cv::Mat bitwise_mat;
        cv::bitwise_and(data_, veh_data, bitwise_mat);
        if (cv::countNonZero(bitwise_mat) > 0) {
            index_ = i;
//            return true;
        }
    }
    return false;
}

void CVPicture::genrateObstaclCVMat() {
    auto height = static_cast<int>(map_max / resolution_);
    auto width  = static_cast<int>(map_max / resolution_);

    base_x_ = static_cast<int>(0.0 / resolution_);
    base_y_ = static_cast<int>(0.0 / resolution_);

    data_ = cv::Mat::zeros(height, width, CV_8UC1);
//    auto now = ros::Time::now();
    for (auto &obstacle : obstacles_) {
        auto pts = new cv::Point2i[obstacle.contour.size()];
        for (size_t i = 0; i < obstacle.contour.size(); ++i) {
            pts[i].x = base_x_ + static_cast<int>(obstacle.contour[i].x / resolution_);
            pts[i].y = base_y_ + static_cast<int>(obstacle.contour[i].y / resolution_);
        }
        const cv::Point2i* ppts[1] = {pts};
        auto npt = (int)obstacle.contour.size();
        cv::fillPoly(data_, ppts, &npt, 1, cv::Scalar(100));
        delete[] pts;
    }
//    auto dura = (ros::Time::now() - now).toNSec();
//    std::cout << "time = " << dura << std::endl;
}

bool CVPicture::CheckObstacleCollision() {
//    auto now = ros::Time::now();
    if (obstacles_.empty()) {
        return false;
    }

    if (resolution_ <= 0.0 || resolution_ >= 10.0) {
        return false;
    }

    genrateObstaclCVMat();

    for (size_t i = 0; i < path_.size(); ++i) {
        vehicle_model_.rectangleRep(path_[i]);
        auto veh_contour = vehicle_model_.rect_contour;
        cv::Mat veh_data = cv::Mat::zeros(data_.rows, data_.cols, CV_8UC1);

        auto pts = new cv::Point2i[veh_contour.size()];
        for (size_t j = 0; j < veh_contour.size(); ++j) {
            pts[j].x = base_x_ + static_cast<int>(veh_contour[j].x / resolution_);
            pts[j].y = base_x_ + static_cast<int>(veh_contour[j].y / resolution_);
        }
        const cv::Point2i* ppts[1] = {pts};
        auto npt = (int)veh_contour.size();
        cv::fillPoly(veh_data, ppts, &npt, 1, cv::Scalar(100));
        delete[] pts;
        cv::Mat bitwise_mat;
        cv::bitwise_and(data_, veh_data, bitwise_mat);
        if (cv::countNonZero(bitwise_mat) > 0) {
            index_ = i;
//            return true;
        }
    }

//    auto dura = (ros::Time::now() - now).toNSec();
//    std::cout << "time = " << dura << std::endl;
    return false;
}
