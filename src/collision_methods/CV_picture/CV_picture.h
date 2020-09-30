#ifndef COLLISION_METHODS_CV_PICTURE_H_
#define COLLISION_METHODS_CV_PICTURE_H_
#include "collision_methods/base/collision_base.h"
#include "collision_detect.h"
#include "opencv2/opencv.hpp"

class CVPicture : public CollisionBase {
public:
  CVPicture() {
      resolution_ = 0.1;
      is_init_ = true;
  }
  ~CVPicture() {};

private:
  bool Init();
  bool CheckCollision();
  bool CheckOgmCollision();
  bool CheckObstacleCollision();
  void transOgmPointToOgm();
  void genrateObstaclCVMat();

private:
    double  resolution_;
    int     base_x_;
    int     base_y_;
    cv::Mat data_;
};
#endif  // COLLISION_METHODS_CV_PICTURE_H_