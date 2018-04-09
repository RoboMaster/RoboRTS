#include <ros/ros.h>
#include <csignal>

#include "image.h"
#include "cpp2c.hpp"
#include "modules/perception/detection/util/cv_toolbox.h"

#ifdef __cplusplus
extern "C" {
#endif

IplImage* GetNextImage(int camera_id) {
  cv::Mat mat_img;
  static rrts::perception::detection::CVToolbox cv_toolbox;
  cv_toolbox.NextImage(mat_img, camera_id);
  if(!mat_img.empty()) {
    IplImage src = IplImage(mat_img);
    IplImage *out = cvCloneImage(&src);
    return out;
  } else
    return nullptr;
}

void Notice(char* message) {
  NOTICE(std::string(message))
}

void ShowImage(IplImage *img) {
  cv::Mat img_for_show;
  img_for_show = cv::cvarrToMat(img);
  cv::imshow("demo", img_for_show);
}

#ifdef __cplusplus
};
#endif