#ifndef CANNY_H
#define CANNY_H

#include <opencv2/opencv.hpp>

cv::Mat Canny(const cv::Mat image,
              uint32_t thresh_low,
              uint32_t thresh_high);

#endif  //CANNY_H