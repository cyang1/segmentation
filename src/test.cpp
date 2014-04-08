#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdint>

int main()
{
    cv::Mat src = cv::imread("pic.png");
    if(src.empty()){
        std::cerr<<"can't read the image"<<std::endl;
        return -1;
    }

    // Convert to lab color space so euclidian distance between colors
    // mimics perceptual difference.
    // Might need to convert src to CV_32FC3 later for accuracy.
    cv::Mat src_lab(src.size(), CV_8UC3);
    cv::cvtColor(src, src_lab, CV_BGR2Lab);

    cv::imshow("original pic", src);

    // Remove noise.
    // Erode then dilate is best for objects on lighter background.
    // Dilate then erode is best for objects on darker background.
    int dilation_size = 5;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size));
    erode(src_lab, src_lab, element);
    dilate(src_lab, src_lab, element);

    // Floodfill regions that are close enough in color.
    cv::Mat mask = cv::Mat::zeros(src_lab.rows + 2, src_lab.cols + 2, CV_8U);

    for (int row = 0; row < src_lab.rows; ++row) {
        auto src_lab_begin = src_lab.ptr<uint8_t>(row);
        for (int col = 0; col < src_lab.cols; ++col) {
            // Pixel at row, col = Mask at row + 1, col + 1
            if (mask.at<uint8_t>(row + 1, col + 1) == 0) {
                cv::Scalar_<uint8_t> pixel(src_lab_begin[0], src_lab_begin[1], src_lab_begin[2]);
                cv::floodFill(src_lab, mask, cv::Point(col, row), pixel, NULL, cv::Scalar(3, 2, 2), cv::Scalar(3, 2, 2));
            }
            src_lab_begin += 3;
        }
    }

    // Convert back.
    cv::Mat dest(src.size(), CV_8UC3);
    cv::cvtColor(src_lab, dest, CV_Lab2BGR);

    cv::imshow("filled with erode, then dilate", dest);

    cv::waitKey();

    return 0;
}