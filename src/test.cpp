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
    cv::imshow("original pic", src);

    // Seems to be the most effective way to remove noise.
    cv::medianBlur(src, src, 5);
    cv::imshow("blurred pic", src);

    // Convert to lab color space so euclidian distance between colors
    // mimics perceptual difference.
    // Might need to convert src to CV_32FC3 later for accuracy.
    cv::Mat src_lab(src.size(), CV_8UC3);
    cv::cvtColor(src, src_lab, CV_BGR2Lab);

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

    cv::imshow("filled", dest);

    cv::waitKey();

    return 0;
}