#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdint>

int main()
{
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        return -1;
    }
    cv::namedWindow("orig");
    cv::namedWindow("filled");
    cv::namedWindow("segmented");
    for (;;)
    {
        cv::Mat src;
        cap >> src;
        cv::resize(src, src, cv::Size(640, 480));
        // Seems to be the most effective way to remove noise.
        cv::medianBlur(src, src, 5);

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
        cv::Mat dest(src.size(), CV_8UC3), yuv(src.size(), CV_8UC3), segmented(src.size(), CV_8UC3);
        cv::cvtColor(src_lab, dest, CV_Lab2BGR);
        cv::cvtColor(dest, yuv, CV_BGR2YCrCb);

        for (int i = 0; i < src.rows; ++i) {
            for (int j = 0; j < src.cols; ++j) {
                int y = yuv.at<cv::Vec3b>(i, j)[0];
                int u = yuv.at<cv::Vec3b>(i, j)[1];
                int v = yuv.at<cv::Vec3b>(i, j)[2];
                if (y <= 80)        //black
                {
                    segmented.at<cv::Vec3b>(i, j) = {0, 0, 0};
                }
                else if (v >= 144)  //red
                {
                    segmented.at<cv::Vec3b>(i, j) = {0, 0, 255};
                }
                else if ((y < 32 && u < 120 && v < 120) ||  //green
                         (y >= 32 && y < 96 && v < 120) ||
                         (y >= 96 && y < 144 && v < 104))
                {
                    segmented.at<cv::Vec3b>(i, j) = {0, 255, 0};
                }
                else if ((y >= 32 && y < 64 && u >= 136) || //blue
                         (y >= 64 && y < 144 && u >= 144))
                {
                    segmented.at<cv::Vec3b>(i, j) = {255, 0, 0};
                }
                else
                {
                    segmented.at<cv::Vec3b>(i, j) = {255, 255, 255};
                }
            }
        }


        imshow("orig", src);
        imshow("filled", dest);
        imshow("segmented", segmented);
        if (cv::waitKey(30) >= 0)
        {
            break;
        }
    }
    return 0;
}