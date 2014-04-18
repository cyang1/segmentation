#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
#include <vector>
#include <cstdint>

struct LabeledComponent {
    unsigned int id;
    cv::Vec3b avg_color;
};

struct Point {
    int row;
    int col;
};

static inline bool within_delta(cv::Vec3b c1, cv::Vec3b c2, cv::Scalar delta)
{
    int d_a = c1[0] - c2[0];
    int d_b = c1[1] - c2[1];
    int d_c = c1[2] - c2[2];
    return abs(d_a) <= delta[0] && abs(d_b) <= delta[1] && abs(d_c) <= delta[2];
}

// Point is row, col
static inline void floodFill(cv::Mat src, cv::Mat mask, LabeledComponent& cmp, Point seed, cv::Scalar diff)
{
    const static uint8_t NUM_DIRECTIONS = 4;
    const static int8_t DIRECTIONS[][2] = { { -1, 0 }, { 0, -1 }, { 1, 0 }, { 0, 1 } };

    unsigned int num_pixels = 1;
    cv::Vec3i total_color = src.at<cv::Vec3b>(seed.row, seed.col);
    std::queue<Point> next_points;
    next_points.push(seed);

    unsigned char* src_data = src.data;
    while (!next_points.empty()) {
        Point cur_pt = next_points.front();
        cv::Vec3b cur_color = src.at<cv::Vec3b>(cur_pt.row, cur_pt.col);
        next_points.pop();

        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int new_row = cur_pt.row + DIRECTIONS[i][0];
            int new_col = cur_pt.col + DIRECTIONS[i][1];

            if (mask.at<uint16_t>(new_row + 1, new_col + 1) == 0) {
                int c_1 = src_data[src.step * new_row + src.channels() * new_col];
                int c_2 = src_data[src.step * new_row + src.channels() * new_col + 1];
                int c_3 = src_data[src.step * new_row + src.channels() * new_col + 2];
                cv::Vec3b next_color(c_1, c_2, c_3);

                if (within_delta(cur_color, next_color, diff))
                {
                    total_color += next_color;
                    num_pixels++;
                    mask.at<uint16_t>(new_row + 1, new_col + 1) = cmp.id;
                    next_points.push({ new_row, new_col });
                }
            }
        }
    }
    cmp.avg_color = total_color * (1.0 / num_pixels);
}

static inline cv::Mat segment(cv::Mat bgr) {
    cv::Mat yuv(bgr.size(), CV_8UC3), segmented(bgr.size(), CV_8UC3);
    cv::cvtColor(bgr, yuv, CV_BGR2YCrCb);

    for (int i = 0; i < bgr.rows; ++i) {
        for (int j = 0; j < bgr.cols; ++j) {
            auto pixel = yuv.data + i*yuv.step + j*yuv.channels();
            auto seg_pixel = segmented.data + i*segmented.step + j*segmented.channels();
            int y = pixel[0], v = pixel[1], u = pixel[2];   //y, cb, cr
            if ((y >= 32 && y < 64 && u >= 136) || //blue
                (y >= 64 && y < 144 && u >= 144))
            {
                seg_pixel[0] = 255;
                seg_pixel[1] = 0;
                seg_pixel[2] = 0;
            }
            else if ((y < 32 && u < 144 && v < 120) ||  //green
                (y >= 32 && y < 96 && v < 120) ||
                (y >= 96 && y < 144 && v < 104))
            {
                seg_pixel[0] = 0;
                seg_pixel[1] = 255;
                seg_pixel[2] = 0;
            }
            else if (v >= 144)  //red
            {
                seg_pixel[0] = 0;
                seg_pixel[1] = 0;
                seg_pixel[2] = 255;
            }
            else
            {
                seg_pixel[0] = 255;
                seg_pixel[1] = 255;
                seg_pixel[2] = 255;
            }
            if (y <= 48)        //black
            {
                seg_pixel[0] = 0;
                seg_pixel[1] = 0;
                seg_pixel[2] = 0;
            }

        }
    }
    return segmented;
}

int main()
{
    cv::VideoCapture cap(1);
    if (!cap.isOpened())
    {
        std::cerr << "Webcam could not be opened." << std::endl;
        return -1;
    }
    cv::namedWindow("orig");
    cv::namedWindow("filled");
    cv::namedWindow("canny");
    cv::namedWindow("segmented");
    cv::namedWindow("filled + segmented");
    for (;;)
    {
        cv::Mat src;
        cap >> src;
        cv::resize(src, src, cv::Size(640, 480));

        // Convert to lab color space so euclidian distance between colors
        // mimics perceptual difference.
        // Might need to convert src to CV_32FC3 later for accuracy.
        cv::Mat src_lab(src.size(), CV_8UC3);
        cv::cvtColor(src, src_lab, CV_BGR2Lab);

        cv::Mat src_copy(src.size(), CV_8UC3);
        cv::bilateralFilter(src_lab, src_copy, 10, 20, 5);
        src_lab = src_copy;

        cv::cvtColor(src_lab, src, CV_Lab2BGR);

        cv::Mat canny;
        cv::Canny(src_lab, canny, 40, 90);
        canny.convertTo(canny, CV_16U, 256);

        // Floodfill regions that are close enough in color.
        std::vector<LabeledComponent> components;
        cv::Mat mask = cv::Mat::zeros(src_lab.rows + 2, src_lab.cols + 2, CV_16U);

        cv::Mat tmp = mask.clone();
        cv::Mat roi(tmp(cvRect(1, 1, src.cols, src.rows)));
        canny.copyTo(roi);
        canny = tmp.clone();
        mask |= canny;

        for (int row = 0; row < mask.rows; row++) {
            auto mask_begin = mask.ptr<uint16_t>(row);
            for (int col = 0; col < mask.cols; col++) {
                if (mask_begin[col] != 0 || row == 0 || col == 0 ||
                    row == mask.rows - 1 || col == mask.cols - 1) {
                    mask_begin[col] = (uint16_t)-1;
                }
            }
        }

        for (int row = 0; row < src_lab.rows; ++row) {
            auto mask_begin = mask.ptr<uint16_t>(row + 1);
            for (int col = 0; col < src_lab.cols; ++col) {
                // Pixel at row, col = Mask at row + 1, col + 1
                if (mask_begin[col + 1] == 0) {
                    LabeledComponent cmp;
                    cmp.id = components.size() + 1;
                    floodFill(src_lab, mask, cmp, { row, col }, cv::Scalar(3, 1, 1));
                    components.push_back(cmp);
                }
            }
        }

        for (int row = 0; row < src.rows; ++row) {
            auto mask_ptr = mask.ptr<uint16_t>(row + 1);
            auto src_ptr = src_lab.ptr<cv::Vec3b>(row);
            for (int col = 0; col < src.cols; ++col) {
                short cmp_id = mask_ptr[col + 1];
                if (cmp_id > 0)
                    src_ptr[col] = components[cmp_id - 1].avg_color;
            }
        }

        // Convert back.
        cv::Mat dest(src.size(), CV_8UC3);
        cv::cvtColor(src_lab, dest, CV_Lab2BGR);

        cv::Mat orig_segmented = segment(src);
        cv::Mat segmented = segment(dest);

        imshow("orig", src);
        imshow("filled", dest);
        imshow("canny", canny);
        imshow("segmented", orig_segmented);
        imshow("filled + segmented", segmented);
        if (cv::waitKey(30) >= 0)
        {
            break;
        }
    }
    return 0;
}