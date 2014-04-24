#include <cassert>
#include <climits>
#include <cstddef>

#include "canny.h"

void sobel(const cv::Mat img,
           cv::Mat edges,
           cv::Mat dir)
{
    int dx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    int dy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

    uint8_t *img_ptr = (uint8_t*)img.data;
    uint8_t *edges_ptr = (uint8_t*)edges.data;
    uint8_t *dir_ptr = (uint8_t*)dir.data;

    for (int i = 1; i < img.rows - 1; i++) {
        for (int j = 1; j < img.cols - 1; j++) {
            int index = i*edges.cols + j;
            int sum_x = 0, sum_y = 0;
            for (int m = -1; m <= 1; m++) {
                for (int n = -1; n <= 1; n++) {
                    uint8_t pixel = img_ptr[(i+m)*img.cols + (j+n)];
                    sum_x += pixel*dx[m+1][n+1];
                    sum_y += pixel*dy[m+1][n+1];
                }
            }
            int sum = sqrt(sum_x*sum_x + sum_y*sum_y);
            edges_ptr[index] = (sum > 255) ? 255 : sum;

            /*
             * Estimate the direction of the sobel gradient, in degrees.
             */
            if (sum_x == 0 && sum_y == 0) {
                dir_ptr[index] = 0;
            }
            else if (sum_x == 0) {
                dir_ptr[index] = 90;
            }
            else {
                double abs_div = abs(sum_y / (double) sum_x);
                if (abs_div > 2.41421356237) {  //atan2(2.4...) ~= 67.5 deg
                    dir_ptr[index] = 0;
                }
                else if (abs_div > 0.414213562373) {    //atan(0.4...) ~= 22.5 deg
                    if ((sum_y / sum_x) > 0) {
                        dir_ptr[index] = 135;
                    }
                    else {
                        dir_ptr[index] = 45;
                    }
                }
                else {
                    dir_ptr[index] = 90;
                }
            }
        }
    }
}

inline void nms_val(uint8_t* nms_ptr, uint8_t* s_ptr, int p, int diff)
{
    if (p < abs(diff)) {
        return;
    }
    if (s_ptr[p] > s_ptr[p + diff] && s_ptr[p] > s_ptr[p - diff]) {
        nms_ptr[p] = s_ptr[p];
    }
    else {
        nms_ptr[p] = 0;
    }
} 

 /*
  * For each pixel, check the adjacent pixels in the direction indicated by
  * the sobel gradient. Suppress the pixel if either of the adjacent pixels
  * are brighter. This gets rid of thick lines from the sobel operator.
 */
void non_max_suppression(const cv::Mat s, cv::Mat nms, const cv::Mat dir)
{
    uint8_t *nms_ptr = (uint8_t*)nms.data;
    uint8_t *s_ptr = (uint8_t*)s.data;
    uint8_t *dir_ptr = (uint8_t*)dir.data;

    for (int i = 0; i < nms.rows; i++) {
        for (int j = 0; j < nms.cols; j++) {
            int p = i*nms.cols + j;
            switch (dir_ptr[p]) {
                case 0:     // N/S
                    nms_val(nms_ptr, s_ptr, p, nms.cols);
                    break;
                case 45:    // NW/SE
                    nms_val(nms_ptr, s_ptr, p, nms.cols + 1);
                    break;
                case 90:    // W/E
                    nms_val(nms_ptr, s_ptr, p, 1);
                    break;
                case 135:   // NE/SW
                    nms_val(nms_ptr, s_ptr, p, nms.cols - 1);
                    break;
                default:
                    break;
            }
        }
    }
}

void trace(int i, int j, uint32_t low,
          uint8_t *in_ptr, uint8_t *out_ptr, uint8_t *dir_ptr,
          int rows, int cols)
{
    if (out_ptr[i * cols + j] == 0) {
        out_ptr[i * cols + j] = 255;
        switch (dir_ptr[i * cols + j]) {
            case 0:
                i += 1;
                break;
            case 45:
                i += 1;
                j += 1;
                break;
            case 90:
                j += 1;
                break;
            case 135:
                i += 1;
                j -= 1;
                break;
            default:
                break;
        }
        if (i >= 0 && i < rows && j >= 0 && j < cols &&
            in_ptr[i * cols + j] >= low) {
            trace(i, j, low, in_ptr, out_ptr, dir_ptr, rows, cols);
        }
    }
}

/*
 * Finding edges with a simple threshold is insufficient, since edges with
 * intensities around the threshold may have streaking (holes in the edge).
 * The solution is to have two thresholds: the high threshold is necessary to
 * start an edge, and the low threshold is used to continue an edge. Edges are
 * continued in the direction of the gradient found in sobel.
 */
cv::Mat hysteresis(uint32_t high, uint32_t low, const cv::Mat nms, const cv::Mat dir)
{
    cv::Mat out = cv::Mat::zeros(nms.size(), CV_8UC1);
    uint8_t *nms_ptr = (uint8_t*)nms.data;
    uint8_t *out_ptr = (uint8_t*)out.data;
    uint8_t *dir_ptr = (uint8_t*)dir.data;
    for (int i = 1; i < out.rows - 1; i++) {
        for (int j = 1; j < out.cols - 1; j++) {
            if (nms_ptr[i * out.cols + j] >= high) {
                trace(i, j, low, nms_ptr, out_ptr, dir_ptr, out.rows, out.cols);
            }
        }
    }

    return out;
}

cv::Mat Canny(const cv::Mat image, uint32_t thresh_low, uint32_t thresh_high)
{
    assert(thresh_low >= 0 && thresh_high >= 0);
    //assert(image.channels() == 1); 
    //cv::Mat grey(image.size(), CV_8UC1);
    cv::Mat s(image.size(), CV_8UC1);
    cv::Mat dir(image.size(), CV_8UC1);
    cv::Mat nms(image.size(), CV_8UC1);
    //cv::cvtColor(image, grey, CV_BGR2GRAY);
    sobel(image, s, dir);
    non_max_suppression(s, nms, dir);
    return hysteresis(thresh_high, thresh_low, nms, dir);
}