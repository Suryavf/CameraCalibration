#include "utils.h"

Mat hough_transform(Mat image);
void hough_transform_from_video(string name_video);
void find_rings(string name_video);

void gridDetection(cv::Mat &frame,     cv::Mat &binarized,
                   cv::Mat &morphology,cv::Mat &ellipses,
                   cv::Mat &result);
