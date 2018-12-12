#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef UTILS_H
#define UTILS_H

using namespace cv;
using namespace std;

Mat binary_thresholding(Mat image);
Mat inverse_binary_thresholding(Mat image);
Mat truncate_thresholding(Mat image);
Mat threshold_to_zero(Mat image);
Mat canny_edge_detector(Mat image);
Mat sobel_edge_detector(Mat image);

//Function to convert an integer number to a string.
String intToString(int number);

#endif
