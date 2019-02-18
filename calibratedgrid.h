#ifndef CALIBRATEDGRID_H
#define CALIBRATEDGRID_H
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <QString>
#include <omp.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;


bool findPattern(cv::Mat  &frame      , cv::Mat &binarized,
                 cv::Mat  &morphology , cv::Mat &ellipses ,
                 cv::Mat  &result     ,
                 cv::Size &patternSize,
                 cv::RotatedRect &minRec,
                 vector<Point2f> &framePoints,
                 QString &type,
                 int     &countPoints,
                 float   &timeLapse);

bool runCalibrationAndSave(vector<vector<Point2f> > &imagePoints,
                           QString &type, float &squareSize,
                           Size &boardSize, Size &imageSize,
                           Mat  &cameraMatrix, Mat& distCoeffs,
                           double &totalAvgErr);

vector<Point2f>getExtremePoints(Mat view, vector<Point2f> pointBuf, vector<Point2f> dst_vertices, int offset);
void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point2f>& corners, QString type);

bool findRingsGrid2( Mat &view, Mat &binarized, Mat &morphology, Mat &ellipses, Mat &result, RotatedRect &minRect, Size &boardSize, vector<Point2f> &puntos);

void runCalibrationRuntine();

#endif // CALIBRATEDGRID_H
