#include "utils.h"
#include "../Patron/mapping.h"

Mat hough_transform(Mat image);
void hough_transform_from_video(string name_video);
void find_rings(string name_video);

void gridDetection(cv::Mat &frame     , cv::Mat  &binarized,
                   cv::Mat &morphology, cv::Mat  &ellipses ,
                   cv::Mat &result, cv::RotatedRect &minRec,
                   vector<Point2f> &good_ellipses,
                   int &ellipseCount);
void calcBoardCornerPositions(cv::Size &boardSize, float squareSize, vector<Point3f>& corners);
double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
                                 const vector<vector<Point2f> >& imagePoints,
                                 const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs,
                                 vector<float>& perViewErrors);
