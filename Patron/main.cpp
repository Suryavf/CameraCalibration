#include "mapping.h"

int main( int argc, char** argv ) {
/*
 *  Load image
 *  ----------  
 */
    cv::Mat src;
    src = cv::imread("patron.png" , CV_LOAD_IMAGE_COLOR);
    
    if(! src.data ) {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

/*
 *  Gray scale
 *  ----------  
 */
    cv::Mat src_gray;
    cv::cvtColor( src, src_gray, CV_BGR2GRAY );

/*
 *  Hough Circles
 *  -------------  
 */
    GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );
    std::vector<cv::Vec3f> circles;
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );

    std::vector<Point> pts(circles.size());

    for( uint i = 0; i < pts.size(); i++ ){
        pts[i] = Point(circles[i][0],circles[i][1]);
    }

/*
 *  Mapping
 *  -------  
 */
    std::vector< std::vector<Point> > patron;
    mapping(pts,patron,src.rows,src.cols,5,4);

// -----------------------------------------------------------------------------------------------------

    /*
    for( size_t i = 0; i < circles.size(); i++ ){
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        // circle center
        circle( src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }
    */

/*
 *  Display image
 *  -------------  
 */
    /*
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", src );

    cv::waitKey(0);
    */
    return 0;
}