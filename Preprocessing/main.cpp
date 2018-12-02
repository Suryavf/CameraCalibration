#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;


Mat binary_thresholding(Mat image)
{
	Mat result;

	// Set threshold and maxValue
	double thresh = 98;
	double maxValue = 255; 

	// Binary Threshold
	threshold(image, result, thresh, maxValue, THRESH_BINARY);

	return result;
}


Mat inverse_binary_thresholding(Mat image)
{
	Mat result;

	// Set threshold and maxValue
	double thresh = 0;
	double maxValue = 255; 

	// Binary Threshold
	threshold(image, result, thresh, maxValue, THRESH_BINARY_INV);

	return result;
}

Mat truncate_thresholding(Mat image)
{
	Mat result;

	// Set threshold and maxValue
	double thresh = 128;
	double maxValue = 255; 

	// Binary Threshold
	threshold(image, result, thresh, maxValue, THRESH_TRUNC);

	return result;
}


Mat threshold_to_zero(Mat image)
{
	Mat result;

	// Set threshold and maxValue
	double thresh = 200;
	double maxValue = 255; 

	// Binary Threshold
	threshold(image, result, thresh, maxValue, THRESH_TOZERO_INV);

	return result;
}

Mat canny_edge_detector(Mat image)
{
	Mat result, detected_edges;

	int edgeThresh = 1;
	int lowThreshold = 120;
	int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;

	
	/// Reduce noise with a kernel 3x3
  	blur(image, detected_edges, Size(3,3) );

  	/// Canny detector
  	Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  	/// Using Canny's output as a mask, we display our result
  	result = Scalar::all(0);

  	image.copyTo( result, detected_edges);
  	
  	return result;
}

Mat sobel_edge_detector(Mat image)
{

	Mat grad;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;

	//GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

	/// Generate grad_x and grad_y
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	//Scharr( image, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	Sobel( image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );

	/// Gradient Y
	//Scharr( image, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	Sobel( image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_y, abs_grad_y );

	/// Total Gradient (approximate)
	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

	return grad;
}



int main()
{

	// Read image
	Mat image1 = imread("patron.png", CV_LOAD_IMAGE_COLOR);
	Mat image = imread("patron.png", IMREAD_GRAYSCALE);
	
	//Mat binary_image = binary_thresholding(image);
	//Mat binary_image = inverse_binary_thresholding(image);
	//Mat binary_image = truncate_thresholding(image);
	//Mat binary_image = threshold_to_zero(image);
	//imshow( "Display window", binary_image);


	//Mat canny_image = canny_edge_detector(image);
	//canny_image = inverse_binary_thresholding(canny_image);
	//imshow( "Display window", canny_image);


	Mat sobel_image = sobel_edge_detector(image);
	imshow( "Display window", sobel_image);


    waitKey(0);                                        // Wait for a keystroke in the window
    return 0;
}


