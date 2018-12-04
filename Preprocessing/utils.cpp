#include "utils.h"

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
	double thresh = 98;
	double maxValue = 255; 

	// Binary Threshold
	threshold(image, result, thresh, maxValue, THRESH_TRUNC);

	return result;
}


Mat threshold_to_zero(Mat image)
{
	Mat result;

	// Set threshold and maxValue
	double thresh = 98;
	double maxValue = 255; 

	// Binary Threshold
	threshold(image, result, thresh, maxValue, THRESH_TOZERO_INV);

	return result;
}

Mat canny_edge_detector(Mat image)
{
	Mat result, detected_edges;

	//int edgeThresh = 1;
	int lowThreshold = 128;
	//int const max_lowThreshold = 128;
	int ratio = 5;
	int kernel_size = 5;

  	/// Canny detector
  	Canny( image, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

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

	/// Generate grad_x and grad_y
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	Sobel( image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );

	/// Gradient Y
	Sobel( image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_y, abs_grad_y );

	/// Total Gradient (approximate)
	addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

	return grad;
}

//Function to convert an integer number to a string.
String intToString(int number)
{
  stringstream ss;
  ss << number;
  return ss.str();
}