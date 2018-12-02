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

//Function to convert an integer number to a string.
String intToString(int number)
{
  stringstream ss;
  ss << number;
  return ss.str();
}

Mat hough_transform(Mat image)
{

  //Variables to write the radius and center position on the screen.
  string radiusStr;
  string xcenterStr;
  string ycenterStr;
  int Rvalue;
  int Xvalue;
  int Yvalue;



  // Open the original colored image and convert to a grayscaled image
  Mat coloredimage;
  Mat grayimage;



  //vector to store the center value ( x and y coordinates ) and the radius of each detected circle
  vector<Vec3f> circles;

  //Open and resize the colored image.
  coloredimage = image;

  // The next line could be neccessary
  //resize(coloredimage, coloredimage, Size(640, 480));
  
  //imshow("Original Image", coloredimage);

  //convert colored image to gray scale
  cvtColor(coloredimage, grayimage, CV_BGR2GRAY);

  // Apply blur to Reduce the noise to avoid false detections.
  GaussianBlur(grayimage, grayimage, Size(9, 9), 2, 2);

  // Apply the Hough Transform to find the circles (use the gray scale image as input)
  //Arguments: 1: input image( grayscaled ) . 2: vector to tore the circle parameters .3: detection method,
  // 4: inverse ratio of resolution . 5 minimum distance between detected centers. 6: upper threshold for the internal canny edge detector
  //7: threshold for center detection . 8: Minimum radius to be detected (0=unknown) . 9: maximum radius to be detected


  HoughCircles(grayimage, circles, CV_HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);


  // Draw the circles detected
  for (size_t i = 0; i < circles.size(); i++)
  {
    //Get the informations from the circles vector generated by the function HoughCircles. 
    //X center coordinate is circles[i][0]  , Y center coordinate is circles[i][1] , radius is circles[i][2] 
    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);

    //Store these values into variables to be converted into string and displayed on the image
    Rvalue = radius;
    Xvalue = cvRound(circles[i][0]);
    Yvalue = cvRound(circles[i][1]);

    //DRAWING THE CENTER OF THE CIRCLE
    //Use the circle function to draw the center of the detected circle
    //Use the center coordinate and a radius of 3 to just draw a point on the center.
    circle(coloredimage, center, 3, Scalar(0, 255, 0), -1, 8, 0);

    //DRAWING THE CIRCLE CONTOUR.
    //Use the circle function to draw the detected circle on the image
    //Use the center coordinate and the radius coordinate detected by the HoughCircles function
    circle(coloredimage, center, radius, Scalar(0, 0, 255), 3, 8, 0);



    //Convert the integer Center point and radius values to string
    radiusStr = intToString(Rvalue);
    xcenterStr = intToString(Xvalue);
    ycenterStr = intToString(Yvalue);

    //Display on the colored image the center and radius values.
    putText(coloredimage, "(" + xcenterStr + "," + ycenterStr + ")", Point(Xvalue, Yvalue - 20), 1, 1, Scalar(0, 255, 0), 2);


    cout << "radius:" << radiusStr << "    Column:" << xcenterStr << "    Row:" << ycenterStr;
  }

  return coloredimage;

}


int main()
{

	// Read image
	Mat colored_image = imread("patron.png", CV_LOAD_IMAGE_COLOR);
	Mat image = imread("patron.png", IMREAD_GRAYSCALE);
	
	//Mat binary_image = binary_thresholding(image);
	//Mat binary_image = inverse_binary_thresholding(image);
	//Mat binary_image = truncate_thresholding(image);
	//Mat binary_image = threshold_to_zero(image);
	//imshow( "Display window", binary_image);


	//Mat canny_image = canny_edge_detector(image);
	//canny_image = inverse_binary_thresholding(canny_image);
	//imshow( "Display window", canny_image);


	//Mat sobel_image = sobel_edge_detector(image);
	//imshow( "Display window", sobel_image);

  	Mat hough_image = hough_transform(colored_image);

    // Show your results
  	namedWindow("Circle Detector", CV_WINDOW_AUTOSIZE);
  	imshow("Circle Detector", hough_image);



    waitKey(0);                                        // Wait for a keystroke in the window
    return 0;
}


