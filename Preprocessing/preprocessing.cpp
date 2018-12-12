#include "preprocessing.h"

void find_rings(string name_video){
/*
    Define parameters 
	-----------------
 */
	//Variables to store  from the video , and a converted version of the video
	Mat coloredimage;
	Mat grayimage;
	Mat threshold_output; // binarized image
  	vector<vector<Point> > contours; //contours (elipses)
  	vector<Vec4i> hierarchy; // 
  	Mat drawing;
  	Scalar color;

	// Erosion & dilation
	Mat element = getStructuringElement( MORPH_ELLIPSE, Size ( 5, 5 ),Point( 2, 2 ));

  	float x_t0, y_t0;
  	float x_t1, y_t1;
  	float holgura = 1;
  	float distance;
  	int counter = 0;

	
	
	int min_x = 1000000, min_y = 1000000;
    int max_x = 0, max_y = 0;
    bool flag = false;
    

/*
    Video configure
	---------------
 */
	VideoCapture capture(name_video);

	//Set Capture device properties.
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	//Check for Failure
	if (!capture.isOpened()){
		printf("Failed to open the video");
	}

	//auxiliar variable to quit the loop and end the program
	char key = 0;

	//Loop will stop if "q" is pressed in the keyboard
	while (key != 'q'){

		//Capture a frame of the webcam live video and store it on the image variable
		capture >> coloredimage;

		int width = coloredimage.cols;
		int height = coloredimage.rows;

		//Resize this frame and convert to gray scale
		cvtColor(coloredimage, grayimage, CV_BGR2GRAY);
		GaussianBlur(grayimage, grayimage, Size(9, 9), 2, 2);

		/// Detect edges using Threshold
  		adaptiveThreshold(grayimage, threshold_output, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,11,3);
  		imshow("Binarized Image", threshold_output);

  		// Erode/dilate
	    erode ( threshold_output, threshold_output, element );
	    dilate( threshold_output, threshold_output, element );
  		imshow( "Erosion/Dilation", threshold_output );

  		
  		/// Find contours
  		findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		/// Find the  ellipses for each contour
		vector<RotatedRect> minEllipse( contours.size() );


  		for( int i = 0; i < contours.size(); i++ )
     	{ 
     		
       		if( contours[i].size() > 5 )
         	{ 
         		minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
         	}
     	}

  		/// Draw contours + rotated rects + ellipses
  		drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  		//cout <<"number of elipses: "<<contours.size()<<endl;
  		
  		vector<int> ellipses_to_verify( contours.size() );
  		vector<Point2f> good_ellipses;

  		for( int i = 0; i< contours.size()-1; i++ )
     	{
     		if(minEllipse[i].size.width > width/8 || minEllipse[i].size.height > height/6 || 
     		   minEllipse[i].size.width < 6 || minEllipse[i].size.height < 6 || ellipses_to_verify[i] ==
     		   -1)
     		{

     		}

     		else
     		{

	       		color = Scalar(0,0,255);//Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	       		
	       		// contour
	       		//drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
	       		
	       		// ellipse
	       		x_t0 = minEllipse[i].center.x;
	       		y_t0 = minEllipse[i].center.y;
	       		
	       		int counter_verify = 1;
	       		float sumx = x_t0;
	       		float sumy = y_t0;

	       		for( int j = i+1; j< contours.size()-1; j++ )
	       		{
	       			x_t1 = minEllipse[j].center.x;
		       		y_t1 = minEllipse[j].center.y;

		       		distance = sqrt(pow(x_t0 - x_t1, 2) + pow(y_t0 - y_t1, 2));

		       		if(distance <= holgura){
			       		counter++;
			       		counter_verify++;
			       		sumx += x_t1;
			       		sumy += y_t1;
			       		ellipses_to_verify[j] = -1;
			       	}
			    }

			    if(counter_verify > 1)
			    {
			    	good_ellipses.push_back(Point2f(sumx/counter_verify, sumy/counter_verify));
			    }

		    }
     	}

     	// DE ACUERDO AL PATRÓN
     	if(good_ellipses.size() == 12)
     	{
     		flag = true;
     		for(int i=0; i<good_ellipses.size(); i++)
     		{
     			if(good_ellipses[i].x < min_x)
     				min_x = good_ellipses[i].x;

     			if(good_ellipses[i].y < min_y)
     				min_y = good_ellipses[i].y;

     			if(good_ellipses[i].x > max_x)
     				max_x = good_ellipses[i].x;

     			if(good_ellipses[i].y > max_y)
     				max_y = good_ellipses[i].y;
     		}
     	}

     	/*else
     		flag = false;
*/
     	for (int i = 0; i < good_ellipses.size(); ++i){
     		circle(drawing, good_ellipses[i], 2, Scalar(0,255,0), -1, 8, 0); //-1 full circle
			cv::putText(drawing,     std::to_string(i),
                                      good_ellipses[i], // Coordinates
                               cv::FONT_HERSHEY_DUPLEX, // Font
                                                   0.6, // Scale. 2.0 = 2x bigger
                                   cv::Scalar(255,0,0), // BGR Color
                                                    2); // Line Thickness (Optional)
     	}

     	counter = 0;


     	cout<<"Number of good ellipses: "<<good_ellipses.size()<<endl;
     		cout<<"AUXILIOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"<<endl;

     	namedWindow("HELLO", CV_WINDOW_AUTOSIZE);
		imshow("HELLO", drawing);

		key = waitKey(10);

	}

}


void gridDetection(cv::Mat &frame, cv::Mat &binarized, cv::Mat &morphology, cv::Mat &ellipses, cv::Mat &result){

    int width  = frame.cols;
    int height = frame.rows;
    float holgura = 1;
    float distance;
    int counter = 0;
    vector<Vec4i> hierarchy;

/*
    Binarized Image
    ---------------
 */
    cv::Mat gray;
    cvtColor(frame, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);
    adaptiveThreshold(gray, binarized, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,11,3);

/*
    Morphological Transformations
    -----------------------------
 */
    cv::Mat element = getStructuringElement( MORPH_ELLIPSE, Size ( 5, 5 ),Point( 2, 2 ));
    erode ( binarized,  binarized, element );
    dilate( binarized, morphology, element );

/*
    Ellipse detection
    -----------------
 */
    std::vector<std::vector<cv::Point> > contours;
    findContours( morphology, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    std::vector<RotatedRect> minEllipse( contours.size() );

    for( size_t i = 0; i < contours.size(); i++ ){
        if( contours[i].size() > 5 ){
            minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }
    }

/*
    Ellipse purge
    -------------
 */
    vector<int> ellipses_to_verify( contours.size() );
    vector<Point2f> good_ellipses;
    int counter_verify;
    float x_t0, y_t0;
    float x_t1, y_t1;
    float disX, disY;
    float sumx, sumy;

    for( size_t i = 0; i< contours.size()-1; i++ ){
        if( minEllipse[i].size.width > width/8 || minEllipse[i].size.height > height/6 ||
            minEllipse[i].size.width < 6       || minEllipse[i].size.height < 6        ||
            ellipses_to_verify[i] == -1){}
        else{

            x_t0 = minEllipse[i].center.x;
            y_t0 = minEllipse[i].center.y;

            counter_verify = 1;
            sumx = x_t0;
            sumy = y_t0;

            for( size_t j = i+1; j< contours.size()-1; j++ ){
                x_t1 = float(minEllipse[j].center.x);
                y_t1 = float(minEllipse[j].center.y);

                disX = x_t0 - x_t1;
                disY = y_t0 - y_t1;
                distance = sqrt(disX*disX + disY*disY);

                if(distance <= holgura){
                    counter++;
                    counter_verify++;
                    sumx += x_t1;
                    sumy += y_t1;
                    ellipses_to_verify[j] = -1;
                }
            }

            if(counter_verify > 1){
                good_ellipses.push_back(Point2f(sumx/counter_verify, sumy/counter_verify));
            }

        }
    }

    // Draw ellipse
    ellipses = Mat::zeros( morphology.size(), CV_8UC3 );

    for (size_t i = 0; i < good_ellipses.size(); ++i){
        circle(ellipses, good_ellipses[i], 2, Scalar(0,255,0), -1, 8, 0); //-1 full circle
        cv::putText(ellipses,      std::to_string(i),
                                    good_ellipses[i], // Coordinates
                             cv::FONT_HERSHEY_DUPLEX, // Font
                                                 0.6, // Scale. 2.0 = 2x bigger
                                 cv::Scalar(0,0,255), // BGR Color
                                                  2); // Line Thickness (Optional)
    }
}
