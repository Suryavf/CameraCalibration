#include "preprocessing.h"


void distance(const cv::Point2f &p1, const cv::Point2f &p2, float &dist){
    float x = p1.x - p2.x;
    float y = p1.y - p2.y;
    dist =  sqrt( x*x + y*y );
}



/*
 *  Get corners
 *  -----------
 */
void getCorners(const std::vector<cv::Point2f> &points,
                    uint &IL, uint &IR,
                    uint &SL, uint &SR,
                    const cv::RotatedRect &ROI){
    float aux;
    float MAX_T = std::numeric_limits<T>::max();
    float IL_min = MAX_T, SL_min = MAX_T,
          IR_min = MAX_T, SR_min = MAX_T;

    Point2f diff;
    cv::Point2f cornerROI[4];
    ROI.points( cornerROI );
    for( uint i = 0; i < points.size(); i++ ){

        // Inferior Left
        diff = points[i] - cornerROI[1];
        aux = diff.x*diff.x + diff.y*diff.y;
        if (IL_min>aux){ IL_min = aux;
                         IL     =   i;}

        // Superior Left
        diff = points[i] - cornerROI[0];
        aux = diff.x*diff.x + diff.y*diff.y;
        if (SL_min>aux){ SL_min = aux;
                         SL     =   i;}

        // Inferior Right
        diff = points[i] - cornerROI[2];
        aux = diff.x*diff.x + diff.y*diff.y;
        if (IR_min>aux){ IR_min = aux;
                         IR     =   i;}

        // Superior Right
        diff = points[i] - cornerROI[3];
        aux = diff.x*diff.x + diff.y*diff.y;
        if (SR_min>aux){ SR_min = aux;
                         SR     =   i;}
    }
}




void sortingPoints(vector<Point2f> &points, const cv::RotatedRect &ROI){

    if(points.size()<2){
        // No hace nada
    }
    else if(points.size()<4){
        std::sort(points.begin(),points.end(),[] (const Point2f& p1, const Point2f& p2){
                                                    float dist1, dist2;
                                                    distance(Point2f(0,0),p1,dist1);
                                                    distance(Point2f(0,0),p2,dist2);
                                                    return (dist1 < dist2);
                                                  });
    }
    else{
    /*
     *   IL ______________ IR
     *     |  -->x        |
     *     | |            |
     *     | v y          |
     *     |______________|
     *   SL                SR
     */
        // Copy data
        vector<Point2f> cpPts(points.size());
        vector<  int  > index(points.size());
        for(size_t i=0;i<size_t(index.size());++i)
            index[i] = int(i);

        //cout << "Estoy aqui!! (Copy data)" << endl;

        // Corners
        uint IL, IR, SL, SR;
        getCorners(points,IL,IR,SL,SR,ROI);

        //cout << "Estoy aqui!! (Corners)" << endl;

        // Traslate
        for(size_t i=0;i<size_t(index.size());++i)
            cpPts[i] = points[i] - points[IL];

        // ---------------------------
        Mat a = Mat::zeros( Size(300,300), CV_8UC3 );
        for(size_t i=0;i<size_t(index.size());++i)
            circle(a, cpPts[i] + Point2f(100,100), 2, Scalar(0,255,0), -1, 8, 0);
        // ---------------------------

        imshow( "Antes de rotar", a );                   // Show our image inside it.
        waitKey(1);


        // Calcular angulo
        float h = sqrt( cpPts[IR].x*cpPts[IR].x + cpPts[IR].y*cpPts[IR].y );
        float cos = cpPts[IR].x/h;
        float sen = cpPts[IR].y/h;


        // Rotate
        float x,y;
        for(size_t i=0;i<size_t(index.size());++i){
            x = cpPts[i].x;  y = cpPts[i].y;
            cpPts[i].x =   cos*x + sen*y;
            cpPts[i].y = - sen*x + cos*y;

            cout << "(" << cpPts[i].x << "," << cpPts[i].y << ")\t";
        }

        // ---------------------------
        Mat b = Mat::zeros( Size(300,300), CV_8UC3 );
        for(size_t i=0;i<size_t(index.size());++i)
            circle(b, cpPts[i] + Point2f(100,100), 2, Scalar(0,255,0), -1, 8, 0);
        // ---------------------------
        imshow( "Luego de rotar", b );                   // Show our image inside it.
        waitKey(1);


        //std::cin.get();



/*
        // Transform
        cv::Point2f * inputQuad = new cv::Point2f[4];
        cv::Point2f *outputQuad = new cv::Point2f[4];

        inputQuad[0] = points[IL]; inputQuad[1] = points[IR];
        inputQuad[3] = points[SL]; inputQuad[2] = points[SR];

        inputQuad[0] = cv::Point2f(0,0); inputQuad[1] = cv::Point2f(1,0);
        inputQuad[3] = cv::Point2f(0,1); inputQuad[2] = cv::Point2f(1,1);
 */
/*
        cv::Mat lambda( 5 , 4, CV_32FC1 );
        lambda = getPerspectiveTransform( inputQuad, outputQuad );

        cv::Mat dst;
        cv::warpPerspective( cv::Mat( points ), dst, lambda, dst.size());
 */

    }



    /*
    vector<Point2f> pointTras( point.size() );

    float cosAngle = float(cos(double(angle)*M_PI/180.0));
    float sinAngle = float(sin(double(angle)*M_PI/180.0));

    float min_x = 999999999.99f, max_x = -999999999.99f;
    float min_y = 999999999.99f, max_y = -999999999.99f;
    Point2f p;

    // Traslate and rotation
    for(size_t i=0; i<point.size(); ++i){
        p = point[i] - origin;

        p.x =   p.x*cosAngle + p.x*sinAngle;
        p.y = - p.y*sinAngle + p.y*cosAngle;

        if(min_x>p.x) min_x = p.x; if(max_x<p.x) max_x = p.x;
        if(min_y>p.y) min_y = p.y; if(max_y<p.y) max_y = p.y;

        // Update
        pointTras[i] = p;
    }
    */


}


bool DoesntRectangleContainPoint(RotatedRect &rectangle, Point2f &point) {
    //Get the corner points.
    Point2f corners[4];
    rectangle.points(corners);

    //Convert the point array to a vector.
    //https://stackoverflow.com/a/8777619/1997617
    Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    vector<Point2f> contour(corners, lastItemPointer);

    //Check if the point is within the rectangle.
    double indicator = pointPolygonTest(contour, point, false);
    return (indicator < 0);
}

void ellipsePurge(Mat &morphology, Mat &ellipses,
                  const std::vector<RotatedRect> &elipses,
                  vector<Point2f> &centers,
                  int width, int height){
    vector<int> ellipses_to_verify( elipses.size() );
    Point2f center;
    int counter_verify;
    float x_t0, y_t0;
    float x_t1, y_t1;
    float disX, disY;
    float sumx, sumy;

    int counter = 0;
    float holgura = 1;
    float distance;

    // Draw ellipse|
    ellipses = Mat::zeros( morphology.size(), CV_8UC3 );

    for( size_t i = 0; i< elipses.size()-1; i++ ){
        if( elipses[i].size.width > width/8 || elipses[i].size.height > height/6 ||
            elipses[i].size.width < 6       || elipses[i].size.height < 6        ||
            ellipses_to_verify[i] == -1){}
        else{
            x_t0 = elipses[i].center.x;
            y_t0 = elipses[i].center.y;

            counter_verify = 1;
            sumx = x_t0;
            sumy = y_t0;

            for( size_t j = i+1; j< elipses.size()-1; j++ ){
                x_t1 = float(elipses[j].center.x);
                y_t1 = float(elipses[j].center.y);

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
                center = Point2f(sumx/counter_verify, sumy/counter_verify);
                circle(ellipses, center, 2, Scalar(0,255,0), -1, 8, 0);
                ellipse( ellipses, elipses[i], Scalar(0,0,255), 2, 8 );

                // Add to ellipse list
                centers.push_back(center);
            }

        }
    }
}

void gridDetection(cv::Mat &frame     , cv::Mat  &binarized,
                   cv::Mat &morphology, cv::Mat  &ellipses ,
                   cv::Mat &result, cv::RotatedRect &minRec,
                   int &ellipseCount){

    int width  = frame.cols;
    int height = frame.rows;
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
    vector<Point2f> good_ellipses;
    ellipsePurge(morphology,ellipses,minEllipse,
                    good_ellipses,width, height);

    auto it = std::remove_if(good_ellipses.begin(),
                             good_ellipses.  end(),
                             [&minRec](Point2f &p){
                                return DoesntRectangleContainPoint(minRec,p);
                            });
    good_ellipses.erase(it,good_ellipses.end());

    //Point2f* corners;
    //minRec.points(corners);


/*
    Update ROI
    ----------
 */
    if(good_ellipses.size() > 16){
        minRec = cv::minAreaRect( cv::Mat(good_ellipses) );
        minRec.size.width  = minRec.size.width  + 10;
        minRec.size.height = minRec.size.height + 10;
    }
    else if(good_ellipses.size() > 10){
        minRec = cv::minAreaRect( cv::Mat(good_ellipses) );
        minRec.size.width  = minRec.size.width  + 50;
        minRec.size.height = minRec.size.height + 50;
    }else{
        minRec = cv::RotatedRect(cv::Point(frame.rows,         0),
                                 cv::Point(         0,         0),
                                 cv::Point(         0,frame.cols));
    }

    sortingPoints(good_ellipses,minRec);

/*
    Grid patron;
    Pts centers;
    for(size_t i=0;i<good_ellipses.size();++i)centers.push_back( Pt( good_ellipses[i] ) );

    mapping(centers,patron,uint(frame.rows),uint(frame.cols),5,4);

    good_ellipses.clear();
    for(size_t i =0; i<patron.size();++i){
        for(size_t j =0; j<patron[i].size();++j){
            if(patron[i][j].x > 0.0f){
                good_ellipses.push_back( cv::Point2f( patron[i][j].x,patron[i][j].y ) );
            }
        }
    }
*/
/*
    Result
    ------
 */
    frame.copyTo(result);
    for (size_t i = 0; i < good_ellipses.size(); ++i){
        circle(result, good_ellipses[i], 2, Scalar(0,255,0), -1, 8, 0); //-1 full circle
        putText(result,      std::to_string(i),
                              good_ellipses[i], // Coordinates
                       cv::FONT_HERSHEY_DUPLEX, // Font
                                           0.6, // Scale. 2.0 = 2x bigger
                           cv::Scalar(0,0,255), // BGR Color
                                            2); // Line Thickness (Optional)
    }

    // Save ellipse count
    ellipseCount = int(good_ellipses.size());
}
