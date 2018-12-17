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
                    const uint &cols,
                    const uint &rows){
    float x_cols,y_rows,aux;
    float x,y;
    float MAX_T = std::numeric_limits<T>::max();
    float IL_min = MAX_T, SL_min = MAX_T,
          IR_min = MAX_T, SR_min = MAX_T;
    for( uint i = 0; i < points.size(); i++ ){
        x = points[i].x; x_cols = cols - x;
        y = points[i].y; y_rows = rows - y;

        // Inferior Left
        aux = x*x + y*y;
        if (IL_min>aux){ IL_min = aux;
                         IL     =   i;}

        // Superior Left
        aux = x*x + y_rows*y_rows;
        if (SL_min>aux){ SL_min = aux;
                         SL     =   i;}

        // Inferior Right
        aux = x_cols*x_cols + y*y;
        if (IR_min>aux){ IR_min = aux;
                         IR     =   i;}

        // Superior Right
        aux = x_cols*x_cols + y_rows*y_rows;
        if (SR_min>aux){ SR_min = aux;
                         SR     =   i;}
    }
}




void sortingPoints(vector<Point2f> &points, const uint &cols, const uint &rows){

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
        uint IL, IR, SL, SR;
        getCorners(points,IL,IR,SL,SR,cols,rows);

        // Transform
        cv::Point2f * inputQuad = new cv::Point2f[4];
        cv::Point2f *outputQuad = new cv::Point2f[4];

        inputQuad[0] = points[IL]; inputQuad[1] = points[IR];
        inputQuad[3] = points[SL]; inputQuad[2] = points[SR];

        inputQuad[0] = cv::Point2f(0,0); inputQuad[1] = cv::Point2f(1,0);
        inputQuad[3] = cv::Point2f(0,1); inputQuad[2] = cv::Point2f(1,1);
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


vector<Point2f> ellipses_order(vector<Point2f> good_ellipses)
{
    vector<Point2f> convex_hull;
    vector<Point2f> sorted_ellipses;


    convexHull(good_ellipses, convex_hull, true);
    convex_hull.push_back(convex_hull[0]);
    convex_hull.push_back(convex_hull[1]);

    float x_s = convex_hull[0].x;
    float y_s = convex_hull[0].y;
    float m = (convex_hull[1].y - y_s)/(convex_hull[1].x - x_s + 0.0001);
    float holgura = 5;
    float y_eq = 0;
    float x_eq = 0;

    for(uint i=1; i<convex_hull.size(); i++)
    {
        if(abs(convex_hull[i].x - x_s) > abs(convex_hull[i].y - y_s))
        {
            y_eq = m*(convex_hull[i].x - x_s) + y_s;

            if(abs(y_eq - convex_hull[i].y) >= holgura)
            {
                sorted_ellipses.push_back(convex_hull[i-1]);
                x_s = convex_hull[i-1].x;
                y_s = convex_hull[i-1].y;
                m = (convex_hull[i].y - y_s)/(convex_hull[i].x - x_s + 0.0001);
            }

        }
        else
        {
            x_eq = (convex_hull[i].y - y_s)/m + x_s;

            if(abs(x_eq - convex_hull[i].x) >= holgura)
            {
                sorted_ellipses.push_back(convex_hull[i-1]);
                x_s = convex_hull[i-1].x;
                y_s = convex_hull[i-1].y;
                m = (convex_hull[i].y - y_s)/(convex_hull[i].x - x_s + 0.0001);
            }

        }


    }
    cout<<sorted_ellipses.size()<<endl;
    x_s = sorted_ellipses[0].x;
    y_s = sorted_ellipses[0].y;
    m = (sorted_ellipses[1].y - y_s)/(sorted_ellipses[1].x - x_s + 0.0001);

    int count_dist1 = 0;
    int count_dist2 = 0;

    for(uint i=0; i<good_ellipses.size(); i++)
    {
        if(abs(good_ellipses[i].x - x_s) > abs(good_ellipses[i].y - y_s))
        {
            y_eq = m*(good_ellipses[i].x - x_s) + y_s;

            if(abs(y_eq - good_ellipses[i].y) >= holgura)
            {
                count_dist1++;
            }

        }
        else
        {
            x_eq = (good_ellipses[i].y - y_s)/m + x_s;

            if(abs(x_eq - good_ellipses[i].x) >= holgura)
            {
                count_dist1++;
            }

        }
    }




    x_s = sorted_ellipses[1].x;
    y_s = sorted_ellipses[1].y;
    m = (sorted_ellipses[2].y - y_s)/(sorted_ellipses[2].x - x_s + 0.0001);

    for(uint i=0; i<good_ellipses.size(); i++)
    {
        if(abs(good_ellipses[i].x - x_s) > abs(good_ellipses[i].y - y_s))
        {
            y_eq = m*(good_ellipses[i].x - x_s) + y_s;

            if(abs(y_eq - good_ellipses[i].y) >= holgura)
            {
                count_dist2++;
            }

        }
        else
        {
            x_eq = (good_ellipses[i].y - y_s)/m + x_s;

            if(abs(x_eq - good_ellipses[i].x) >= holgura)
            {
                count_dist2++;
            }

        }
    }


    if(count_dist1 < count_dist2)
    {
        Point p = sorted_ellipses[2];
        sorted_ellipses[2] = sorted_ellipses[3];
        sorted_ellipses[3] = p;
        cout<<"FORMA AMPLIA"<<endl;
    }
    else
    {
        vector<Point2f> sorted_ellipses2;
        sorted_ellipses2.push_back(sorted_ellipses[3]);
        sorted_ellipses2.push_back(sorted_ellipses[0]);
        sorted_ellipses2.push_back(sorted_ellipses[2]);
        sorted_ellipses2.push_back(sorted_ellipses[1]);
        cout<<"FORMA ALTA"<<endl;
        sorted_ellipses = sorted_ellipses2;

    }



    return sorted_ellipses;
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
        minRec.size.width = minRec.size.width + 100;
        minRec.size.height = minRec.size.height + 100;
    }
    else if(good_ellipses.size() > 10){
        minRec = cv::minAreaRect( cv::Mat(good_ellipses) );
        minRec.size.width = minRec.size.width + 200;
        minRec.size.height = minRec.size.height + 200;
    }else
    {
        minRec = cv::RotatedRect(cv::Point(frame.cols,         0),
                                 cv::Point(         0,         0),
                                 cv::Point(         0,frame.rows));
    }

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
    vector<Point2f> sorted_ellipses = good_ellipses;

    if(good_ellipses.size() == 20)
        sorted_ellipses = ellipses_order(good_ellipses);


    frame.copyTo(result);
    for (size_t i = 0; i < sorted_ellipses.size(); ++i){
        circle(result, sorted_ellipses[i], 2, Scalar(0,255,0), -1, 8, 0); //-1 full circle
        putText(result,      std::to_string(i),
                              sorted_ellipses[i], // Coordinates
                       cv::FONT_HERSHEY_DUPLEX, // Font
                                           0.9, // Scale. 2.0 = 2x bigger
                           cv::Scalar(0,0,255), // BGR Color
                                            2); // Line Thickness (Optional)
    }

    // Save ellipse count
    ellipseCount = int(good_ellipses.size());
    Point2f pt1 = Point2f(minRec.center.x - minRec.size.width/2, minRec.center.y - minRec.size.height/2);
    Point2f pt2 = Point2f(minRec.center.x + minRec.size.width/2, minRec.center.y + minRec.size.height/2);
    rectangle(result, pt1, pt2, Scalar(255, 0, 0), 1, 8, 0);
    imshow("RESULT", result);
    waitKey(1);
}


