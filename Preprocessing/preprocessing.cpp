#include "preprocessing.h"


void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat){
    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);

    // rows -> height -> y
    int nRows = inputMat.rows;
    // cols -> width -> x
    int nCols = inputMat.cols;

    // create the integral image
    cv::Mat sumMat;
    cv::integral(inputMat, sumMat);

    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);

    int S = MAX(nRows, nCols)/32;
    double T = 0.15;

    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;

    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;

    for( int i = 0; i < nRows; ++i){
        y1 = i-s2;
        y2 = i+s2;

        if (y1 < 0){
            y1 = 0;
        }
        if (y2 >= nRows) {
            y2 = nRows-1;
        }

        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);

        for ( int j = 0; j < nCols; ++j){
            // set the SxS region
            x1 = j-s2;
            x2 = j+s2;

            if (x1 < 0) {
                x1 = 0;
            }
            if (x2 >= nCols) {
                x2 = nCols-1;
            }

            count = (x2-x1)*(y2-y1);

            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];

            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 255;
            else
                p_outputMat[j] = 0;
        }
    }
}



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
                circle(ellipses, center, 2,cv::Scalar(35,255,75), -1, 8, 0);
                ellipse( ellipses, elipses[i], Scalar(253,35,255), 2, 8 );

                // Add to ellipse list
                centers.push_back(center);
            }

        }
    }
}
vector<Point2f> get_intermediate_sorted_points(Point p, vector<Point2f> intermediate_vectors)
{
    vector<Point2f> result;
    if(intermediate_vectors.size() == 1)
    {
        result.push_back(intermediate_vectors[0]);
        return result;
    }

    if(intermediate_vectors.size() == 3 )
    {
        float d1 = sqrt(pow(p.x - intermediate_vectors[0].x,2) + pow(p.y - intermediate_vectors[0].y,2) );
        float d2 = sqrt(pow(p.x - intermediate_vectors[1].x,2) + pow(p.y - intermediate_vectors[1].y,2) );
        float d3 = sqrt(pow(p.x - intermediate_vectors[2].x,2) + pow(p.y - intermediate_vectors[2].y,2) );

        if(d1 < d2 && d1 < d3)
        {
            result.push_back(intermediate_vectors[0]);

            if(d2 < d3)
            {
                result.push_back(intermediate_vectors[1]);
                result.push_back(intermediate_vectors[2]);
            }

            else
            {
                result.push_back(intermediate_vectors[2]);
                result.push_back(intermediate_vectors[1]);
            }
        }

        if(d2 < d1 && d2 < d3)
        {
            result.push_back(intermediate_vectors[1]);

            if(d1 < d3)
            {
                result.push_back(intermediate_vectors[0]);
                result.push_back(intermediate_vectors[2]);
            }

            else
            {
                result.push_back(intermediate_vectors[2]);
                result.push_back(intermediate_vectors[0]);
            }
        }

        if(d3 < d1 && d3< d2)
        {
            result.push_back(intermediate_vectors[2]);

            if(d1 < d2)
            {
                result.push_back(intermediate_vectors[0]);
                result.push_back(intermediate_vectors[1]);
            }

            else
            {
                result.push_back(intermediate_vectors[1]);
                result.push_back(intermediate_vectors[0]);
            }
        }

    }

    if(intermediate_vectors.size() == 2)
    {
        float d1 = sqrt(pow(p.x - intermediate_vectors[0].x,2) + pow(p.y - intermediate_vectors[0].y,2) );
        float d2 = sqrt(pow(p.x - intermediate_vectors[1].x,2) + pow(p.y - intermediate_vectors[1].y,2) );

        if(d1 < d2)
        {
            result.push_back(intermediate_vectors[0]);
            result.push_back(intermediate_vectors[1]);
        }

        else
        {
            result.push_back(intermediate_vectors[1]);
            result.push_back(intermediate_vectors[0]);
        }

    }

    return result;

}
vector<Point2f> get_intermediate_points(Point p1, Point p2, vector<Point2f> good_ellipses)
{
    vector<Point2f> intermediate_vectors;
    float m = (p2.y - p1.y)/(p2.x - p1.x + 0.0001);
    float y_eq, x_eq;
    int holgura = 5;

    for(uint i=0; i<good_ellipses.size(); i++)
    {
        if( !(
                (abs(good_ellipses[i].x-p1.x)<1 && abs(good_ellipses[i].y - p1.y)<1)
                || (abs(good_ellipses[i].x - p2.x)<1 && abs(good_ellipses[i].y - p2.y)<1)
                    ))
        {
            if(abs(good_ellipses[i].x - p1.x) > abs(good_ellipses[i].y - p1.y))
            {
                y_eq = m*(good_ellipses[i].x - p1.x) + p1.y;

                if(abs(y_eq - good_ellipses[i].y) <= holgura)
                {
                    intermediate_vectors.push_back(good_ellipses[i]);
                }

            }
            else
            {
                x_eq = (good_ellipses[i].y - p1.y)/m + p1.x;

                if(abs(x_eq - good_ellipses[i].x) <= holgura)
                {
                    intermediate_vectors.push_back(good_ellipses[i]);
                }

            }
        }
    }
    return intermediate_vectors;
}
vector<Point2f> ellipses_order20(vector<Point2f> good_ellipses)
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
    //cout<<sorted_ellipses.size()<<endl;
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
        //cout<<"FORMA AMPLIA"<<endl;
    }
    else
    {
        vector<Point2f> sorted_ellipses2;
        sorted_ellipses2.push_back(sorted_ellipses[3]);
        sorted_ellipses2.push_back(sorted_ellipses[0]);
        sorted_ellipses2.push_back(sorted_ellipses[2]);
        sorted_ellipses2.push_back(sorted_ellipses[1]);
        //cout<<"FORMA ALTA"<<endl;
        sorted_ellipses = sorted_ellipses2;

    }

    //============================================FUNCTION, HOW?============================================================
    vector<Point2f> sorted_ellipses_final(20);

    //Adding first row 0 to 4
    vector<Point2f> intermediate_vectors = get_intermediate_points(sorted_ellipses[0], sorted_ellipses[1], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[0], intermediate_vectors); // consider the most near to sorted

    sorted_ellipses_final[0] = sorted_ellipses[0];

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+1] = intermediate_vectors[i];
    }
    sorted_ellipses_final[4] = sorted_ellipses[1];




    //Adding first row 5, 10, 15
    intermediate_vectors = get_intermediate_points(sorted_ellipses[0], sorted_ellipses[2], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[0], intermediate_vectors); // consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[(i+1)*5] = intermediate_vectors[i];
    }
    sorted_ellipses_final[15] = sorted_ellipses[2];


    //Adding first row 16 to 19
    intermediate_vectors = get_intermediate_points(sorted_ellipses[2], sorted_ellipses[3], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[2], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+16] = intermediate_vectors[i];
    }
    sorted_ellipses_final[19] = sorted_ellipses[3];



    //Adding first row 9 , 14
    intermediate_vectors = get_intermediate_points(sorted_ellipses[1], sorted_ellipses[3], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[1], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[9+(i*5)] = intermediate_vectors[i];
    }

    //Adding first row 6 to 8
    intermediate_vectors = get_intermediate_points(sorted_ellipses_final[5], sorted_ellipses_final[9], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses_final[5], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+6] = intermediate_vectors[i];
    }

    //Adding first row 11 to 13
    intermediate_vectors = get_intermediate_points(sorted_ellipses_final[10], sorted_ellipses_final[14], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses_final[10], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+11] = intermediate_vectors[i];
    }





    //=========================================================================================================================

    return sorted_ellipses_final;
}


// ================================= AUXILIO ==================================================================================

vector<Point2f> ellipses_order12(vector<Point2f> good_ellipses)
{
    vector<Point2f> convex_hull;
    vector<Point2f> sorted_ellipses;


    convexHull(good_ellipses, convex_hull, true); // Get corners
    convex_hull.push_back(convex_hull[0]);  // To ensure the square, searching the change to add the last corner
    convex_hull.push_back(convex_hull[1]);

    float x_s = convex_hull[0].x;
    float y_s = convex_hull[0].y;
    float m = (convex_hull[1].y - y_s)/(convex_hull[1].x - x_s + 0.0001); //Pendiente
    float holgura = 5; // In order to say the element is inside the line (recta)
    float y_eq = 0; // Equation with respect y
    float x_eq = 0; // Equation with respect x

    for(uint i=1; i<convex_hull.size(); i++)
    {
        if(abs(convex_hull[i].x - x_s) > abs(convex_hull[i].y - y_s)) // X mayor -> calculate the normal eq in function to y
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

    //cout<<sorted_ellipses.size()<<endl;
    x_s = sorted_ellipses[0].x;
    y_s = sorted_ellipses[0].y;
    m = (sorted_ellipses[1].y - y_s)/(sorted_ellipses[1].x - x_s + 0.0001);

    int count_dist1 = 0;
    int count_dist2 = 0;

    // Count the number of points from 0 to 1 position
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

    // Count the number of points from 1 to 2 position
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

    // Sort the corners
    if(count_dist1 < count_dist2)
    {
        Point p = sorted_ellipses[2];
        sorted_ellipses[2] = sorted_ellipses[3];
        sorted_ellipses[3] = p;
        //cout<<"FORMA AMPLIA"<<endl;
    }
    else
    {
        vector<Point2f> sorted_ellipses2;
        sorted_ellipses2.push_back(sorted_ellipses[3]);
        sorted_ellipses2.push_back(sorted_ellipses[0]);
        sorted_ellipses2.push_back(sorted_ellipses[2]);
        sorted_ellipses2.push_back(sorted_ellipses[1]);
        //cout<<"FORMA ALTA"<<endl;
        sorted_ellipses = sorted_ellipses2;

    }

    //============================================FUNCTION, HOW?============================================================
    vector<Point2f> sorted_ellipses_final(12);

    //Adding first row 0 to 4
    vector<Point2f> intermediate_vectors = get_intermediate_points(sorted_ellipses[0], sorted_ellipses[1], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[0], intermediate_vectors); // consider the most near to sorted

    sorted_ellipses_final[0] = sorted_ellipses[0];

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+1] = intermediate_vectors[i];
    }
    sorted_ellipses_final[3] = sorted_ellipses[1];




    //Adding first row 4
    intermediate_vectors = get_intermediate_points(sorted_ellipses[0], sorted_ellipses[2], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[0], intermediate_vectors); // consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[4] = intermediate_vectors[0];
    }
    sorted_ellipses_final[8] = sorted_ellipses[2];


    //Adding first row 9, 10
    intermediate_vectors = get_intermediate_points(sorted_ellipses[2], sorted_ellipses[3], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[2], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+9] = intermediate_vectors[i];
    }
    sorted_ellipses_final[11] = sorted_ellipses[3];



    //Adding first row 7
    intermediate_vectors = get_intermediate_points(sorted_ellipses[1], sorted_ellipses[3], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses[1], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[7] = intermediate_vectors[0];
    }

    //Adding first row 5, 6
    intermediate_vectors = get_intermediate_points(sorted_ellipses_final[4], sorted_ellipses_final[7], good_ellipses);
    intermediate_vectors  = get_intermediate_sorted_points(sorted_ellipses_final[4], intermediate_vectors); //consider the most near to sorted

    for (uint i=0; i<intermediate_vectors.size(); i++)
    {
        sorted_ellipses_final[i+5] = intermediate_vectors[i];
    }



    //=========================================================================================================================

    return sorted_ellipses_final;
}





// ============================================================================================================================











void gridDetection(cv::Mat &frame     , cv::Mat  &binarized,
                   cv::Mat &morphology, cv::Mat  &ellipses ,
                   cv::Mat &result, cv::RotatedRect &minRec,
                   vector<Point2f> &good_ellipses,
                   int &ellipseCount){

    int width  = frame.cols;
    int height = frame.rows;
    vector<Vec4i> hierarchy;
    good_ellipses.clear();



/*
    Binarized Image
    ---------------
 */
    cv::Mat gray;
    cvtColor(frame, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);
    //binarized = cv::Mat(gray);
    //thresholdIntegral(gray, binarized);

    adaptiveThreshold(gray, binarized, 255, 0, THRESH_BINARY,11,3);
    //adaptiveThreshold(gray, binarized, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,11,3);

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
    //vector<Point2f> good_ellipses;
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
        minRec.size.width  = minRec.size.width  + 100;
        minRec.size.height = minRec.size.height + 100;
    }
    else if(good_ellipses.size() > 10){
        minRec = cv::minAreaRect( cv::Mat(good_ellipses) );
        minRec.size.width  = minRec.size.width + 200;
        minRec.size.height = minRec.size.height+ 200;
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
        sorted_ellipses = ellipses_order20(good_ellipses);

    if(good_ellipses.size() == 12)
        sorted_ellipses = ellipses_order12(good_ellipses);


    frame.copyTo(result);
    for (size_t i = 0; i < sorted_ellipses.size(); ++i){
        circle(result, sorted_ellipses[i], 2, Scalar(253,35,255), -1, 8, 0); //-1 full circle
        putText(result,      std::to_string(i+1),
                              sorted_ellipses[i], // Coordinates
                       cv::FONT_HERSHEY_DUPLEX, // Font
                                          0.65, // Scale. 2.0 = 2x bigger
                         cv::Scalar(35,255,75), // BGR Color
                                            1); // Line Thickness (Optional)
    }

    // Save ellipse count
    ellipseCount = int(good_ellipses.size());
    //Point2f pt1 = Point2f(minRec.center.x - minRec.size.width/2, minRec.center.y - minRec.size.height/2);
    //Point2f pt2 = Point2f(minRec.center.x + minRec.size.width/2, minRec.center.y + minRec.size.height/2);
    //rectangle(result, pt1, pt2, Scalar(255, 0, 0), 1, 8, 0);
    //imshow("RESULT", result);
    //waitKey(1);
}

void calcBoardCornerPositions(cv::Size &boardSize, float squareSize, vector<Point3f>& corners){
    corners.clear();

    for( int i = boardSize.height - 1; i > -1; --i )
        for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
}

double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
                                 const vector<vector<Point2f> >& imagePoints,
                                 const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs,
                                 vector<float>& perViewErrors){
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i ){

        projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);

        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = float(std::sqrt(err*err/n));
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
