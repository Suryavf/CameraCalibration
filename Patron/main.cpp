#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <limits>

typedef float T;
typedef unsigned char uchar;
typedef unsigned  int uint ;

uint LEN_X = 5;
uint LEN_Y = 4;

/*
 * Class Vect
 * =================================================
 */
class Vect{
public:

    // Parameters
    T x,y;

    // Constructors
    Vect(): x(0), y(0){}
    Vect(  T   &_x,   T   &_y): x(_x), y(_y){} 
    Vect( int  &_x,  int  &_y): x((T)_x), y((T)_y){} 
    Vect(uchar &_x, uchar &_y): x((T)_x), y((T)_y){} 

    void module(T &m);
    T    module();

    void norm(T &m);
    T    norm();

    void dot(const Vect& p, T &prod);
    T    dot(const Vect& p);

    // Operators
    Vect operator+(const Vect& p){
        T a = this->x+p.x;
        T b = this->y+p.y;
        return Vect(a,b);
    }
    Vect operator-(const Vect& p){
        T a = this->x - p.x;
        T b = this->y - p.y;
        return Vect(a,b);
    }
};

void Vect::module(float &m){
    m = x*x + y*y;
}
T    Vect::module(){
    return x*x + y*y;
}

void Vect::norm(T &m){
    m = std::sqrt(x*x + y*y);
}
T    Vect::norm(){
    return std::sqrt(x*x + y*y);
}

void Vect::dot(const Vect& p, T &prod){
    prod = this->x*p.x + this->y*p.y;
}
T    Vect::dot(const Vect& p){
    return this->x*p.x + this->y*p.y;
}


/*
 * Class Point
 * =================================================
 */
class Point : public Vect{
public:
    // Parameters
    bool check;
    T        d;

    // Constructors
    Point() : check(true) {x=0;y=0;}
    Point(  T   &_x,   T   &_y) : check(true) { x=   _x; y=   _y;} 
    Point( int  &_x,  int  &_y) : check(true) { x=(T)_x; y=(T)_y;} 
    Point(uchar &_x, uchar &_y) : check(true) { x=(T)_x; y=(T)_y;} 

    void distance(const Point& p, T &d);
    T    distance(const Point& p);
};

void Point::distance(const Point& p, T &d){
    T a = this->x - p.x;
    T b = this->y - p.y;

    d = std::sqrt( a*a + b*b );
}
T    Point::distance(const Point& p){
    T a = this->x - p.x;
    T b = this->y - p.y;
    
    return std::sqrt( a*a + b*b );
}

class Points{
public:
    Points(uint n){ data = std::vector< Point >(n);  }

protected:
    std::vector< Point > data;

private:
    bool sortCondition(const Point &a, const Point &b);

};


/*
 * Class Line
 * =================================================
 * 
 */
class Line{
private:
    T x_coef;
    T y_coef;
    T   bias;

public:
    Line() : x_coef(0),y_coef(0),bias(0){}
    Line(const Point &a, const Point &b) : x_coef(b.y - a.y),
                                           y_coef(a.x - b.x),
                                             bias(a.y*(b.x-a.x) + a.x*(a.y-b.y)){
        T module = sqrt(x_coef*x_coef + y_coef*y_coef);
        
        x_coef = x_coef/module;
        y_coef = y_coef/module;
        bias   =   bias/module;
    }

    void eval(const Point &p, T &v);
    T    eval(const Point &p);
};

void Line::eval(const Point &p, T &v){
    v = p.x*x_coef + p.y*y_coef +  bias;
}
T    Line::eval(const Point &p){
    return p.x*x_coef + p.y*y_coef +  bias;
}


/*
 *  Distance Line to Point
 *  ----------------------
 *           |L(p)|
 *  d(L,p) = ------
 *            ||w||
 */
void distance(Point &p, Line &l, T &d){
    if (p.check) d = abs( l.eval(p) ); // w = 1
    else         d = std::numeric_limits<T>::max();
}

bool sortCondition(const std::pair<int,bool> &a, const std::pair <int,bool> &b){
    return ( a.second && (a.first < b.first) );
}


bool sortDistance(const Point &a, const Point &b){
    return ( a.d < b.d );
}
bool sortX(const Point &a, const Point &b){
    return ( a.x < b.x );
}
bool sortY(const Point &a, const Point &b){
    return ( a.y < b.y );
}


int main( int argc, char** argv ) {
/*
 *  Load image
 *  ----------  
 */
    cv::Mat src;
    src = cv::imread("patron.png" , CV_LOAD_IMAGE_COLOR);
    int n_rowsSRC = src.rows;
    int n_colsSRC = src.cols;

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

    Point *centers;
    centers = new Point[circles.size()];

    std::vector<Point> pts(circles.size());

    T x,y;
    for( int i = 0; i < pts.size(); i++ ){
        pts[i] = Point(circles[i][0],circles[i][1]);
    }
/*
 *  Corners
 *  -------
 */ 
    T mod;
    T x_cols,y_rows;
    std::vector<T> a(pts.size()) , b(pts.size()),
                   c(pts.size()) , d(pts.size()) ;
    for( int i = 0; i < pts.size(); i++ ){
        x = pts[i].x;
        y = pts[i].y;
        
        x_cols = n_colsSRC - x;
        y_rows = n_rowsSRC - y;

        a[i] = x     *x      + y     *y     ;
        b[i] = x     *x      + y_rows*y_rows;
        c[i] = x_cols*x_cols + y     *y     ;
        d[i] = x_cols*x_cols + y_rows*y_rows;
    }

    auto a_corner = std::min_element( a.begin(), a.end() ) - a.begin();
    auto b_corner = std::min_element( b.begin(), b.end() ) - b.begin();
    auto c_corner = std::min_element( c.begin(), c.end() ) - c.begin();
    auto d_corner = std::min_element( d.begin(), d.end() ) - d.begin();

    pts[a_corner].check = false;
    pts[b_corner].check = false;
    pts[c_corner].check = false;
    pts[d_corner].check = false;

    // Line create
    Line L1 = Line(pts[a_corner],pts[c_corner]);
    Line L2 = Line(pts[a_corner],pts[b_corner]);
    
    std::vector< std::pair <T,uint> > dL1p(pts.size());
    std::vector< std::pair <T,uint> > dL2p(pts.size());

// -----------------------------------------------------------------------------------------------------------------------------------------------------
    std::vector<Point> ptsL1(pts);
    for(uint i = 0; i<pts.size(); ++i) distance(ptsL1[i], L1, ptsL1[i].d);
    std::sort(ptsL1.begin(),ptsL1.end(),sortDistance);

    T sumDL1p = 0;
    for(uint i = 0; i<LEN_X-2; ++i) sumDL1p += ptsL1[i].d; 

// -----------------------------------------------------------------------------------------------------------------------------------------------------
    std::vector<Point> ptsL2(pts);
    for(uint i = 0; i<ptsL2.size(); ++i) distance(ptsL2[i], L2, ptsL2[i].d);
    std::sort(ptsL2.begin(),ptsL2.end(),sortDistance);
    
    T sumDL2p = 0;
    for(uint i = 0; i<LEN_X-2; ++i) sumDL2p += ptsL2[i].d; 
    

// -----------------------------------------------------------------------------------------------------------------------------------------------------


/*
 *  Definir orientacion
 *  -------------------
 */ 
    uint n_rows, n_cols;
    if( sumDL1p<sumDL2p ){ n_rows = LEN_Y; n_cols = LEN_X;}
    else                 { n_rows = LEN_X; n_cols = LEN_Y;} 

    std::sort(ptsL1.begin(),ptsL1.begin()+n_cols-2,sortX); // Sort x
    std::sort(ptsL2.begin(),ptsL2.begin()+n_rows-2,sortY); // Sort y

/*
 *  Points Array
 *  ------------
 */ 
    std::vector< std::vector<Point> > patron(n_rows, std::vector<Point>(n_cols));
    patron[    0   ][    0   ] = pts[a_corner];
    patron[n_rows-1][    0   ] = pts[b_corner];
    patron[    0   ][n_cols-1] = pts[c_corner];
    patron[n_rows-1][n_cols-1] = pts[d_corner];

    uint id;
    for(uint i = 0; i<n_cols-2; ++i){
        ptsL1[i].check = false;
        patron[0][i+1] = ptsL1[i];
    }
    for(uint j = 0; j<n_rows-2; ++j){ 
        ptsL2[j].check = false;
        patron[j+1][0] = ptsL2[j];
    }



/*
 *  Two more
 *  --------
 */ 

    Line L3 = Line(pts[b_corner],pts[d_corner]);
    Line L4 = Line(pts[c_corner],pts[d_corner]);

    for(uint i = 0; i<pts.size(); ++i) distance(pts[i], L3, pts[i].d);
    std::sort(pts.begin(),pts.end(),sortDistance);
    std::sort(pts.begin(),pts.begin()+n_cols-2,sortX);
    
    for(uint i = 0; i<n_cols-2; ++i){
        pts[i].check = false;
        patron[n_rows-1][i+1] = pts[i];
    }

    for(uint i = 0; i<pts.size(); ++i) distance(pts[i], L4, pts[i].d);
    std::sort(pts.begin(),pts.end(),sortDistance);
    std::sort(pts.begin(),pts.begin()+n_rows-2,sortY);
    
    for(uint i = 0; i<n_rows-2; ++i){
        pts[i].check = false;
        patron[i+1][n_cols-1] = pts[i];
    }
    
/*
 *  Main Loop
 *  ---------
 */ 

    Line L;
    for(uint i = 1; i<n_rows-1; ++i){
        L = Line(patron[i][0],patron[i][n_cols-1]);

        for(uint j = 0; j<pts.size(); ++j) distance(pts[j], L, pts[j].d);
        std::sort(pts.begin(),pts.end(),sortDistance);
        std::sort(pts.begin(),pts.begin()+n_cols-2,sortX);

        for(uint j = 0; j<n_cols-2; ++j){
            pts[j].check = false;
            patron[i][j+1] = pts[j];
        }
    }
    

    std::cout << "\nPatron!:\n========" << std::endl;
    for(uint i = 0; i<n_rows; ++i){
        for(uint j = 0; j<n_cols; ++j){ 
            std::cout << "(" << patron[i][j].x << "," << patron[i][j].y << ")\t";
        }
        std::cout << std::endl;
    }



// -----------------------------------------------------------------------------------------------------

    /*
    std::cout << "Cucaracha:";// 
    auto laura = std::min_element( a.begin(), a.end() ) - a.begin();
    std::cout <<  laura << std::endl;
    std::cout << laura << std::endl;//
    */
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
    std::vector< std::pair <int,bool> > test;
    test.push_back( std::make_pair(5,false) );
    test.push_back( std::make_pair(3,true) );
    test.push_back( std::make_pair(2,true) );
    test.push_back( std::make_pair(4,true) );
    test.push_back( std::make_pair(0,false) );


    std::sort(test.begin(),test.end(),sortCondition);

    for(int i =0; i<test.size();++i){
        std::cout << "[" << test[i].first << "," << test[i].second << "]" << std::endl;
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