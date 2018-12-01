#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
 
typedef float T;
typedef unsigned char uchar;
typedef unsigned  int uint ;

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
    // Constructors
    Point(){x=0;y=0;}
    Point(  T   &_x,   T   &_y){ x=   _x; y=   _y;} 
    Point( int  &_x,  int  &_y){ x=(T)_x; y=(T)_y;} 
    Point(uchar &_x, uchar &_y){ x=(T)_x; y=(T)_y;} 


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
    d = abs( l.eval(p) ); // w = 1
}




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

    Point *centers;
    centers = new Point[circles.size()];
    T x,y;
    for( int i = 0; i < circles.size(); i++ ){
        centers[i] = Point(circles[i][0],circles[i][1]);
    }
/*
 *  Corners
 *  -------
 */ 
    T mod;
    std::vector<T> a;
    for( int i = 0; i < circles.size(); i++ ){
        
        
        
        mod = centers[i].norm();
        std::cout << "P: (" << centers[i].x << "," << centers[i].y << ") -> norm = " << mod << std::endl;
    }

    
    for( size_t i = 0; i < circles.size(); i++ ){
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        // circle center
        circle( src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }
    



/*
 *  Display image
 *  -------------  
 */
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", src );

    cv::waitKey(0);
    return 0;
}