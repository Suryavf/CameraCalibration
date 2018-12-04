#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <limits>

typedef float T;
typedef unsigned char uchar;
typedef unsigned  int uint ;

uint LEN_X = 5;
uint LEN_Y = 4;
T    MAX_T = std::numeric_limits<T>::max();

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

    // Basic functions
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


/*
 *  Basic functions
 *  ---------------
 */
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
void distance(const Point &p,  Line &l, T &d){
    if (p.check) d = abs( l.eval(p) ); // w = 1
    else         d = std::numeric_limits<T>::max();
}


/*
 *  Sort functions
 *  --------------
 */
bool sortDistance(const Point &a, const Point &b){
    return ( a.d < b.d );
}
bool sortX(const Point &a, const Point &b){
    return ( a.x < b.x );
}
bool sortY(const Point &a, const Point &b){
    return ( a.y < b.y );
}


/*
 *  Get corners
 *  -----------
 */
void getCorners(const std::vector<Point>  &pts,
                            uint &IL, uint &IR,
                            uint &SL, uint &SR,
                            const uint &n_colsSRC,
                            const uint &n_rowsSRC){
    T x_cols,y_rows,aux;
    T x,y;
    T IL_min = MAX_T, SL_min = MAX_T, 
      IR_min = MAX_T, SR_min = MAX_T;
    for( uint i = 0; i < pts.size(); i++ ){
        x = pts[i].x; x_cols = n_colsSRC - x;
        y = pts[i].y; y_rows = n_rowsSRC - y;

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



void initPatron(const std::vector<Point>  &pts,
                std::vector< std::vector<Point> > &patron,
                Line &L1, Line &L2,
                const uint &IL, const uint &IR,
                const uint &SL, const uint &SR,
                uint &n_rows, uint &n_cols){

/*
 *  Definir orientacion
 *  -------------------
 */ 
    std::vector<Point> ptsL1(pts); 
    std::vector<Point> ptsL2(pts); 
    T sumDL1p = 0; T sumDL2p = 0;

//- Line create: L1
    for(uint i = 0; i<pts.size(); ++i) 
        distance(ptsL1[i], L1, ptsL1[i].d);
    std::sort(ptsL1.begin(),ptsL1.end(),sortDistance);

    /// Calculte distance sum
    for(uint i = 0; i<LEN_X-2; ++i) 
        sumDL1p += ptsL1[i].d; 
    
//- Line create: L2
    for(uint i = 0; i<ptsL2.size(); ++i) 
        distance(ptsL2[i], L2, ptsL2[i].d);
    std::sort(ptsL2.begin(),ptsL2.end(),sortDistance);

    /// Calculte distance sum
    for(uint i = 0; i<LEN_X-2; ++i) 
        sumDL2p += ptsL2[i].d; 

//- Define direction
    if( sumDL1p<sumDL2p ){ n_rows = LEN_Y; n_cols = LEN_X;}
    else                 { n_rows = LEN_X; n_cols = LEN_Y;} 

    std::sort(ptsL1.begin(),ptsL1.begin()+n_cols-2,sortX); // Sort x
    std::sort(ptsL2.begin(),ptsL2.begin()+n_rows-2,sortY); // Sort y

/*
 *  Points Array
 *  ------------
 */ 
    patron = std::vector< std::vector<Point> >(n_rows, std::vector<Point>(n_cols));
    patron[    0   ][    0   ] = pts[ IL ];
    patron[n_rows-1][    0   ] = pts[ SL ];
    patron[    0   ][n_cols-1] = pts[ IR ];
    patron[n_rows-1][n_cols-1] = pts[ SR ];

    uint id;
    for(uint i = 0; i<n_cols-2; ++i){
        ptsL1[i].check = false;
        patron[0][i+1] = ptsL1[i];
    }
    for(uint j = 0; j<n_rows-2; ++j){ 
        ptsL2[j].check = false;
        patron[j+1][0] = ptsL2[j];
    }
}


void addPatron(std::vector<Point>  &pts,
               std::vector< std::vector<Point> > &patron,
               Line &L,
               const uint &n_rows, const uint &n_cols,
               const uint &position,
               const bool &horz = true){
    uint i;

//- Line horizontal
    if(horz){
        // Calculate distance
        for(i = 0; i<pts.size(); ++i) 
            distance(pts[i], L, pts[i].d);
        
        // Sort
        std::sort(pts.begin(),pts.end(),sortDistance);
        std::sort(pts.begin(),pts.begin()+n_cols-2,sortX);
        
        for(i = 0; i<n_cols-2; ++i){
            pts[i].check = false;
            patron[position][i+1] = pts[i];
        }
    }

//- Line Vertical
    else{
        // Calculate distance
        for(i = 0; i<pts.size(); ++i) 
            distance(pts[i], L, pts[i].d);

        // Sort
        std::sort(pts.begin(),pts.end(),sortDistance);
        std::sort(pts.begin(),pts.begin()+n_rows-2,sortY);

        for(i = 0; i<n_rows-2; ++i){
            pts[i].check = false;
            patron[i+1][position] = pts[i];
        }
    }
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

    std::vector<Point> pts(circles.size());

    T x,y;
    for( int i = 0; i < pts.size(); i++ ){
        pts[i] = Point(circles[i][0],circles[i][1]);
    }
/*
 *  Corners
 *  -------
 */ 
    uint IL, IR, SL, SR;
    getCorners(pts,IL,IR,SL,SR, n_colsSRC, n_rowsSRC);
    
    pts[ IL ].check = false; pts[ IR ].check = false;
    pts[ SL ].check = false; pts[ SR ].check = false;
    
/*
 *  Definir orientacion
 *  -------------------
 */ 
    uint n_rows, n_cols;
    std::vector< std::vector<Point> > patron;

    Line L1 = Line(pts[ IL ],pts[ IR ]);
    Line L2 = Line(pts[ IL ],pts[ SL ]);

    initPatron(pts,patron,L1,L2,IL,IR,SL,SR,n_rows,n_cols);

/*
 *  Two more
 *  --------
 */ 

    Line L3 = Line(pts[SL],pts[SR]);
    Line L4 = Line(pts[IR],pts[SR]);

    addPatron(pts,patron,L3,n_rows, n_cols,n_rows-1,true );
    addPatron(pts,patron,L4,n_rows, n_cols,n_cols-1,false);
/*
 *  Main Loop
 *  ---------
 */ 

    Line L;
    for(uint i = 1; i<n_rows-1; ++i){
        L = Line(patron[i][0],patron[i][n_cols-1]);
        addPatron(pts,patron,L,n_rows, n_cols,i);
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