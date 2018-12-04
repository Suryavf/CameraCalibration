#include "structures.h"

/*
 * Class Vect
 * =================================================
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