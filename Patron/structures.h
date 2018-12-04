#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <limits>

#ifndef STRUCTURES_H
#define STRUCTURES_H

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



/*
 * Class Line
 * =================================================
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


/*
 * Distance
 * =================================================
 */
void distance(Point &p, Line &l, T &d);

#endif