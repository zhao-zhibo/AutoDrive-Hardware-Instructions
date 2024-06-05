#pragma once
#include "navislam_global.h"

namespace navicore
{

#define  POSITIVE_INFINITY   99999999999

class  Envelope
{

public:
     Envelope(void);
     Envelope(double x, double y, double width, double height);
    ~Envelope(void);
     void makeFromCenter(double centerX,double centerY,double width,double height);
     void extendCenter(double w, double h);
     void setBounds(double x_new, double y_new, double width_new,double height_new);
     bool isEmpty();
     void add(Envelope rect);
     bool contains(double x, double y);
     void add(double px, double py);
     bool equals(Envelope r);
     void operator = (Envelope env);
    double m_fX;
    double m_fY;
    double m_fWidth;
    double m_fHeight;
    bool intersect(Envelope env,Envelope& intersect);

};

}

