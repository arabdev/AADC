// example: class constructor
#include <iostream>
#include "DetectedObject.h"
using namespace std;

DetectedObject::DetectedObject (float a, float b,float c,float d) {
  xmin = a;
  ymin = b;
  xmax = c;
  ymax = d;
}


float DetectedObject::get_xmin()
{
    return xmin;
}

float DetectedObject::get_ymin()
{
    return ymin;
}

float DetectedObject::get_xmax()
{
    return xmax;
}

float DetectedObject::get_ymax()
{
    return ymax;
}
