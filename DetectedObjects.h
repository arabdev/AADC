#ifndef DetectedObject_H
#define DetectedObject_H

//CREATED BY GIACOMO BARTOLI

class DetectedObject
{
  float xmin, ymin, xmax, ymax;

public:
  DetectedObject(float a, float b,float c,float d);

  float get_xmin();
  float get_ymin();
  float get_xmax();
  float get_ymax();

};

#endif
