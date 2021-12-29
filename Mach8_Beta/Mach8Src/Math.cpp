#include "vex.h"

double Delta(double v1, double v2){
  double delta = v2-v1;
  
  return delta;
}

double distanceBetween(double delX, double delY) {
  double distance = sqrt(pow(delX, 2)+pow(delY, 2));

  return distance;
}

double toRadians(double angle) {
  return angle * (M_PI/180);
}

double toDegrees(double angle) {
  return angle * (180/M_PI);
} 