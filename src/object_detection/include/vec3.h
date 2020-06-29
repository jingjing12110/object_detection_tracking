#ifndef VEC3_H_
#define VEC3_H_

#include <math.h>
#include <cmath>
#include <iostream>

class Vec3 {
  public:
    float x;
    float y;
    float z;
    float r;
    float theta;
    float theta_horizontal;
    float label;

    Vec3() : x(0), y(0), z(0), r(0), theta(0), label(-1), theta_horizontal(0) {};

    Vec3(float a, float b, float c) : x(a), y(b), z(c) {
      r = sqrt(x*x + y*y);
      theta = atan2(z, r) * 180 / 3.1415926;
      label = -1;
      theta_horizontal = atan2(y, x) - 3.14159265/2;
    }

    float distance(Vec3 v1) const {
      float x_new = this->x - v1.x;
      float y_new = this->y - v1.y;
      float z_new = this->z - v1.z;
      return sqrt(x_new * x_new + y_new * y_new + z_new * z_new);
    }

    float distance_squared(const float * v1) const {
      float x_new = this->x - v1[0];
      float y_new = this->y - v1[1];
      float z_new = this->z - v1[2];
      return x_new * x_new + y_new * y_new + z_new * z_new;
    }
};

std::ostream& operator<<(std::ostream&, const Vec3& vec);

#endif
