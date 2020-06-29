#ifndef _INCLUDE_BOXTYPE_H_
#define _INCLUDE_BOXTYPE_H_

#include <vector>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv.h>


namespace myshape{

struct Point{
  float x;
  float y;
  Point(){}
  Point(float a, float b)
  {
    x = a;
    y = b;
  }
  bool operator <(const Point &p) const {
    return x < p.x || (x == p.x && y < p.y);
  }

  Point operator +(const Point &p) const{
    Point point;
    point.x = x + p.x;
    point.y = y + p.y;
    return point;
  }

  Point operator -(const Point &p) const{
    Point point;
    point.x = x - p.x;
    point.y = y - p.y;
    return point;
  }

  friend std::ostream &operator <<(std::ostream &stream, const Point &p)
  {
      stream << p.x << " " << p.y << std::endl;
      return stream;
  }
  
  friend Point operator *(const float& k, const Point &p){
    Point point;
    point.x = k * p.x;
    point.y = k * p.y;
    return point;
  }

  friend Point operator /(const Point &p,const float& k){
    Point point;
    point.x = p.x / k;
    point.y = p.y / k;
    return point;
  }

  Point rotate(float rad){
    Point point;
    point.x = cos(rad) * x - sin(rad) * y;
    point.y = sin(rad) * x + cos(rad) * y;
    return point;
  }

  //叉积
  float cross(Point p1, Point p2){
    return (p1.x-x)*(p2.y-y) - (p2.x-x)*(p1.y-y);
  }
  //两点之间的距离
  float distance(Point p1){
    float dis = sqrt((p1.x - x) * (p1.x - x) + (p1.y - y) * (p1.y - y));
    return dis;
  }
  float distance(){
    return sqrt(x*x + y*y);
  }
};


class Polygon{
public:
  void inputPolygon(std::vector<Point> pts)
  {
    vertices.clear();
    vertices.insert(vertices.end(), pts.begin(), pts.end());
  }
  std::vector<Point> vertices;
  bool insidePoly(Point p){
    int count = vertices.size();
    if(count < 3)
    {
      return false;
    }

    bool result = false;

    for(int i = 0, j = count - 1; i < count; i++)
    {
      Point p1 = vertices[i];
      Point p2 = vertices[j];

      if((p1.x < p.x && p2.x >= p.x) || (p2.x < p.x && p1.x >= p.x))
      {
        if(p1.y + (p.x - p1.x) / (p2.x - p1.x) * (p2.y - p1.y) < p.y)
        {
          result = !result;
        }
        else if(p1.y + (p.x - p1.x) / (p2.x - p1.x) * (p2.y - p1.y) == p.y)
        {
          return true;
        }
      }
      j = i;
    }
    return result;
  }
};



class Line{
public:
  Line(Point p0, Point p1){
    line = std::pair<Point, Point>(p0, p1);
  }

  float getDistance(){
    return line.first.distance(line.second);
  }

// 点到直线的距离
  float getVerticalDistance(Point point){
    return abs(line.first.cross(line.second, point) / line.first.distance(line.second));
  }

  int intersect(Line l2, Point& intersect_point){
    float x0 = line.first.x;
    float y0 = line.first.y;
    float x1 = line.second.x;
    float y1 = line.second.y;
    float x2 = l2.line.first.x;
    float y2 = l2.line.first.y;
    float x3 = l2.line.second.x;
    float y3 = l2.line.second.y;
    if(((x1-x0) * (y3-y2) - (x3-x2) * (y1-y0)) == 0) return -1;    //如果平行 返回-1
    intersect_point.y = ( (y0-y1)*(y3-y2)*x0 + (y3-y2)*(x1-x0)*y0 + (y1-y0)*(y3-y2)*x2 + (x2-x3)*(y1-y0)*y2 ) / ( (x1-x0)*(y3-y2) + (y0-y1)*(x3-x2) );

    if(y3 != y2){    //防止出现分母为0的情况
      intersect_point.x = x2 + (x3-x2)*(intersect_point.y-y2) / (y3-y2);
    }
    else{
      intersect_point.x = x0 + (x1-x0)*(intersect_point.y-y0) / (y1-y0);
    }
    return 1;
  }
private:
  std::pair<Point, Point> line;

};

// void loadMap(std::string pkg_loc);
// Point toMapPosition(Point pt);
// bool insideMapMask(Point pt);

}




#endif
