#pragma once

#include <math.h>

#include <iostream>
#include <string>
#include <vector>

namespace utils {

const uint8_t FREE = 0;
const uint8_t SENSOR = 1;
const uint8_t OCCUPIED = 255;

struct Cell {
  int row = 0;
  int col = 0;
  Cell();
  Cell(const int r, const int c) : row(r), col(c) {}
  friend std::ostream& operator<<(std::ostream& os, const Cell& c) {
    return os << "[r: " << c.row << ", c: " << c.col << "]";
  }
};

struct CellInfo {
  bool valid = false;
  utils::Cell cell;
  CellInfo(const bool v, const utils::Cell c) : valid(v), cell(c) {}
};

struct Point2D {
  double x = 0.0;
  double y = 0.0;
  Point2D(){};
  Point2D(const double x_axis, const double y_axis) : x(x_axis), y(y_axis) {}
  // Point2D& operator=(const Point2D& rhs) {
  //   this->x = rhs.x;
  //   this->y = rhs.y;
  //   return *this;
  // }
  Point2D operator+(const Point2D& rhs) const {
    Point2D ans;
    ans.x = this->x + rhs.x;
    ans.y = this->y + rhs.y;
    return ans;
  }
  // Point2D& operator+=(const Point2D& rhs) {
  //   this->x += rhs.x;
  //   this->y += rhs.y;
  //   return *this;
  // }
  Point2D operator-(const Point2D& rhs) const {
    Point2D ans;
    ans.x = this->x - rhs.x;
    ans.y = this->y - rhs.y;
    return ans;
  }
  // Point2D& operator-=(const Point2D& rhs) {
  //   this->x -= rhs.x;
  //   this->y -= rhs.y;
  //   return *this;
  // }
  // Point2D operator*(double val) {
  //   Point2D ans;
  //   ans.x = this->x * val;
  //   ans.y = this->y * val;
  //   return ans;
  // }
  // Point2D& operator*=(double val) {
  //   this->x *= val;
  //   this->y *= val;
  //   return *this;
  // }
  // Point2D operator/(double val) {
  //   Point2D ans;
  //   ans.x = this->x / val;
  //   ans.y = this->y / val;
  //   return ans;
  // }
  // Point2D& operator/=(double val) {
  //   this->x /= val;
  //   this->y /= val;
  //   return *this;
  // }
  friend std::ostream& operator<<(std::ostream& os, const Point2D& p) {
    return os << "[x: " << p.x << ", y: " << p.y << "]";
  }
};

struct Pose2D {
  Point2D point;
  double theta = 0;
  Pose2D(){};
  Pose2D(const double x_axis, const double y_axis, double th)
      : point(Point2D(x_axis, y_axis)), theta(th) {}
  Pose2D(const Point2D& p, const double th) : point(p), theta(th) {}
  // Pose2D& operator=(const Pose2D& rhs) {
  //   this->point = rhs.point;
  //   this->theta = rhs.theta;
  //   return *this;
  // }
  Pose2D operator+(const Pose2D& rhs) const {
    Pose2D ans;
    ans.point = this->point + rhs.point;
    ans.theta = this->theta + rhs.theta;
    return ans;
  }
  // Pose2D& operator+=(const Pose2D& rhs) {
  //   this->point += rhs.point;
  //   this->theta += rhs.theta;
  //   return *this;
  // }
  Pose2D operator-(const Pose2D& rhs) const {
    Pose2D ans;
    ans.point = this->point - rhs.point;
    ans.theta = this->theta - rhs.theta;
    return ans;
  }
  // Pose2D& operator-=(const Pose2D& rhs) {
  //   this->point -= rhs.point;
  //   this->theta -= rhs.theta;
  //   return *this;
  // }
  friend std::ostream& operator<<(std::ostream& os, const Pose2D& p) {
    return os << "Point: " << p.point << " theta: " << p.theta;
  }
};

struct Frame2D {
  Pose2D origin;
  std::string id = "";
  Frame2D(){};
  Frame2D(Pose2D pose, std::string name) : origin(pose), id(name){};
  Frame2D Inverse() {
    double cos_theta = std::cos(origin.theta);
    double sin_theta = std::sin(origin.theta);
    double x = -cos_theta * origin.point.x - sin_theta * origin.point.y;
    double y = +sin_theta * origin.point.x - cos_theta * origin.point.y;
    double theta = -origin.theta;
    return Frame2D(Pose2D(x, y, theta), "Inverse " + id);
  }

  Point2D TransformBack(Point2D point) {
    double cos_theta = std::cos(origin.theta);
    double sin_theta = std::sin(origin.theta);
    double x = cos_theta * point.x - sin_theta * point.y + origin.point.x;
    double y = sin_theta * point.x + cos_theta * point.y + origin.point.y;
    return Point2D(x, y);
  }

  Point2D TransformForward(Point2D point) {
    double cos_theta = std::cos(origin.theta);
    double sin_theta = std::sin(origin.theta);
    double x = cos_theta * (point.x - origin.point.x) +
               sin_theta * (point.y - origin.point.y);
    double y = -sin_theta * (point.x - origin.point.x) +
               cos_theta * (point.y - origin.point.y);
    return Point2D(x, y);
  }

  Pose2D TransformBack(Pose2D pose) {
    Point2D point = TransformBack(pose.point);
    double theta = pose.theta + origin.theta;
    return Pose2D(point, theta);
  }

  Pose2D TransformForward(Pose2D pose) {
    Point2D point = TransformForward(pose.point);
    double theta = pose.theta - origin.theta;
    return Pose2D(point, theta);
  }

  friend std::ostream& operator<<(std::ostream& os, const Frame2D& f) {
    return os << "Frame: " << f.id << " Origin: [" << f.origin << "]";
  }
};

double Norm(const Pose2D& p1, const Pose2D& p2);

void Display(const std::vector<std::vector<uint8_t>>& occupancy);

std::vector<std::vector<uint8_t>> Creat2DArray(int h, int w, const uint8_t val);

Pose2D GenerateRandPose(const double linear_stddev = 0.01,
                        const double angular_stddev = 0.01);
}  // namespace utils