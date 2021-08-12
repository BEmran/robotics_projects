/**
 * @file common.h
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief defines all common strucure to be used all over the code
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <math.h>  // sin, cos

#include <iostream>  // ostream
#include <string>    // string
#include <vector>    // vector

namespace comm {

// define cell value
const uint8_t FREE = 0;    // free cell
const uint8_t SENSOR = 1;  // sensor cell for debug purposes, treat as free cell
const uint8_t OCCUPIED = 255;  // occupied by an obstacle

/**
 * @brief defines indices of a cell
 * @details a 2d container is indexed by row first then column i.e. Vector[r][c]
 *
 */
struct Cell {
  int row = 0;  // row index
  int col = 0;  // column index

  /**
   * @brief Default Constructor for Cell object
   *
   */
  Cell();

  /**
   * @brief Construct a new Cell object
   *
   * @param r row index
   * @param c column index
   */
  Cell(const int r, const int c) : row(r), col(c) {}

  /**
   * @brief a pretty way to printout cell values
   *
   * @param os output stream
   * @param c cell object to be printed
   * @return std::ostream& output stream with cell information pass to it
   */
  friend std::ostream& operator<<(std::ostream& os, const Cell& c) {
    return os << "[r: " << c.row << ", c: " << c.col << "]";
  }
};

/**
 * @brief Contains cell information in addition to the cell indices
 * @details It is meant to be used mainly as a return type by methods.
 *
 */
struct CellInfo {
  bool valid = false;  // indicate if cell indices are valied
  Cell cell;           // cell object contains cell indices
  /**
   * @brief Construct a new CellInfo object
   *
   * @param v validation of cell indices
   * @param c cell Object
   */
  CellInfo(const bool v, const Cell c) : valid(v), cell(c) {}
};

/**
 * @brief contains a 2D point information of an object
 *
 */
struct Point2D {
  double x = 0.0;  // distance along x-axis in meter
  double y = 0.0;  // distance along y-axis in meter

  /**
   * @brief default constructor of Point2D object
   *
   */
  Point2D(){};

  /**
   * @brief Construct a new Point2D object
   *
   * @param x_axis distance along x-axis
   * @param y_axis distance along y-axis
   */
  Point2D(const double x_axis, const double y_axis) : x(x_axis), y(y_axis) {}

  /**
   * @brief overload addion operator of two Point2D objects
   *
   * @param rhs right-hand-side Point2D object
   * @return Point2D result of arithmetic addition of two Point2D
   */
  Point2D operator+(const Point2D& rhs) const {
    Point2D ans;
    ans.x = this->x + rhs.x;
    ans.y = this->y + rhs.y;
    return ans;
  }

  /**
   * @brief overload subtract operator of two Point2D objects
   *
   * @param rhs right-hand-side Point2D object
   * @return Point2D result of arithmetic subtraction of two Point2D
   */
  Point2D operator-(const Point2D& rhs) const {
    Point2D ans;
    ans.x = this->x - rhs.x;
    ans.y = this->y - rhs.y;
    return ans;
  }

  /**
   * @brief a pretty way to printout Point2D values
   *
   * @param os output stream
   * @param p Point2D object to be printed
   * @return std::ostream& output stream with Point2D information pass to it
   */
  friend std::ostream& operator<<(std::ostream& os, const Point2D& p) {
    return os << "[x: " << p.x << ", y: " << p.y << "]";
  }
};

/**
 * @brief contins a 2D pose information of an object
 *
 */
struct Pose2D {
  Point2D point;     // linear information of the pose
  double theta = 0;  // angular information in rad

  /**
   * @brief default constructor a new Pose2D object
   *
   */
  Pose2D(){};

  /**
   * @brief Construct a new Pose2D object
   *
   * @param x_axis distance along x-axis
   * @param y_axis  distance along y-axis
   * @param th rotation angle (theta)
   */
  Pose2D(const double x_axis, const double y_axis, double th)
      : point(Point2D(x_axis, y_axis)), theta(th) {}

  /**
   * @brief Construct a new Pose2D object using point information
   *
   * @param p linear information
   * @param th rotation angle (theta)
   */
  Pose2D(const Point2D& p, const double th) : point(p), theta(th) {}

  /**
   * @brief overload addion operator of two Pose2D objects
   *
   * @param rhs right-hand-side Pose2D object
   * @return Pose2D result of arithmetic addition of two Pose2D
   */
  Pose2D operator+(const Pose2D& rhs) const {
    Pose2D ans;
    ans.point = this->point + rhs.point;
    ans.theta = this->theta + rhs.theta;
    return ans;
  }

  /**
   * @brief overload subtraction operator of two Pose2D objects
   *
   * @param rhs right-hand-side Pose2D object
   * @return Pose2D result of arithmetic subtraction of two Pose2D
   */
  Pose2D operator-(const Pose2D& rhs) const {
    Pose2D ans;
    ans.point = this->point - rhs.point;
    ans.theta = this->theta - rhs.theta;
    return ans;
  }

  /**
   * @brief a pretty way to printout Pose2D values
   *
   * @param os output stream
   * @param p Pose2D object to be printed
   * @return std::ostream& output stream with Pose2D information pass to it
   */
  friend std::ostream& operator<<(std::ostream& os, const Pose2D& p) {
    return os << "Point: " << p.point << " theta: " << p.theta;
  }
};

/**
 * @brief defines a 2D reference frame to simplify pose/point maping procedures
 *
 */
struct Frame2D {
  Pose2D origin;          // origin pose
  std::string name = "";  // name of the frame

  /**
   * @brief default constructor a new Frame2D object
   *
   */
  Frame2D(){};

  /**
   * @brief Construct a new Frame2D object using pose information
   *
   * @param pose frame origin pose
   * @param name name of the frame
   */
  Frame2D(Pose2D pose, std::string str) : origin(pose), name(str){};

  /**
   * @brief inverse frame pose (origin)
   *
   * @return Frame2D inverted frame
   */
  Frame2D Inverse() {
    // calculate cosine and sine
    double cos_theta = std::cos(origin.theta);
    double sin_theta = std::sin(origin.theta);
    // invert rotation matrix and multiply by the negative of translation
    // t21 = -R12^T * t12
    double x = -cos_theta * origin.point.x - sin_theta * origin.point.y;
    double y = +sin_theta * origin.point.x - cos_theta * origin.point.y;
    // invert the rotation angle
    double theta = -origin.theta;
    // return inverted frame
    return Frame2D(Pose2D(x, y, theta), "Inverse " + name);
  }

  /**
   * @brief Transform a point defined in this frame back to the reference frame
   *
   * @param point point defined in this frame
   * @return Point2D transferred point to the reference frame
   */
  Point2D TransformBack(Point2D point) {
    // calculate cosine and sine
    double cos_theta = std::cos(origin.theta);
    double sin_theta = std::sin(origin.theta);
    // multiply point of intrest by frame rotation and add frame translation
    // P1 = R12 * P2 + t12
    double x = cos_theta * point.x - sin_theta * point.y + origin.point.x;
    double y = sin_theta * point.x + cos_theta * point.y + origin.point.y;
    return Point2D(x, y);
  }

  /**
   * @brief Transform a point defined in this frame forward to this frame
   *
   * @param point point defined in the reference frame
   * @return Point2D transferred point to this print
   */
  Point2D TransformForward(Point2D point) {
    // calculate cosine and sine
    double cos_theta = std::cos(origin.theta);
    double sin_theta = std::sin(origin.theta);
    // multiply the linear difference by the frame rotation
    // P2 = R12 * (P1-t12)
    double x = cos_theta * (point.x - origin.point.x) +
               sin_theta * (point.y - origin.point.y);
    double y = -sin_theta * (point.x - origin.point.x) +
               cos_theta * (point.y - origin.point.y);
    return Point2D(x, y);
  }

  /**
   * @brief Transform a pose defined in this frame back to the reference frame
   *
   * @param pose pose defined in this frame
   * @return Pose2D transferred pose to the reference frame
   */
  Pose2D TransformBack(Pose2D pose) {
    // transfer point back
    Point2D point = TransformBack(pose.point);
    // add rotation angles
    double theta = pose.theta + origin.theta;
    return Pose2D(point, theta);
  }

  /**
   * @brief Transform a pose defined in this frame forward to this frame
   *
   * @param point pose defined in the reference frame
   * @return Point2D transferred pose to this print
   */
  Pose2D TransformForward(Pose2D pose) {
    // transfer point forward
    Point2D point = TransformForward(pose.point);
    // calculate rotation difference
    double theta = pose.theta - origin.theta;
    return Pose2D(point, theta);
  }

  /**
   * @brief a pretty way to printout Frame2D values
   *
   * @param os output stream
   * @param p Frame2D object to be printed
   * @return std::ostream& output stream with Frame2D information pass to it
   */
  friend std::ostream& operator<<(std::ostream& os, const Frame2D& f) {
    return os << "Frame: " << f.name << " Origin: [" << f.origin << "]";
  }
};

/**
 * @brief represents a single data measured by a range finder sensor
 *
 */
struct RangeFinderData {
  double angle = 0;  // ray angle in rad
  double range = 0;  // ray range in m

  /**
   * @brief default constructor a new RangeFinderData object
   *
   */
  RangeFinderData(){};

  /**
   * @brief Construct a new RangeFinderData object
   *
   * @param ang ray angle in rad
   * @param r ray range in m
   */
  RangeFinderData(const double ang, const double r) : angle(ang), range(r) {}

  /**
   * @brief a pretty way to printout RangeFinderData values
   *
   * @param os output stream
   * @param data RangeFinderData object to be printed
   * @return std::ostream& output stream with RangeFinderData information pass
   * to it
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const RangeFinderData& data) {
    return os << "[ang: " << data.angle << ", r: " << data.range << "]";
  }
};

typedef std::vector<RangeFinderData> RangeData;

/**
 * @brief holds a grid size in height x width
 *
 */
struct Size {
  size_t height = 0;
  size_t width = 0;
  /**
   * @brief default constructor a new Size object
   *
   */
  Size(){};

  /**
   * @brief Construct a new Size object
   *
   * @param h grid height
   * @param w grid width
   */
  Size(const double h, const double w) : height(h), width(w) {}

  /**
   * @brief a pretty way to printout Size values
   *
   * @param os output stream
   * @param data Size object to be printed
   * @return std::ostream& output stream with Size information pass
   * to it
   */
  friend std::ostream& operator<<(std::ostream& os, const Size& s) {
    return os << "[h: " << s.height << ", w: " << s.width << "]";
  }
};

}  // namespace comm