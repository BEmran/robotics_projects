/**
 * @file grid.h
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief defines Grid class
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <vector>  // vector

#include "search_scan_matching/common.h"  // Point2D, Pose2D, Frame2D

/**
 * @brief defines a 2D Grid consist of and occupancy array defines the obstacle
 * position and used to localize the sensor in the grid
 *
 */
class Grid2D {
 public:
  /**
   * @brief Construct a new Grid 2D object using its length and width in meter
   *
   * @param length length of the grid in meter
   * @param width width of the grid in meter
   */
  Grid2D(const double length, const double width);

  /**
   * @brief Construct a new Grid 2D object using its height and width at
   * specific pose
   *
   * @param frame grid pose with respect to World
   * @param resolution grid resolution
   * @param length length of the grid in meter
   * @param width width of the grid in meter
   */
  Grid2D(const comm::Frame2D frame, const double resolution,
         const double length, const double width);

  /**
   * @brief Create a rectangle obstacle on the grid defined by its length and
   * width in meters
   *
   * @param origin origin pose of the obstacle
   * @param length length of the obstacle in meter
   * @param width width of the obstacle in meter
   */
  void CreateObstacle(const comm::Pose2D origin, const double length,
                      const double width);

  /**
   * @brief indicates the status of a cell defined by projection of the
   * passed point on the grid
   *
   * @param point point to the desired cell
   * @return true Occupied, if cell value == comm::OCCUPIED
   * @return false otherwise
   */
  bool IsOccupied(comm::Point2D point) const;

  /**
   * @brief Gets the Grid size
   *
   * @return Size grid size height x width
   */
  comm::Size GetSize() const;

  /**
   * @brief Gets the Grid resolution
   *
   * @return double resolution
   */
  double GetResolution() const;

  /**
   * @brief Gets the Grid occupancy array
   *
   * @return std::vector<std::vector<uint8_t>> 2D array occupancy
   */
  std::vector<std::vector<uint8_t>> GetOccupancy() const;

  /**
   * @brief Sets the Grid occupancy array
   *
   * @return std::vector<std::vector<uint8_t>> 2D array occupancy
   */
  void SetOccupancy(const std::vector<std::vector<uint8_t>>& occ);

 private:
  comm::Frame2D
      frame_;        // grid pose with respect to reference frame, i.e. World
  double res_;       // grid resolution m/cell
  comm::Size size_;  // grid size
  std::vector<std::vector<uint8_t>> occupancy_;  // occupancy grid
};