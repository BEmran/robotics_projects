/**
 * @file grid_itest.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief integration test for grid object
 * @version 0.1
 * @date 2021-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <math.h>

#include "gtest/gtest.h"
#include "search_scan_matching/common.h"
#include "search_scan_matching/grid.h"
#include "search_scan_matching/utils.h"
#include "search_scan_matching_test/helper.h"

/**
 * @brief tests construction of Grid2D object
 *
 */
TEST(Grid2D, Construction) {
  // act
  const Grid2D g_h1_w1(1, 1);
  const Grid2D g_h5_w10(5, 10);
  const Grid2D g_h10_w5(10, 5);
  // assert
  EXPECT_THROW(Grid2D(0, 0), std::runtime_error);
  EXPECT_THROW(Grid2D(0, 1), std::runtime_error);
  EXPECT_THROW(Grid2D(1, 0), std::runtime_error);
  EXPECT_THROW(Grid2D(0, -1), std::runtime_error);
  EXPECT_THROW(Grid2D(-1, 0), std::runtime_error);
  const double def_res = g_h1_w1.GetResolution();
  EXPECT_NEAR(0.05, def_res, EPS);
  EXPECT_SIZE(comm::Size(1 / def_res, 1 / def_res), g_h1_w1.GetSize());
  EXPECT_SIZE(comm::Size(5 / def_res, 10 / def_res), g_h5_w10.GetSize());
  EXPECT_SIZE(comm::Size(10 / def_res, 5 / def_res), g_h10_w5.GetSize());
}

/**
 * @brief tests full construction of Grid2D object
 *
 */
TEST(Grid2D, FullConstruction) {
  // arrange
  const double x = 1.6, y = 2.0, theta = 0.5;
  const double length = 1, width = 2.0, res = 0.01;
  const comm::Pose2D pose(x, y, theta);
  const comm::Size size(length / res, width / res);
  // act
  Grid2D grid_cell(comm::Pose2D(), res, res, res);
  Grid2D grid_row(comm::Pose2D(), res, res, width);
  Grid2D grid_col(comm::Pose2D(), res, length, res);
  Grid2D grid_rect(pose, res, length, width);
  // assert
  EXPECT_THROW(Grid2D(pose, 0, 1, 1), std::runtime_error);
  EXPECT_THROW(Grid2D(pose, -1, 1, 1), std::runtime_error);
  EXPECT_NEAR(res, grid_rect.GetResolution(), EPS);
  EXPECT_POSE2D(pose, grid_rect.GetFrame().origin);
  EXPECT_EQ(size.height, grid_rect.GetOccupancy().size());
  EXPECT_EQ(size.width, grid_rect.GetOccupancy()[0].size());
  uint sum = 0;
  for (const auto& row : grid_rect.GetOccupancy()) {
    for (const auto& ele : row) {
      EXPECT_EQ(ele, comm::FREE);
    }
  }
  EXPECT_SIZE(comm::Size(1, 1), grid_cell.GetSize());
  EXPECT_SIZE(comm::Size(1, size.width), grid_row.GetSize());
  EXPECT_SIZE(comm::Size(size.height, 1), grid_col.GetSize());
  EXPECT_SIZE(size, grid_rect.GetSize());
}

/**
 * @brief tests setting cell as occupied using Cell
 *
 */
TEST(Grid2D, SetOccupancyUsingCell) {
  // arrange
  const double length = 1, width = 2.0, res = 0.1;
  // act
  Grid2D grid(comm::Pose2D(), res, length, width);
  const bool valid__1__1 =
      grid.SetCellOccupancy(comm::Cell(-1, -1), comm::OCCUPIED);
  const bool valid_00_00 =
      grid.SetCellOccupancy(comm::Cell(0, 0), comm::OCCUPIED);
  const bool valid_09_00 =
      grid.SetCellOccupancy(comm::Cell(9, 0), comm::OCCUPIED);
  const bool valid_10_00 =
      grid.SetCellOccupancy(comm::Cell(10, 00), comm::OCCUPIED);
  const bool valid_00_19 =
      grid.SetCellOccupancy(comm::Cell(0, 19), comm::OCCUPIED);
  const bool valid_00_20 =
      grid.SetCellOccupancy(comm::Cell(0, 20), comm::OCCUPIED);
  const bool valid_09_19 =
      grid.SetCellOccupancy(comm::Cell(9, 19), comm::OCCUPIED);
  const bool valid_09_20 =
      grid.SetCellOccupancy(comm::Cell(9, 20), comm::OCCUPIED);

  // assert
  EXPECT_FALSE(valid__1__1);
  EXPECT_TRUE(valid_00_00);
  EXPECT_TRUE(valid_09_00);
  EXPECT_FALSE(valid_10_00);
  EXPECT_TRUE(valid_00_19);
  EXPECT_FALSE(valid_00_20);
  EXPECT_TRUE(valid_09_19);
  EXPECT_FALSE(valid_09_20);
  const auto occ = grid.GetOccupancy();
  EXPECT_TRUE(grid.IsOccupied(comm::Cell(0, 0)));
  EXPECT_TRUE(grid.IsOccupied(comm::Cell(9, 0)));
  EXPECT_TRUE(grid.IsOccupied(comm::Cell(0, 19)));
  EXPECT_TRUE(grid.IsOccupied(comm::Cell(9, 19)));
}

/**
 * @brief tests setting cell as occupied using point
 *
 */
TEST(Grid2D, SetOccupancyUsingPoint) {
  // arrange
  const double length = 1, width = 2.0, res = 0.1;

  const comm::Point2D p_00_00(0, 0);
  const comm::Point2D p_09_00(0.9, 0);
  const comm::Point2D p_00_19(0, 1.9);
  const comm::Point2D p_09_19(0.9, 1.9);
  const comm::Point2D p_1__1(-1, -1);
  const comm::Point2D p_10_00(1, 0);
  const comm::Point2D p_00_20(0, 2);
  const comm::Point2D p_09_20(0.9, 2);
  const comm::Point2D p_10_21(1, 1.9);
  // act
  Grid2D grid(comm::Pose2D(), res, length, width);
  const bool valid_00_00 = grid.SetCellOccupancy(p_00_00, comm::OCCUPIED);
  const bool valid_09_00 = grid.SetCellOccupancy(p_09_00, comm::OCCUPIED);
  const bool valid_00_19 = grid.SetCellOccupancy(p_00_19, comm::OCCUPIED);
  const bool valid_09_19 = grid.SetCellOccupancy(p_09_19, comm::OCCUPIED);
  const bool valid__1__1 = grid.SetCellOccupancy(p_1__1, comm::OCCUPIED);
  const bool valid_10_00 = grid.SetCellOccupancy(p_10_00, comm::OCCUPIED);
  const bool valid_00_20 = grid.SetCellOccupancy(p_00_20, comm::OCCUPIED);
  const bool valid_09_20 = grid.SetCellOccupancy(p_09_20, comm::OCCUPIED);
  const bool valid_10_21 = grid.SetCellOccupancy(p_10_21, comm::OCCUPIED);

  // assert
  EXPECT_TRUE(valid_00_00);
  EXPECT_TRUE(valid_09_00);
  EXPECT_TRUE(valid_00_19);
  EXPECT_TRUE(valid_09_19);
  EXPECT_FALSE(valid__1__1);
  EXPECT_FALSE(valid_10_00);
  EXPECT_FALSE(valid_00_20);
  EXPECT_FALSE(valid_09_20);
  EXPECT_FALSE(valid_10_21);
  const auto occ = grid.GetOccupancy();
  EXPECT_TRUE(grid.IsOccupied(p_00_00));
  EXPECT_TRUE(grid.IsOccupied(p_09_00));
  EXPECT_TRUE(grid.IsOccupied(p_00_19));
  EXPECT_TRUE(grid.IsOccupied(p_09_19));
  EXPECT_TRUE(grid.IsOccupied(comm::Cell(0, 0)));
  EXPECT_TRUE(grid.IsOccupied(comm::Cell(9, 0)));
  EXPECT_TRUE(grid.IsOccupied(comm::Cell(0, 19)));
  EXPECT_TRUE(grid.IsOccupied(comm::Cell(9, 19)));
}
