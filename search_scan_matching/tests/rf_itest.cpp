/**
 * @file rf_itest.cpp
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief integration test for range finder object
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
#include "search_scan_matching/range_finder.h"
#include "search_scan_matching/utils.h"
#include "search_scan_matching_test/helper.h"

/**
 * @brief tests construction of Range Finder object
 *
 */
TEST(RangeFinder, Construction) {
  // act
  const RangeFinder rf_0fov_1res(1, 0, 1);
  const RangeFinder rf_1fov_0res(1, 1, 0);
  const RangeFinder rf_1fov_1res(1, 0, 0);
  const RangeFinder rf_0(1, M_PI, M_PI);
  const RangeFinder rf_1(1, M_PI, M_PI_4);
  const RangeFinder rf_2(1, 2 * M_PI, M_PI_4);
  // assert
  EXPECT_THROW(RangeFinder(0, 0, 0), std::runtime_error);
  EXPECT_THROW(RangeFinder(0, 0, -1), std::runtime_error);
  EXPECT_EQ(1, rf_0fov_1res.GetData().size());
  EXPECT_EQ(1, rf_1fov_0res.GetData().size());
  EXPECT_EQ(1, rf_1fov_1res.GetData().size());
  EXPECT_EQ(1, rf_0.GetData().size());
  EXPECT_EQ(5, rf_1.GetData().size());
  EXPECT_EQ(9, rf_2.GetData().size());
}

/**
 * @brief tests execution of Range Finder
 *
 */
TEST(RangeFinder, ExecutionSingleBeam) {
  // arrange
  const double length = 1, width = 2, res = 0.1;
  const double max = 3;
  Grid2D grid_row(comm::Pose2D(), res, res, width);
  Grid2D grid_col(comm::Pose2D(), res, length, res);
  RangeFinder rf(max, 0, 0);
  comm::Pose2D point_right(0, 0, M_PI_2);
  comm::Pose2D point_down(0, 0, 0);
  // test no object where beam is pointing downward
  const auto row_nothing = rf.Execute(grid_row, point_down);
  EXPECT_EQ(1, row_nothing.size());
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, max), row_nothing[0]);

  // test no object where beam is pointing toward the row
  const auto row_nothing_rot = rf.Execute(grid_row, point_right);
  EXPECT_EQ(1, row_nothing_rot.size());
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, max), row_nothing_rot[0]);

  // add object at end of the row
  comm::Cell end_of_row(0, width / res - 1);
  grid_row.SetCellOccupancy(end_of_row, comm::OCCUPIED);
  const auto end_of_row_obj = rf.Execute(grid_row, point_right);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, width - res), end_of_row_obj[0]);

  // add object at mid of the row
  comm::Cell mid_of_row(0, width / 2 / res - 1);
  grid_row.SetCellOccupancy(mid_of_row, comm::OCCUPIED);
  const auto mid_of_row_obj = rf.Execute(grid_row, point_right);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, width / 2 - res),
                      mid_of_row_obj[0]);

  // test no object where beam is pointing toward the col
  const auto col_nothing = rf.Execute(grid_col, point_down);
  EXPECT_EQ(1, col_nothing.size());
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, max), col_nothing[0]);

  // add object at end of the col
  comm::Cell end_of_col(length / res - 1, 0);
  grid_col.SetCellOccupancy(end_of_col, comm::OCCUPIED);
  const auto end_of_col_obj = rf.Execute(grid_col, point_down);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, length - res),
                      end_of_col_obj[0]);

  // add object at mid of the col
  comm::Cell mid_of_col(length / 2 / res - 1, 0);
  grid_col.SetCellOccupancy(mid_of_col, comm::OCCUPIED);
  const auto mid_of_col_obj = rf.Execute(grid_col, point_down);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, length / 2 - res),
                      mid_of_col_obj[0]);
}
/**
 * @brief tests execution of Range Finder
 *
 */
TEST(RangeFinder, ExecutionHalfCircle) {
  // arrange
  const double length = 1, width = 2, res = 0.1;
  const double max = 3;
  Grid2D grid(comm::Pose2D(), res, length, width);
  RangeFinder rf(max, M_PI, M_PI_4);
  comm::Cell buttom_left_corner(length / res - 1, 0);
  comm::Cell top_right_corner(0, width / res - 1);

  // test no object
  const auto nothing = rf.Execute(grid, comm::Pose2D());
  EXPECT_EQ(5, nothing.size());
  EXPECT_RANGE_FINDER(comm::RangeFinderData(-M_PI_2, max), nothing[0]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(-M_PI_4, max), nothing[1]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, max), nothing[2]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(M_PI_4, max), nothing[3]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(M_PI_2, max), nothing[4]);

  // test only bottom left object
  grid.SetCellOccupancy(buttom_left_corner, comm::OCCUPIED);
  // std::cout << buttom_left_corner << std::endl;
  const auto one_corner = rf.Execute(grid, comm::Pose2D());
  EXPECT_EQ(5, one_corner.size());
  EXPECT_RANGE_FINDER(comm::RangeFinderData(-M_PI_2, max), one_corner[0]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(-M_PI_4, max), one_corner[1]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, length - res), one_corner[2]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(M_PI_4, max), one_corner[3]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(M_PI_2, max), one_corner[4]);

  // test only bottom left and top right objects
  grid.SetCellOccupancy(top_right_corner, comm::OCCUPIED);
  const auto two_corners = rf.Execute(grid, comm::Pose2D());
  EXPECT_EQ(5, two_corners.size());
  EXPECT_RANGE_FINDER(comm::RangeFinderData(-M_PI_2, max), two_corners[0]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(-M_PI_4, max), two_corners[1]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(0, length - res), two_corners[2]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(M_PI_4, max), two_corners[3]);
  EXPECT_RANGE_FINDER(comm::RangeFinderData(M_PI_2, width - res),
                      two_corners[4]);
}