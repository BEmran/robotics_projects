/**
 * @file helper.h
 * @author Bara Emran (bara.erman@gmail.com)
 * @brief helping functions to be used for testing the project
 * @version 0.1
 * @date 2021-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <search_scan_matching/common.h>

#include <array>

const double EPS = 0.001;

/*****************************************************************************
 * HELPING FUNCTIONS
 *****************************************************************************/

/**
 * @brief Map a pose from frame A to B
 *
 * @param x_b2a x-axis translation from frame B to A
 * @param y_b2a y-axis translation from frame B to A
 * @param t_b2a rotational angle from frame B to A
 * @param x_a x-axis coordination of pose a
 * @param y_a y-axis coordination of pose a
 * @param t_a angle of pose a
 * @return std::array<double, 3> array represent the mapped point in frame B (x,
 * y, t)
 */

std::array<double, 3> MapAToB(double x_g2w, double y_g2w, double t_g2w,
                              double x_g, double y_g, double t_g);
/**
 * @brief Inverse of a frame defined by a pose
 *
 * @param x x-axis coordination of the pose
 * @param y y-axis coordination of the pose
 * @param t angle of the pose
 * @return std::array<double, 3> array represent the inverse frame
 */
std::array<double, 3> Inverse(double x, double y, double t);

/*****************************************************************************
 * Extra ASSERTIONS
 *****************************************************************************/

/**
 * @brief compare to Size object
 *
 * @param s1 expected Size
 * @param s2 calculated Size
 */
void EXPECT_SIZE(const comm::Size& s1, const comm::Size& s2);

/**
 * @brief compare to Point2D object
 *
 * @param p1 expected Point2D
 * @param p2 calculated Point2D
 */
void EXPECT_POINT2D(const comm::Point2D& p1, const comm::Point2D& p2);

/**
 * @brief compare to Pose2D object
 *
 * @param p1 expected Pose2D
 * @param p2 calculated Pose2D
 */
void EXPECT_POSE2D(const comm::Pose2D& p1, const comm::Pose2D& p2);

/**
 * @brief compare to Frame2D object
 *
 * @param f1 expected Frame2D
 * @param f2 calculated Frame2D
 */
void EXPECT_FRAME2D(const comm::Frame2D& f1, const comm::Frame2D& f2);

/**
 * @brief compare to Cell object
 *
 * @param c1 expected Cell
 * @param c2 calculated Cell
 */
void EXPECT_CELL(const comm::Cell& c1, const comm::Cell& c2);

/**
 * @brief compare to CellInfo object
 *
 * @param c1 expected CellInfo
 * @param c2 calculated CellInfo
 */
void EXPECT_CELLINFO(const comm::CellInfo& c1, const comm::CellInfo& c2);

/**
 * @brief compare to RangeFinderData object
 *
 * @param rfd1 expected RangeFinderData
 * @param rfd2 calculated RangeFinderData
 */
void EXPECT_RANGE_FINDER(const comm::RangeFinderData& rfd1,
                         const comm::RangeFinderData& rfd2);
