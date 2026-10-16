// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <string>
#include <utility>

#include "surface_raster_planner/raster_planner.hpp"

using surface_raster_planner::PlannerOptions;
using surface_raster_planner::RasterPlanner;
using surface_raster_planner::RasterResult;
using surface_raster_planner::TriangleMesh;
using surface_raster_planner::splitSegments;

namespace
{

/// Grid mesh z = f(x, y) on [0, lx] x [0, ly]; cells for which skip(x, y) is true are left empty.
TriangleMesh gridMesh(
  double lx, double ly, double res, const std::function<double(double, double)> & f,
  const std::function<bool(double, double)> & skip = nullptr, bool flip_winding = false)
{
  TriangleMesh m;
  const int nx = static_cast<int>(std::round(lx / res)) + 1;
  const int ny = static_cast<int>(std::round(ly / res)) + 1;
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      m.vertices.emplace_back(i * res, j * res, f(i * res, j * res));
    }
  }
  for (int j = 0; j + 1 < ny; ++j) {
    for (int i = 0; i + 1 < nx; ++i) {
      if (skip && skip((i + 0.5) * res, (j + 0.5) * res)) {
        continue;
      }
      const uint32_t a = j * nx + i, b = a + 1, c = a + nx, d = c + 1;
      if (flip_winding) {
        m.triangles.push_back({a, c, b});
        m.triangles.push_back({b, c, d});
      } else {
        m.triangles.push_back({a, b, c});
        m.triangles.push_back({b, d, c});
      }
    }
  }
  return m;
}

double flat(double, double) {return 0.1;}

PlannerOptions defaultOptions()
{
  PlannerOptions o;
  o.point_spacing = 0.005;
  o.max_ray_length = 0.5;
  return o;
}

void expectOrthonormal(const Eigen::Isometry3d & p)
{
  const Eigen::Matrix3d R = p.linear();
  EXPECT_TRUE((R.transpose() * R).isIdentity(1e-9));
  EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
}

}  // namespace

TEST(RasterPlanner, PcaFrameOfFlatPatch)
{
  RasterPlanner planner;
  std::string error;
  ASSERT_TRUE(planner.setMesh(gridMesh(0.3, 0.1, 0.01, flat), Eigen::Vector3d::UnitZ(), error));
  const Eigen::Matrix3d R = planner.localFrame().linear();
  EXPECT_NEAR(R.col(0).dot(Eigen::Vector3d::UnitX()), 1.0, 1e-6);  // long side, positive sign
  EXPECT_NEAR(R.col(2).dot(Eigen::Vector3d::UnitZ()), 1.0, 1e-6);  // agrees with "up"
  EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
  EXPECT_NEAR(planner.localBox().sizes().x(), 0.3, 1e-6);
  EXPECT_NEAR(planner.localBox().sizes().y(), 0.1, 1e-6);
  EXPECT_FALSE(planner.isAxisAmbiguous(0.9));

  // Opposite "up": e3 flips, the frame stays right-handed
  ASSERT_TRUE(planner.setMesh(gridMesh(0.3, 0.1, 0.01, flat), -Eigen::Vector3d::UnitZ(), error));
  EXPECT_NEAR(planner.localFrame().linear().col(2).z(), -1.0, 1e-6);
  EXPECT_NEAR(planner.localFrame().linear().determinant(), 1.0, 1e-9);
}

TEST(RasterPlanner, SquarePatchIsAmbiguous)
{
  RasterPlanner planner;
  std::string error;
  ASSERT_TRUE(planner.setMesh(gridMesh(0.2, 0.2, 0.01, flat), Eigen::Vector3d::UnitZ(), error));
  EXPECT_TRUE(planner.isAxisAmbiguous(0.9));
}

TEST(RasterPlanner, FlatPatchSerpentine)
{
  RasterPlanner planner;
  std::string error;
  ASSERT_TRUE(planner.setMesh(gridMesh(0.3, 0.1, 0.01, flat), Eigen::Vector3d::UnitZ(), error));

  RasterResult r;
  ASSERT_TRUE(planner.compute(0.02, false, defaultOptions(), r, error)) << error;
  EXPECT_EQ(r.rows, 6u);
  EXPECT_EQ(r.discarded_points, 0u);
  EXPECT_TRUE(r.jump_indices.empty());  // the row change (0.02 m) is not a jump
  EXPECT_EQ(r.poses.size(), r.raster_points);

  const std::size_t per_row = r.poses.size() / r.rows;
  for (std::size_t i = 0; i < r.poses.size(); ++i) {
    const auto & p = r.poses[i];
    expectOrthonormal(p);
    EXPECT_NEAR(p.translation().z(), 0.1, 1e-6);
    EXPECT_NEAR(p.linear().col(2).z(), 1.0, 1e-9);
    // x follows the travel direction: +x on even rows, -x on odd rows
    const double expected = ((i / per_row) % 2 == 0) ? 1.0 : -1.0;
    EXPECT_NEAR(p.linear().col(0).x(), expected, 1e-9);
  }
  // Serpentine: the odd row starts where the even row ended
  EXPECT_NEAR(r.poses[per_row - 1].translation().x(), r.poses[per_row].translation().x(), 1e-9);
}

TEST(RasterPlanner, KeepToolOrientation)
{
  RasterPlanner planner;
  std::string error;
  ASSERT_TRUE(planner.setMesh(gridMesh(0.3, 0.1, 0.01, flat), Eigen::Vector3d::UnitZ(), error));
  RasterResult r;
  ASSERT_TRUE(planner.compute(0.02, true, defaultOptions(), r, error)) << error;
  for (const auto & p : r.poses) {
    EXPECT_NEAR(p.linear().col(0).x(), 1.0, 1e-9);
  }
}

TEST(RasterPlanner, NormalsIndependentOfWinding)
{
  auto bump = [](double x, double y) {
      return 0.05 * std::exp(-((x - 0.15) * (x - 0.15) + (y - 0.05) * (y - 0.05)) / 0.002);
    };
  for (const bool flip : {false, true}) {
    RasterPlanner planner;
    std::string error;
    ASSERT_TRUE(planner.setMesh(
        gridMesh(0.3, 0.1, 0.005, bump, nullptr, flip), Eigen::Vector3d::UnitZ(), error));
    for (const bool interpolate : {true, false}) {
      auto o = defaultOptions();
      o.interpolate_normals = interpolate;
      RasterResult r;
      ASSERT_TRUE(planner.compute(0.01, false, o, r, error)) << error;
      for (const auto & p : r.poses) {
        expectOrthonormal(p);
        EXPECT_GT(p.linear().col(2).z(), 0.0);
        EXPECT_NEAR(p.translation().z(), bump(p.translation().x(), p.translation().y()), 2e-3);
      }
    }
  }
}

TEST(RasterPlanner, HoleCreatesJumps)
{
  // Hole in the middle: rows crossing it are split, the rows around it are not
  auto hole = [](double x, double y) {
      return std::hypot(x - 0.15, y - 0.05) < 0.03;
    };
  RasterPlanner planner;
  std::string error;
  ASSERT_TRUE(planner.setMesh(
      gridMesh(0.3, 0.1, 0.005, flat, hole), Eigen::Vector3d::UnitZ(), error));
  RasterResult r;
  ASSERT_TRUE(planner.compute(0.01, false, defaultOptions(), r, error)) << error;
  EXPECT_GT(r.discarded_points, 0u);
  ASSERT_FALSE(r.jump_indices.empty());
  EXPECT_EQ(r.poses.size() + r.discarded_points, r.raster_points);
  for (std::size_t k = 0; k < r.jump_indices.size(); ++k) {
    EXPECT_GT(r.jump_indices[k], 0u);
    if (k > 0) {
      EXPECT_GT(r.jump_indices[k], r.jump_indices[k - 1]);
    }
  }
  // Inside each segment consecutive poses are close
  const double max_step = 2.0 * 0.01;
  for (const auto & [b, e] : splitSegments(r.poses.size(), r.jump_indices)) {
    for (std::size_t i = b + 1; i < e; ++i) {
      EXPECT_LE((r.poses[i].translation() - r.poses[i - 1].translation()).norm(), max_step);
    }
  }
}

TEST(RasterPlanner, AngleJumpAndShortSegments)
{
  // Roof with a sharp ridge along y = 0.05: the normal changes by about 0.93 rad
  auto roof = [](double, double y) {return 0.1 - 0.5 * std::abs(y - 0.05);};
  RasterPlanner planner;
  std::string error;
  ASSERT_TRUE(planner.setMesh(gridMesh(0.3, 0.1, 0.005, roof), Eigen::Vector3d::UnitZ(), error));

  auto o = defaultOptions();
  RasterResult r;
  ASSERT_TRUE(planner.compute(0.01, false, o, r, error)) << error;
  const auto n_no_angle = r.jump_indices.size();

  o.jump_angle_threshold = 0.35;
  o.interpolate_normals = false;
  ASSERT_TRUE(planner.compute(0.01, false, o, r, error)) << error;
  EXPECT_GT(r.jump_indices.size(), n_no_angle);

  // Dropping short segments keeps the remaining indices consistent
  o.min_segment_poses = 1000;
  ASSERT_FALSE(planner.compute(0.01, false, o, r, error));
  o.min_segment_poses = 2;
  ASSERT_TRUE(planner.compute(0.01, false, o, r, error)) << error;
  for (const auto & [b, e] : splitSegments(r.poses.size(), r.jump_indices)) {
    EXPECT_GE(e - b, 2u);
  }
}

TEST(RasterPlanner, InvalidInputs)
{
  RasterPlanner planner;
  std::string error;
  RasterResult r;
  EXPECT_FALSE(planner.compute(0.01, false, defaultOptions(), r, error));
  EXPECT_FALSE(planner.setMesh(TriangleMesh(), Eigen::Vector3d::UnitZ(), error));
  ASSERT_TRUE(planner.setMesh(gridMesh(0.3, 0.1, 0.01, flat), Eigen::Vector3d::UnitZ(), error));
  EXPECT_FALSE(planner.compute(0.0, false, defaultOptions(), r, error));
  auto o = defaultOptions();
  o.max_raster_points = 10;
  EXPECT_FALSE(planner.compute(0.01, false, o, r, error));
}

TEST(SplitSegments, Basic)
{
  const auto s = splitSegments(10, {3, 7});
  ASSERT_EQ(s.size(), 3u);
  using Segment = std::pair<std::size_t, std::size_t>;
  EXPECT_EQ(s[0], Segment(0, 3));
  EXPECT_EQ(s[1], Segment(3, 7));
  EXPECT_EQ(s[2], Segment(7, 10));
  EXPECT_TRUE(splitSegments(0, {}).empty());
  EXPECT_EQ(splitSegments(5, {}).size(), 1u);
}
