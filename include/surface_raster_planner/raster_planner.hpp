// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <vtkCellLocator.h>
#include <vtkOBBTree.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace surface_raster_planner
{

/// Triangle mesh expressed in the mesh frame.
struct TriangleMesh
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<std::array<uint32_t, 3>> triangles;
};

/// Tuning of the raster generation (node parameters).
struct PlannerOptions
{
  double point_spacing{0.005};        ///< [m] distance between points along a row
  double max_ray_length{0.5};         ///< [m] upper bound of the ray length
  double ray_margin{0.01};            ///< [m] the ray starts at w_max + ray_margin
  double jump_distance_factor{2.0};   ///< jump if distance > factor * max(point_spacing, row_step)
  double jump_angle_threshold{0.0};   ///< [rad] jump if the normals differ more; <= 0 disables
  bool interpolate_normals{true};     ///< barycentric vertex normals (true) or face normal (false)
  std::size_t min_segment_poses{1};   ///< segments with fewer poses are dropped
  std::size_t max_raster_points{1000000};  ///< refuse requests generating more raster points
};

/// Per-request raster settings (ComputeTrajectory request).
struct RasterRequest
{
  double row_step{0.02};              ///< [m] distance between rows, along the row normal d2
  bool keep_tool_orientation{false};  ///< tool x axis always d1 (true) or travel direction (false)
  double row_angle{0.0};              ///< [rad] row direction d1 = rotation of e1 around e3
  double edge_margin{0.0};            ///< [m] drop poses closer than this to the mesh boundary
};

/// Output of RasterPlanner::compute.
struct RasterResult
{
  std::vector<Eigen::Isometry3d> poses;  ///< tool poses in the mesh frame
  std::vector<uint32_t> jump_indices;    ///< sorted, never 0: first pose of each new segment
  uint32_t discarded_points{0};          ///< raster points whose ray missed the mesh
  uint32_t edge_discarded_points{0};     ///< hit points dropped because closer than edge_margin
  uint32_t removed_poses{0};             ///< poses dropped with segments shorter than min_segment_poses
  std::size_t rows{0};                   ///< number of raster rows
  std::size_t raster_points{0};          ///< number of rays cast
};

/// Splits [0, n) into [begin, end) segments at the given jump indices.
std::vector<std::pair<std::size_t, std::size_t>> splitSegments(
  std::size_t n, const std::vector<uint32_t> & jump_indices);

/**
 * Raster (serpentine) path generator over a triangle mesh, independent of ROS.
 *
 * setMesh() computes the PCA frame of the surface, the normals and the OBB tree
 * once; compute() can then be called many times with different row steps.
 *
 * Local frame: origin in the vertex centroid, e1 = major axis (long side),
 * e2 = intermediate axis (short side), e3 = e1 x e2 (mean normal), with e3
 * oriented to agree with the reference "up" direction. Rays are cast along -e3.
 */
class RasterPlanner
{
public:
  /// Builds the PCA frame, the normals and the OBB tree. Degenerate triangles are dropped.
  /// @param up reference direction (mesh frame) towards which e3 must point
  bool setMesh(const TriangleMesh & mesh, const Eigen::Vector3d & up, std::string & error);

  /**
   * @brief True if a mesh is loaded.
   */
  bool hasMesh() const {return tree_ != nullptr;}
  /**
   * @brief The loaded mesh.
   */
  const TriangleMesh & mesh() const {return mesh_;}

  /// T_mesh_local: columns [e1 e2 e3], origin in the centroid.
  const Eigen::Isometry3d & localFrame() const {return T_world_local_;}
  /// PCA eigenvalues, in decreasing order.
  const Eigen::Vector3d & eigenValues() const {return eigen_values_;}
  /// Bounding box of the vertices in the local frame (u, v, w).
  const Eigen::AlignedBox3d & localBox() const {return local_box_;}
  /// True if lambda2 / lambda1 > ratio, i.e. the long side direction is not well defined.
  bool isAxisAmbiguous(double ratio) const;
  /// Number of boundary edges (edges shared by a single triangle); 0 for a closed mesh.
  std::size_t boundaryEdges() const {return boundary_edges_;}

  /**
   * @brief Compute the raster (serpentine) poses on the mesh.
   *
   * The row direction is d1 = cos(row_angle) e1 + sin(row_angle) e2 and the rows are spaced by
   * request.row_step along d2 = e3 x d1, covering the bounding box of the mesh in the rotated
   * frame (d1, d2, e3). On each row, a ray is cast along -e3 every options.point_spacing and the
   * hit point becomes a pose with z axis along the surface normal. Hit points closer than
   * request.edge_margin to the mesh boundary are dropped. A new segment (jump) starts where rays
   * miss the mesh inside a row, where a whole row misses the mesh, or where consecutive poses are
   * far apart or their normals differ too much. Misses before the first hit or after the last
   * hit of a row (the row runs past the outline) do not open a segment by themselves: the
   * distance criterion decides.
   * @param request Row step, tool orientation, row angle and edge margin.
   * @param options Raster options.
   * @param[out] result Poses, jump indices and statistics.
   * @param[out] error Error message.
   * @return False if no mesh is loaded, the parameters are invalid, or too many points are requested.
   */
  bool compute(
    const RasterRequest & request, const PlannerOptions & options,
    RasterResult & result, std::string & error) const;

  /**
   * @brief Same as compute(const RasterRequest &, ...) with row_angle = 0 and edge_margin = 0.
   * @param row_step Distance between rows [m] (> 0).
   * @param keep_tool_orientation If true, the tool x axis is always e1; if false it follows the
   * travel direction (reversed on return rows).
   */
  bool compute(
    double row_step, bool keep_tool_orientation, const PlannerOptions & options,
    RasterResult & result, std::string & error) const;

private:
  /// Closest intersection to the ray origin; false if the ray misses the mesh.
  bool castRay(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & end,
    Eigen::Vector3d & hit, vtkIdType & cell) const;

  /// Distance from p to the mesh boundary is below radius (false if the mesh has no boundary).
  bool nearBoundary(const Eigen::Vector3d & p, double radius) const;

  /// Normal in the hit point, oriented towards +e3 (the side the ray comes from).
  Eigen::Vector3d normalAt(vtkIdType cell, const Eigen::Vector3d & point, bool interpolate) const;

  TriangleMesh mesh_;
  std::vector<Eigen::Vector3d> face_normals_;
  std::vector<Eigen::Vector3d> vertex_normals_;
  Eigen::Isometry3d T_world_local_{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d eigen_values_{Eigen::Vector3d::Zero()};
  Eigen::AlignedBox3d local_box_;

  vtkSmartPointer<vtkPolyData> poly_data_;
  vtkSmartPointer<vtkOBBTree> tree_;

  std::size_t boundary_edges_{0};
  vtkSmartPointer<vtkPolyData> boundary_data_;   ///< boundary edges as lines
  vtkSmartPointer<vtkCellLocator> boundary_locator_;
};

}  // namespace surface_raster_planner
