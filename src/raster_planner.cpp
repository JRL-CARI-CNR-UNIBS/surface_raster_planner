// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_raster_planner/raster_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>

#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkGenericCell.h>

namespace surface_raster_planner
{

namespace
{

constexpr double kEps = 1e-9;

/// Barycentric coordinates of p with respect to the triangle (a, b, c),
/// clamped to the triangle (the hit point may be slightly outside because of the tolerance).
Eigen::Vector3d barycentric(
  const Eigen::Vector3d & p, const Eigen::Vector3d & a,
  const Eigen::Vector3d & b, const Eigen::Vector3d & c)
{
  const Eigen::Vector3d v0 = b - a;
  const Eigen::Vector3d v1 = c - a;
  const Eigen::Vector3d v2 = p - a;
  const double d00 = v0.dot(v0);
  const double d01 = v0.dot(v1);
  const double d11 = v1.dot(v1);
  const double d20 = v2.dot(v0);
  const double d21 = v2.dot(v1);
  const double den = d00 * d11 - d01 * d01;
  if (std::abs(den) < kEps * kEps) {
    return Eigen::Vector3d::Constant(1.0 / 3.0);
  }
  const double v = (d11 * d20 - d01 * d21) / den;
  const double w = (d00 * d21 - d01 * d20) / den;
  Eigen::Vector3d bc(1.0 - v - w, v, w);
  bc = bc.cwiseMax(0.0);
  const double sum = bc.sum();
  return sum > kEps ? Eigen::Vector3d(bc / sum) : Eigen::Vector3d::Constant(1.0 / 3.0);
}

}  // namespace

std::vector<std::pair<std::size_t, std::size_t>> splitSegments(
  std::size_t n, const std::vector<uint32_t> & jump_indices)
{
  std::vector<std::pair<std::size_t, std::size_t>> segments;
  if (n == 0) {
    return segments;
  }
  std::size_t begin = 0;
  for (const auto j : jump_indices) {
    if (j > begin && j < n) {
      segments.emplace_back(begin, j);
      begin = j;
    }
  }
  segments.emplace_back(begin, n);
  return segments;
}

bool RasterPlanner::setMesh(
  const TriangleMesh & mesh, const Eigen::Vector3d & up, std::string & error)
{
  poly_data_ = nullptr;
  tree_ = nullptr;
  boundary_edges_ = 0;
  boundary_data_ = nullptr;
  boundary_locator_ = nullptr;
  mesh_ = TriangleMesh();
  face_normals_.clear();
  vertex_normals_.clear();

  if (mesh.vertices.size() < 3 || mesh.triangles.empty()) {
    error = "the mesh is empty";
    return false;
  }

  // Keep only valid, non-degenerate triangles: cell id in the OBB tree == index in mesh_.triangles
  mesh_.vertices = mesh.vertices;
  std::vector<Eigen::Vector3d> raw_normals;  // not normalised, length = 2 * area
  for (const auto & t : mesh.triangles) {
    if (t[0] >= mesh.vertices.size() || t[1] >= mesh.vertices.size() ||
      t[2] >= mesh.vertices.size())
    {
      continue;
    }
    const auto & a = mesh.vertices[t[0]];
    const auto & b = mesh.vertices[t[1]];
    const auto & c = mesh.vertices[t[2]];
    const Eigen::Vector3d n = (b - a).cross(c - a);
    if (!n.allFinite() || n.norm() < kEps * kEps) {
      continue;
    }
    mesh_.triangles.push_back(t);
    raw_normals.push_back(n);
  }
  if (mesh_.triangles.empty()) {
    error = "the mesh has only degenerate triangles";
    mesh_ = TriangleMesh();
    return false;
  }

  // 2. Local frame with PCA on the vertices
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(mesh_.vertices.size());
  for (const auto & v : mesh_.vertices) {
    cloud->push_back(pcl::PointXYZ(
        static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())));
  }
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);
  const Eigen::Matrix3d eigen_vectors = pca.getEigenVectors().cast<double>();
  eigen_values_ = pca.getEigenValues().cast<double>();
  const Eigen::Vector3d centroid = pca.getMean().head<3>().cast<double>();

  Eigen::Vector3d e1 = eigen_vectors.col(0).normalized();
  Eigen::Vector3d e2 = eigen_vectors.col(1).normalized();
  // The sign of the eigenvectors is arbitrary: make it deterministic
  // (largest component of e1 positive) and force a right-handed frame.
  Eigen::Index imax;
  e1.cwiseAbs().maxCoeff(&imax);
  if (e1(imax) < 0.0) {
    e1 = -e1;
  }
  e2 = (e2 - e2.dot(e1) * e1).normalized();
  Eigen::Vector3d e3 = e1.cross(e2);
  // e3 must point towards the side the tool comes from: rotate by pi around e1 if needed
  if (up.norm() > kEps && e3.dot(up) < 0.0) {
    e2 = -e2;
    e3 = -e3;
  }

  T_world_local_.setIdentity();
  T_world_local_.linear().col(0) = e1;
  T_world_local_.linear().col(1) = e2;
  T_world_local_.linear().col(2) = e3;
  T_world_local_.translation() = centroid;

  // 3. Bounding box in the local frame
  const Eigen::Isometry3d T_local_world = T_world_local_.inverse();
  local_box_.setEmpty();
  for (const auto & v : mesh_.vertices) {
    local_box_.extend(T_local_world * v);
  }

  // Normals: face normals oriented towards +e3 (the rays come from that side, and this makes
  // the result independent of the triangle winding), vertex normals as area-weighted mean.
  face_normals_.resize(mesh_.triangles.size());
  vertex_normals_.assign(mesh_.vertices.size(), Eigen::Vector3d::Zero());
  for (std::size_t i = 0; i < mesh_.triangles.size(); ++i) {
    Eigen::Vector3d n = raw_normals[i];
    if (n.dot(e3) < 0.0) {
      n = -n;
    }
    face_normals_[i] = n.normalized();
    for (const auto idx : mesh_.triangles[i]) {
      vertex_normals_[idx] += n;
    }
  }
  for (auto & n : vertex_normals_) {
    const double norm = n.norm();
    n = norm > kEps ? Eigen::Vector3d(n / norm) : Eigen::Vector3d::Zero();
  }

  // 1. vtkPolyData + OBB tree, built once per mesh
  vtkNew<vtkPoints> points;
  points->SetDataTypeToDouble();
  points->SetNumberOfPoints(static_cast<vtkIdType>(mesh_.vertices.size()));
  for (std::size_t i = 0; i < mesh_.vertices.size(); ++i) {
    points->SetPoint(static_cast<vtkIdType>(i), mesh_.vertices[i].data());
  }
  vtkNew<vtkCellArray> cells;
  for (const auto & t : mesh_.triangles) {
    const vtkIdType ids[3] = {t[0], t[1], t[2]};
    cells->InsertNextCell(3, ids);
  }
  poly_data_ = vtkSmartPointer<vtkPolyData>::New();
  poly_data_->SetPoints(points);
  poly_data_->SetPolys(cells);

  tree_ = vtkSmartPointer<vtkOBBTree>::New();
  tree_->SetDataSet(poly_data_);
  tree_->SetTolerance(1e-6);
  tree_->BuildLocator();

  // Boundary: edges shared by exactly one triangle (outline and holes), used by edge_margin
  std::unordered_map<uint64_t, int> edge_count;
  edge_count.reserve(3 * mesh_.triangles.size());
  auto key = [](uint32_t a, uint32_t b) {
      return (static_cast<uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
    };
  for (const auto & t : mesh_.triangles) {
    for (int i = 0; i < 3; ++i) {
      ++edge_count[key(t[i], t[(i + 1) % 3])];
    }
  }
  vtkNew<vtkCellArray> lines;
  for (const auto & [k, count] : edge_count) {
    if (count == 1) {
      const vtkIdType ids[2] = {static_cast<vtkIdType>(k >> 32),
        static_cast<vtkIdType>(k & 0xffffffffu)};
      lines->InsertNextCell(2, ids);
      ++boundary_edges_;
    }
  }
  if (boundary_edges_ > 0) {
    boundary_data_ = vtkSmartPointer<vtkPolyData>::New();
    boundary_data_->SetPoints(points);
    boundary_data_->SetLines(lines);
    boundary_locator_ = vtkSmartPointer<vtkCellLocator>::New();
    boundary_locator_->SetDataSet(boundary_data_);
    boundary_locator_->BuildLocator();
  }
  return true;
}

bool RasterPlanner::nearBoundary(const Eigen::Vector3d & p, double radius) const
{
  if (!boundary_locator_ || !(radius > 0.0)) {
    return false;
  }
  double x[3] = {p.x(), p.y(), p.z()};
  double closest[3];
  vtkNew<vtkGenericCell> cell;
  vtkIdType cell_id;
  int sub_id;
  double dist2;
  return boundary_locator_->FindClosestPointWithinRadius(
    x, radius, closest, cell, cell_id, sub_id, dist2) != 0;
}

bool RasterPlanner::isAxisAmbiguous(double ratio) const
{
  return eigen_values_(0) <= kEps || eigen_values_(1) / eigen_values_(0) > ratio;
}

bool RasterPlanner::castRay(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & end,
  Eigen::Vector3d & hit, vtkIdType & cell) const
{
  vtkNew<vtkPoints> points;
  points->SetDataTypeToDouble();
  vtkNew<vtkIdList> cells;
  double p0[3] = {origin.x(), origin.y(), origin.z()};
  double p1[3] = {end.x(), end.y(), end.z()};
  tree_->IntersectWithLine(p0, p1, points, cells);
  if (points->GetNumberOfPoints() == 0) {
    return false;
  }
  double best = std::numeric_limits<double>::infinity();
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
    Eigen::Vector3d p;
    points->GetPoint(i, p.data());
    const double d = (p - origin).squaredNorm();
    if (d < best) {
      best = d;
      hit = p;
      cell = cells->GetId(i);
    }
  }
  return true;
}

Eigen::Vector3d RasterPlanner::normalAt(
  vtkIdType cell, const Eigen::Vector3d & point, bool interpolate) const
{
  const auto idx = static_cast<std::size_t>(cell);
  Eigen::Vector3d n = face_normals_[idx];
  if (interpolate) {
    const auto & t = mesh_.triangles[idx];
    const Eigen::Vector3d bc = barycentric(
      point, mesh_.vertices[t[0]], mesh_.vertices[t[1]], mesh_.vertices[t[2]]);
    const Eigen::Vector3d ni =
      bc(0) * vertex_normals_[t[0]] + bc(1) * vertex_normals_[t[1]] + bc(2) * vertex_normals_[t[2]];
    if (ni.norm() > 1e-6) {
      n = ni.normalized();
    }
  }
  // Orient towards the side the ray comes from: n . (-ray_dir) = n . e3 > 0
  if (n.dot(T_world_local_.linear().col(2)) < 0.0) {
    n = -n;
  }
  return n;
}

bool RasterPlanner::compute(
  double row_step, bool keep_tool_orientation, const PlannerOptions & options,
  RasterResult & result, std::string & error) const
{
  RasterRequest request;
  request.row_step = row_step;
  request.keep_tool_orientation = keep_tool_orientation;
  return compute(request, options, result, error);
}

bool RasterPlanner::compute(
  const RasterRequest & request, const PlannerOptions & options,
  RasterResult & result, std::string & error) const
{
  result = RasterResult();
  const double row_step = request.row_step;
  if (!hasMesh()) {
    error = "no mesh loaded";
    return false;
  }
  if (!(row_step > 0.0) || !(options.point_spacing > 0.0)) {
    error = "row_step and point_spacing must be positive";
    return false;
  }
  if (!(options.max_ray_length > 0.0)) {
    error = "max_ray_length must be positive";
    return false;
  }
  if (!std::isfinite(request.row_angle)) {
    error = "row_angle must be finite";
    return false;
  }
  if (!(request.edge_margin >= 0.0) || !std::isfinite(request.edge_margin)) {
    error = "edge_margin must be >= 0";
    return false;
  }

  // Raster frame: (d1, d2, e3) = local frame rotated by row_angle around e3
  const Eigen::Matrix3d R_local_raster =
    Eigen::AngleAxisd(request.row_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d T_world_raster = T_world_local_;
  T_world_raster.linear() = T_world_local_.linear() * R_local_raster;

  // Bounding box of the vertices in the raster frame
  Eigen::AlignedBox3d box;
  if (request.row_angle == 0.0) {
    box = local_box_;
  } else {
    const Eigen::Isometry3d T_raster_world = T_world_raster.inverse();
    for (const auto & vtx : mesh_.vertices) {
      box.extend(T_raster_world * vtx);
    }
  }

  const Eigen::Vector3d & lo = box.min();
  const Eigen::Vector3d & hi = box.max();
  // 4. 2D serpentine: rows parallel to d1 at v = v_min + k * row_step, points every point_spacing
  const auto n_rows = static_cast<std::size_t>(std::floor((hi.y() - lo.y()) / row_step + 1e-9)) + 1;
  const auto n_cols =
    static_cast<std::size_t>(std::floor((hi.x() - lo.x()) / options.point_spacing + 1e-9)) + 1;
  if (static_cast<double>(n_rows) * static_cast<double>(n_cols) >
    static_cast<double>(options.max_raster_points))
  {
    std::ostringstream ss;
    ss << "the raster would have " << n_rows << " x " << n_cols << " points, more than "
       << "max_raster_points = " << options.max_raster_points;
    error = ss.str();
    return false;
  }
  result.rows = n_rows;
  result.raster_points = n_rows * n_cols;

  const Eigen::Matrix3d R = T_world_raster.linear();
  const Eigen::Vector3d d1 = R.col(0);
  const Eigen::Vector3d d2 = R.col(1);
  const Eigen::Vector3d e3 = R.col(2);
  const double w_start = hi.z() + options.ray_margin;
  const double ray_length =
    std::min(options.max_ray_length, hi.z() - lo.z() + 2.0 * options.ray_margin);
  const double jump_distance =
    options.jump_distance_factor * std::max(options.point_spacing, row_step);
  const double cos_angle_threshold = std::cos(options.jump_angle_threshold);

  std::vector<Eigen::Isometry3d> poses;
  std::vector<uint32_t> jumps;
  poses.reserve(result.raster_points);
  bool gap = false;  // the next valid pose must open a new segment

  struct Hit
  {
    bool valid{false};
    Eigen::Vector3d point;
    vtkIdType cell{-1};
  };
  std::vector<Hit> row_hits(n_cols);

  for (std::size_t k = 0; k < n_rows; ++k) {
    const bool return_row = (k % 2) == 1;
    const double v = lo.y() + static_cast<double>(k) * row_step;
    // 7. guide x axis: travel direction, or always d1 with keep_tool_orientation
    const Eigen::Vector3d x_guide = (return_row && !request.keep_tool_orientation) ? -d1 : d1;

    // 5. Ray casting along -e3, in travel order
    std::size_t first = n_cols, last = 0;
    for (std::size_t j = 0; j < n_cols; ++j) {
      const std::size_t jj = return_row ? n_cols - 1 - j : j;
      const double u = lo.x() + static_cast<double>(jj) * options.point_spacing;
      const Eigen::Vector3d origin = T_world_raster * Eigen::Vector3d(u, v, w_start);
      const Eigen::Vector3d end = origin - ray_length * e3;
      Hit & h = row_hits[j];
      h.valid = castRay(origin, end, h.point, h.cell);
      if (h.valid) {
        first = std::min(first, j);
        last = j;
      } else {
        ++result.discarded_points;
      }
    }
    if (first == n_cols) {
      gap = !poses.empty();  // the whole row missed the mesh
      continue;
    }

    for (std::size_t j = first; j <= last; ++j) {
      const Hit & h = row_hits[j];
      if (!h.valid) {
        gap = !poses.empty();  // hole inside the row
        continue;
      }
      if (nearBoundary(h.point, request.edge_margin)) {
        ++result.edge_discarded_points;
        continue;
      }

      // 6. Local normal, 7. pose (Gram-Schmidt)
      const Eigen::Vector3d z = normalAt(h.cell, h.point, options.interpolate_normals);
      Eigen::Vector3d x = x_guide - x_guide.dot(z) * z;
      if (x.norm() < 1e-6) {
        // Normal parallel to the travel direction (vertical wall): fall back on d2
        x = d2 - d2.dot(z) * z;
      }
      x.normalize();
      const Eigen::Vector3d y = z.cross(x);

      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear().col(0) = x;
      pose.linear().col(1) = y;
      pose.linear().col(2) = z;
      pose.translation() = h.point;

      // 8. Jump detection
      if (!poses.empty()) {
        const auto & prev = poses.back();
        bool jump = gap;
        jump = jump || (h.point - prev.translation()).norm() > jump_distance;
        jump = jump || (options.jump_angle_threshold > 0.0 &&
          z.dot(prev.linear().col(2)) < cos_angle_threshold);
        if (jump) {
          jumps.push_back(static_cast<uint32_t>(poses.size()));
        }
      }
      gap = false;
      poses.push_back(pose);
    }
  }

  if (poses.empty()) {
    error = result.edge_discarded_points > 0 ?
      "all the hit points are closer than edge_margin to the mesh boundary" :
      "no ray hit the mesh (check max_ray_length and the mesh)";
    return false;
  }

  // Drop the segments that are too short, the boundaries of the kept ones remain jumps
  if (options.min_segment_poses > 1) {
    std::vector<Eigen::Isometry3d> kept;
    std::vector<uint32_t> kept_jumps;
    kept.reserve(poses.size());
    for (const auto & [b, e] : splitSegments(poses.size(), jumps)) {
      if (e - b < options.min_segment_poses) {
        result.removed_poses += static_cast<uint32_t>(e - b);
        continue;
      }
      if (!kept.empty()) {
        kept_jumps.push_back(static_cast<uint32_t>(kept.size()));
      }
      kept.insert(kept.end(), poses.begin() + b, poses.begin() + e);
    }
    if (kept.empty()) {
      error = "all the segments are shorter than min_segment_poses";
      return false;
    }
    poses = std::move(kept);
    jumps = std::move(kept_jumps);
  }

  result.poses = std::move(poses);
  result.jump_indices = std::move(jumps);
  return true;
}

}  // namespace surface_raster_planner
