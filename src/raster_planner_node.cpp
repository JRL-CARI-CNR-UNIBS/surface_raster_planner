// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_raster_planner/raster_planner_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <vector>

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace surface_raster_planner
{

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

std_msgs::msg::ColorRGBA rgba(double r, double g, double b, double a = 1.0)
{
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(r);
  c.g = static_cast<float>(g);
  c.b = static_cast<float>(b);
  c.a = static_cast<float>(a);
  return c;
}

/// Well separated colours for consecutive segments (golden ratio steps on the hue).
std_msgs::msg::ColorRGBA segmentColor(std::size_t k)
{
  const double h = std::fmod(0.61803398875 * static_cast<double>(k), 1.0) * 6.0;
  const double s = 0.85;
  const double v = 0.95;
  const double c = v * s;
  const double x = c * (1.0 - std::abs(std::fmod(h, 2.0) - 1.0));
  const double m = v - c;
  double r = 0.0, g = 0.0, b = 0.0;
  switch (static_cast<int>(h)) {
    case 0: r = c; g = x; break;
    case 1: r = x; g = c; break;
    case 2: g = c; b = x; break;
    case 3: g = x; b = c; break;
    case 4: r = x; b = c; break;
    default: r = c; b = x; break;
  }
  return rgba(r + m, g + m, b + m);
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

/// Fan-triangulates polygons with more than three vertices.
TriangleMesh fromPclMesh(const pcl::PolygonMesh & mesh)
{
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);
  TriangleMesh out;
  out.vertices.reserve(vertices.size());
  for (const auto & v : vertices) {
    out.vertices.emplace_back(v.x, v.y, v.z);
  }
  for (const auto & poly : mesh.polygons) {
    for (std::size_t k = 1; k + 1 < poly.vertices.size(); ++k) {
      out.triangles.push_back({
          static_cast<uint32_t>(poly.vertices[0]),
          static_cast<uint32_t>(poly.vertices[k]),
          static_cast<uint32_t>(poly.vertices[k + 1])});
    }
  }
  return out;
}

TriangleMesh fromRosMesh(const shape_msgs::msg::Mesh & mesh)
{
  TriangleMesh out;
  out.vertices.reserve(mesh.vertices.size());
  for (const auto & v : mesh.vertices) {
    out.vertices.emplace_back(v.x, v.y, v.z);
  }
  out.triangles.reserve(mesh.triangles.size());
  for (const auto & t : mesh.triangles) {
    out.triangles.push_back({t.vertex_indices[0], t.vertex_indices[1], t.vertex_indices[2]});
  }
  return out;
}

}  // namespace

RasterPlannerNode::RasterPlannerNode(const rclcpp::NodeOptions & options)
: Node("raster_planner_node", options)
{
  declareParameters();

  // The subscription lives in its own group so that the latest mesh keeps being
  // updated while a service is running.
  sub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Node 2 publishes the mesh with a latched (transient_local) QoS
  const auto latched = rclcpp::QoS(1).transient_local().reliable();

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_group_;
  mesh_sub_ = create_subscription<shape_msgs::msg::Mesh>(
    "mesh", latched, std::bind(&RasterPlannerNode::meshCallback, this, _1), sub_options);

  poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("~/poses", latched);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/path_markers", latched);
  mesh_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/mesh_marker", latched);

  load_topic_srv_ = create_service<Trigger>(
    "~/load_mesh_from_topic",
    std::bind(&RasterPlannerNode::loadFromTopicCallback, this, _1, _2),
    rclcpp::ServicesQoS(), srv_group_);
  load_file_srv_ = create_service<LoadMesh>(
    "~/load_mesh_from_file",
    std::bind(&RasterPlannerNode::loadFromFileCallback, this, _1, _2),
    rclcpp::ServicesQoS(), srv_group_);
  compute_srv_ = create_service<ComputeTrajectory>(
    "~/compute_trajectory",
    std::bind(&RasterPlannerNode::computeTrajectoryCallback, this, _1, _2),
    rclcpp::ServicesQoS(), srv_group_);

  RCLCPP_INFO(
    get_logger(), "raster_planner_node ready: mesh topic '%s'", mesh_sub_->get_topic_name());
}

void RasterPlannerNode::declareParameters()
{
  // Mesh
  declare_parameter<std::string>("mesh_frame", "world");  // shape_msgs/Mesh has no header
  declare_parameter<std::vector<double>>("reference_up", {0.0, 0.0, 1.0});
  declare_parameter<double>("axis_ambiguity_ratio", 0.9);

  // Raster
  declare_parameter<double>("point_spacing", 0.005);      // [m]
  declare_parameter<double>("max_ray_length", 0.5);       // [m]
  declare_parameter<double>("ray_margin", 0.01);          // [m]
  declare_parameter<bool>("interpolate_normals", true);
  declare_parameter<int>("max_raster_points", 1000000);

  // Jumps
  declare_parameter<double>("jump_distance_factor", 2.0);
  declare_parameter<double>("jump_angle_threshold", 0.0);  // [rad], <= 0 disables
  declare_parameter<int>("min_segment_poses", 1);

  // Visualisation
  declare_parameter<double>("marker_line_width", 0.002);  // [m]
  declare_parameter<std::vector<double>>("mesh_color", {0.6, 0.6, 0.6, 0.8});
}

PlannerOptions RasterPlannerNode::readOptions() const
{
  PlannerOptions o;
  o.point_spacing = get_parameter("point_spacing").as_double();
  o.max_ray_length = get_parameter("max_ray_length").as_double();
  o.ray_margin = get_parameter("ray_margin").as_double();
  o.interpolate_normals = get_parameter("interpolate_normals").as_bool();
  o.max_raster_points =
    static_cast<std::size_t>(std::max<int64_t>(1, get_parameter("max_raster_points").as_int()));
  o.jump_distance_factor = get_parameter("jump_distance_factor").as_double();
  o.jump_angle_threshold = get_parameter("jump_angle_threshold").as_double();
  o.min_segment_poses =
    static_cast<std::size_t>(std::max<int64_t>(1, get_parameter("min_segment_poses").as_int()));
  return o;
}

void RasterPlannerNode::meshCallback(const shape_msgs::msg::Mesh::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(latest_mutex_);
  latest_mesh_ = msg;
}

bool RasterPlannerNode::setMesh(
  const TriangleMesh & mesh, const std::string & frame_id, std::string & message)
{
  auto up_param = get_parameter("reference_up").as_double_array();
  up_param.resize(3, 0.0);
  const Eigen::Vector3d up(up_param[0], up_param[1], up_param[2]);

  const auto t0 = std::chrono::steady_clock::now();
  std::string error;
  bool ambiguous = false;
  std::ostringstream ss;
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    if (!planner_.setMesh(mesh, up, error)) {
      message = "Invalid mesh: " + error;
      return false;
    }
    mesh_frame_ = frame_id;
    ambiguous = planner_.isAxisAmbiguous(get_parameter("axis_ambiguity_ratio").as_double());

    const auto & ev = planner_.eigenValues();
    const auto & box = planner_.localBox();
    const auto & R = planner_.localFrame().linear();
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    ss.precision(3);
    ss << "Mesh loaded in frame '" << frame_id << "': " << planner_.mesh().vertices.size()
       << " vertices, " << planner_.mesh().triangles.size() << " triangles, local box "
       << box.sizes().x() << " x " << box.sizes().y() << " x " << box.sizes().z()
       << " m, e1 = [" << R.col(0).transpose() << "], e3 = [" << R.col(2).transpose()
       << "], eigenvalues [" << ev.transpose() << "] (" << ms << " ms)";
  }
  message = ss.str();
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
  if (ambiguous) {
    RCLCPP_WARN(
      get_logger(),
      "The two largest PCA eigenvalues are almost equal: the direction of the long side (e1) "
      "is not well defined and may change between similar meshes");
    message += " [warning: long side direction ambiguous]";
  }
  publishMeshMarker();
  publishPath(RasterResult());  // clear the path of the previous mesh
  return true;
}

void RasterPlannerNode::loadFromTopicCallback(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  shape_msgs::msg::Mesh::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    msg = latest_mesh_;
  }
  if (!msg || msg->triangles.empty()) {
    res->success = false;
    res->message = std::string("No mesh received yet on '") + mesh_sub_->get_topic_name() + "'";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }
  res->success = setMesh(fromRosMesh(*msg), get_parameter("mesh_frame").as_string(), res->message);
  if (!res->success) {
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void RasterPlannerNode::loadFromFileCallback(
  const LoadMesh::Request::SharedPtr req, LoadMesh::Response::SharedPtr res)
{
  const std::string frame =
    req->frame_id.empty() ? get_parameter("mesh_frame").as_string() : req->frame_id;

  std::error_code ec;
  if (req->filename.empty() || !std::filesystem::is_regular_file(req->filename, ec)) {
    res->success = false;
    res->message = "File '" + req->filename + "' not found";
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    return;
  }

  pcl::PolygonMesh mesh;
  if (pcl::io::loadPolygonFile(req->filename, mesh) <= 0 || mesh.polygons.empty()) {
    res->success = false;
    res->message = "Cannot read a mesh from '" + req->filename + "' (use .ply, .obj, .vtk, .stl)";
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    return;
  }
  res->success = setMesh(fromPclMesh(mesh), frame, res->message);
  if (!res->success) {
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void RasterPlannerNode::computeTrajectoryCallback(
  const ComputeTrajectory::Request::SharedPtr req, ComputeTrajectory::Response::SharedPtr res)
{
  const auto t0 = std::chrono::steady_clock::now();
  const PlannerOptions options = readOptions();

  RasterResult result;
  std::string error;
  std::string frame;
  bool ok;
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    ok = planner_.compute(req->row_step, req->keep_tool_orientation, options, result, error);
    frame = mesh_frame_;
  }
  res->discarded_points = result.discarded_points;
  if (!ok) {
    res->success = false;
    res->message = "compute_trajectory failed: " + error;
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    publishPath(RasterResult());
    return;
  }

  res->poses.header.frame_id = frame;
  res->poses.header.stamp = now();
  res->poses.poses.reserve(result.poses.size());
  for (const auto & p : result.poses) {
    res->poses.poses.push_back(tf2::toMsg(p));
  }
  res->jump_indices = result.jump_indices;

  const auto segments = splitSegments(result.poses.size(), result.jump_indices);
  const auto single = std::count_if(
    segments.begin(), segments.end(), [](const auto & s) {return s.second - s.first == 1;});
  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  std::ostringstream ss;
  ss.precision(4);
  ss << result.poses.size() << " poses in " << segments.size() << " segments ("
     << result.rows << " rows, " << result.raster_points << " rays, " << result.discarded_points
     << " discarded points";
  if (result.removed_poses > 0) {
    ss << ", " << result.removed_poses << " poses removed in segments shorter than "
       << options.min_segment_poses;
  }
  if (single > 0) {
    ss << ", " << single << " single-pose segments";
  }
  ss << ") in " << ms << " ms";
  res->success = true;
  res->message = ss.str();
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  publishPath(result);
}

void RasterPlannerNode::publishMeshMarker()
{
  visualization_msgs::msg::Marker marker;
  std::lock_guard<std::mutex> lock(planner_mutex_);
  marker.header.frame_id = mesh_frame_;
  marker.header.stamp = now();
  marker.ns = "mesh";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  auto color = get_parameter("mesh_color").as_double_array();
  color.resize(4, 1.0);
  marker.color = rgba(color[0], color[1], color[2], color[3]);
  const auto & mesh = planner_.mesh();
  marker.points.reserve(3 * mesh.triangles.size());
  for (const auto & t : mesh.triangles) {
    for (const auto i : t) {
      marker.points.push_back(toPoint(mesh.vertices[i]));
    }
  }
  mesh_marker_pub_->publish(marker);
}

void RasterPlannerNode::publishPath(const RasterResult & result)
{
  const double width = get_parameter("marker_line_width").as_double();
  std::string frame;
  Eigen::Isometry3d T_world_local;
  Eigen::Vector3d box_size;
  bool has_mesh;
  {
    std::lock_guard<std::mutex> lock(planner_mutex_);
    frame = mesh_frame_;
    has_mesh = planner_.hasMesh();
    T_world_local = planner_.localFrame();
    box_size = planner_.localBox().sizes();
  }
  const auto stamp = now();

  geometry_msgs::msg::PoseArray poses;
  poses.header.frame_id = frame;
  poses.header.stamp = stamp;
  for (const auto & p : result.poses) {
    poses.poses.push_back(tf2::toMsg(p));
  }
  poses_pub_->publish(poses);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  auto make = [&](const std::string & ns, int id, int type) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = stamp;
      m.ns = ns;
      m.id = id;
      m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      return m;
    };

  // One colour per continuous segment: a line through the poses plus the points themselves
  const auto segments = splitSegments(result.poses.size(), result.jump_indices);
  for (std::size_t k = 0; k < segments.size(); ++k) {
    const auto [b, e] = segments[k];
    auto line = make("segments", static_cast<int>(k), visualization_msgs::msg::Marker::LINE_STRIP);
    auto dots = make("points", static_cast<int>(k), visualization_msgs::msg::Marker::SPHERE_LIST);
    line.scale.x = width;
    dots.scale.x = dots.scale.y = dots.scale.z = 2.0 * width;
    line.color = dots.color = segmentColor(k);
    for (std::size_t i = b; i < e; ++i) {
      const auto p = toPoint(result.poses[i].translation());
      line.points.push_back(p);
      dots.points.push_back(p);
    }
    if (line.points.size() >= 2) {
      markers.markers.push_back(line);
    }
    markers.markers.push_back(dots);
  }

  // Jumps: thin grey lines from the end of a segment to the start of the next one
  if (segments.size() > 1) {
    auto jumps = make("jumps", 0, visualization_msgs::msg::Marker::LINE_LIST);
    jumps.scale.x = 0.5 * width;
    jumps.color = rgba(0.5, 0.5, 0.5, 0.6);
    for (std::size_t k = 1; k < segments.size(); ++k) {
      jumps.points.push_back(toPoint(result.poses[segments[k - 1].second - 1].translation()));
      jumps.points.push_back(toPoint(result.poses[segments[k].first].translation()));
    }
    markers.markers.push_back(jumps);
  }

  // PCA frame: e1 red, e2 green, e3 blue, from the centroid
  if (has_mesh) {
    const double length = 0.3 * box_size.head<2>().maxCoeff();
    const std::array<std_msgs::msg::ColorRGBA, 3> colors = {
      rgba(1, 0, 0), rgba(0, 1, 0), rgba(0, 0, 1)};
    for (int i = 0; i < 3; ++i) {
      auto arrow = make("pca_frame", i, visualization_msgs::msg::Marker::ARROW);
      arrow.scale.x = 2.0 * width;  // shaft diameter
      arrow.scale.y = 4.0 * width;  // head diameter
      arrow.scale.z = 6.0 * width;  // head length
      arrow.color = colors[i];
      const Eigen::Vector3d o = T_world_local.translation();
      arrow.points.push_back(toPoint(o));
      arrow.points.push_back(toPoint(o + length * T_world_local.linear().col(i)));
      markers.markers.push_back(arrow);
    }
  }
  markers_pub_->publish(markers);
}

}  // namespace surface_raster_planner
