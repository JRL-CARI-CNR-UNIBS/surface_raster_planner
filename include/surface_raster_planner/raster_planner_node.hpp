// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#pragma once

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <surface_raster_planner_msgs/srv/compute_trajectory.hpp>
#include <surface_raster_planner_msgs/srv/load_mesh.hpp>

#include "surface_raster_planner/raster_planner.hpp"

namespace surface_raster_planner
{

/**
 * Node 3 of the pipeline: generates a raster (serpentine) tool path over the
 * surface mesh reconstructed by node 2 (read from topic) or loaded from file.
 */
class RasterPlannerNode : public rclcpp::Node
{
public:
  explicit RasterPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Trigger = std_srvs::srv::Trigger;
  using LoadMesh = surface_raster_planner_msgs::srv::LoadMesh;
  using ComputeTrajectory = surface_raster_planner_msgs::srv::ComputeTrajectory;

  void declareParameters();

  void meshCallback(const shape_msgs::msg::Mesh::SharedPtr msg);

  void loadFromTopicCallback(
    const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void loadFromFileCallback(
    const LoadMesh::Request::SharedPtr req, LoadMesh::Response::SharedPtr res);
  void computeTrajectoryCallback(
    const ComputeTrajectory::Request::SharedPtr req, ComputeTrajectory::Response::SharedPtr res);

  /// Replaces the current mesh (PCA frame + OBB tree are rebuilt). Returns a description.
  bool setMesh(const TriangleMesh & mesh, const std::string & frame_id, std::string & message);

  PlannerOptions readOptions() const;

  void publishMeshMarker();
  void publishPath(const RasterResult & result);

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::CallbackGroup::SharedPtr srv_group_;
  rclcpp::Subscription<shape_msgs::msg::Mesh>::SharedPtr mesh_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_marker_pub_;
  rclcpp::Service<Trigger>::SharedPtr load_topic_srv_;
  rclcpp::Service<LoadMesh>::SharedPtr load_file_srv_;
  rclcpp::Service<ComputeTrajectory>::SharedPtr compute_srv_;

  // State
  std::mutex latest_mutex_;
  shape_msgs::msg::Mesh::SharedPtr latest_mesh_;

  std::mutex planner_mutex_;
  RasterPlanner planner_;
  std::string mesh_frame_;  ///< frame of the loaded mesh
};

}  // namespace surface_raster_planner
