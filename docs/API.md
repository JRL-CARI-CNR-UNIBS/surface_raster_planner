# surface_raster_planner API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions and the scripts. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`include/surface_raster_planner/raster_planner.hpp`](#includesurface_raster_plannerraster_plannerhpp)
  - [`surface_raster_planner::TriangleMesh`](#struct-surface_raster_plannertrianglemesh)
  - [`surface_raster_planner::PlannerOptions`](#struct-surface_raster_plannerplanneroptions)
  - [`surface_raster_planner::RasterResult`](#struct-surface_raster_plannerrasterresult)
  - [`surface_raster_planner::RasterPlanner`](#class-surface_raster_plannerrasterplanner)
- [`include/surface_raster_planner/raster_planner_node.hpp`](#includesurface_raster_plannerraster_planner_nodehpp)
  - [`surface_raster_planner::RasterPlannerNode`](#class-surface_raster_plannerrasterplannernode)
- [Scripts](#scripts)

## include/surface_raster_planner/raster_planner.hpp

### struct `surface_raster_planner::TriangleMesh`

Triangle mesh expressed in the mesh frame.

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `vertices` | `std::vector< Eigen::Vector3d >` |  |
| `triangles` | `std::vector< std::array< uint32_t, 3 > >` |  |

### struct `surface_raster_planner::PlannerOptions`

Tuning of the raster generation (node parameters).

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `point_spacing` | `double` | [m] distance between points along a row |
| `max_ray_length` | `double` | [m] upper bound of the ray length |
| `ray_margin` | `double` | [m] the ray starts at w_max + ray_margin |
| `jump_distance_factor` | `double` | jump if distance &gt; factor * max(point_spacing, row_step) |
| `jump_angle_threshold` | `double` | [rad] jump if the normals differ more; &lt;= 0 disables |
| `interpolate_normals` | `bool` | barycentric vertex normals (true) or face normal (false) |
| `min_segment_poses` | `std::size_t` | segments with fewer poses are dropped |
| `max_raster_points` | `std::size_t` | refuse requests generating more raster points |

### struct `surface_raster_planner::RasterRequest`

Per-request raster settings (ComputeTrajectory request).

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `row_step` | `double` | [m] distance between rows, along the row normal d2 |
| `keep_tool_orientation` | `bool` | tool x axis always d1 (true) or travel direction (false) |
| `row_angle` | `double` | [rad] row direction d1 = rotation of e1 around e3 |
| `edge_margin` | `double` | [m] drop poses closer than this to the mesh boundary |

### struct `surface_raster_planner::RasterResult`

Output of `RasterPlanner::compute`.

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `poses` | `std::vector< Eigen::Isometry3d >` | tool poses in the mesh frame |
| `jump_indices` | `std::vector< uint32_t >` | sorted, never 0: first pose of each new segment |
| `discarded_points` | `uint32_t` | raster points whose ray missed the mesh |
| `edge_discarded_points` | `uint32_t` | hit points dropped because closer than edge_margin |
| `removed_poses` | `uint32_t` | poses dropped with segments shorter than min_segment_poses |
| `rows` | `std::size_t` | number of raster rows |
| `raster_points` | `std::size_t` | number of rays cast |

### class `surface_raster_planner::RasterPlanner`

Raster (serpentine) path generator over a triangle mesh, independent of ROS.

`setMesh()` computes the PCA frame of the surface, the normals and the OBB tree once; `compute()` can then be called many times with different row steps.

Local frame: origin in the vertex centroid, e1 = major axis (long side), e2 = intermediate axis (short side), e3 = e1 x e2 (mean normal), with e3 oriented to agree with the reference "up" direction. Rays are cast along -e3.

#### Public functions

##### `setMesh`

```cpp
bool setMesh(const TriangleMesh &mesh, const Eigen::Vector3d &up, std::string &error)
```

Builds the PCA frame, the normals and the OBB tree.

Degenerate triangles are dropped.

**Parameters:**

- `up`: reference direction (mesh frame) towards which e3 must point

##### `hasMesh`

```cpp
bool hasMesh() const
```

True if a mesh is loaded.

##### `mesh`

```cpp
const TriangleMesh & mesh() const
```

The loaded mesh.

##### `localFrame`

```cpp
const Eigen::Isometry3d & localFrame() const
```

T_mesh_local: columns [e1 e2 e3], origin in the centroid.

##### `eigenValues`

```cpp
const Eigen::Vector3d & eigenValues() const
```

PCA eigenvalues, in decreasing order.

##### `localBox`

```cpp
const Eigen::AlignedBox3d & localBox() const
```

Bounding box of the vertices in the local frame (u, v, w).

##### `isAxisAmbiguous`

```cpp
bool isAxisAmbiguous(double ratio) const
```

True if lambda2 / lambda1 &gt; ratio, i.e. the long side direction is not well defined.

##### `boundaryEdges`

```cpp
std::size_t boundaryEdges() const
```

Number of boundary edges (edges shared by a single triangle); 0 for a closed mesh.

##### `compute`

```cpp
bool compute(const RasterRequest &request, const PlannerOptions &options, RasterResult &result, std::string &error) const
```

Compute the raster (serpentine) poses on the mesh.

The row direction is d1 = cos(row_angle) e1 + sin(row_angle) e2 and the rows are spaced by `request.row_step` along d2 = e3 x d1, covering the bounding box of the mesh in the rotated frame (d1, d2, e3). On each row, a ray is cast along -e3 every options.point_spacing and the hit point becomes a pose with z axis along the surface normal. Hit points closer than `request.edge_margin` to the mesh boundary are dropped. A new segment (jump) starts where rays miss the mesh inside a row, where a whole row misses the mesh, or where consecutive poses are far apart or their normals differ too much. Misses before the first hit or after the last hit of a row (the row runs past the outline) do not open a segment by themselves: the distance criterion decides.

**Parameters:**

- `request`: Row step, tool orientation, row angle and edge margin.
- `options`: Raster options.
- `result` [out]: Poses, jump indices and statistics.
- `error` [out]: Error message.

**Returns:** False if no mesh is loaded, the parameters are invalid, or too many points are requested.

```cpp
bool compute(double row_step, bool keep_tool_orientation, const PlannerOptions &options, RasterResult &result, std::string &error) const
```

Same as the previous overload with row_angle = 0 and edge_margin = 0.

**Parameters:**

- `row_step`: Distance between rows [m] (&gt; 0).
- `keep_tool_orientation`: If true, the tool x axis is always e1; if false it follows the travel direction (reversed on return rows).

### Free functions and types

#### `splitSegments`

```cpp
std::vector< std::pair< std::size_t, std::size_t > > splitSegments(std::size_t n, const std::vector< uint32_t > &jump_indices)
```

Splits [0, n) into [begin, end) segments at the given jump indices.

## include/surface_raster_planner/raster_planner_node.hpp

### class `surface_raster_planner::RasterPlannerNode`

Inherits from `rclcpp::Node`.

Node 3 of the pipeline: generates a raster (serpentine) tool path over the surface mesh reconstructed by node 2 (read from topic) or loaded from file.

#### Public functions

##### `RasterPlannerNode`

```cpp
RasterPlannerNode(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
```

Declare the parameters and create subscriptions, publishers and services.

**Parameters:**

- `options`: Node options.

## Scripts

### `scripts/make_test_mesh.py`

```
Synthetic test surface for raster_planner_node, usable without node 2.

Generates a rectangular, slightly curved patch (optionally with a bump, a hole
and a tilt) and either writes it to an ASCII PLY file or publishes it as a
latched shape_msgs/Mesh, like node 2 does.

Examples:
    make_test_mesh.py --output /tmp/test_surface.ply --hole-radius 0.03
    make_test_mesh.py --publish --topic /cloud_merger_node/mesh
```
