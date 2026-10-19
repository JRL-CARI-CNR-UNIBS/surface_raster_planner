# surface_raster_planner

`raster_planner_node` generates a raster (serpentine) tool path over a surface mesh. The mesh is the one reconstructed by node 2 ([`surface_reconstruction`](https://github.com/JRL-CARI-CNR-UNIBS/surface_reconstruction)), read from its topic, or a mesh loaded from file. The output is a `geometry_msgs/PoseArray` (pure geometry, timing is left to the downstream nodes) plus the indices where the path has to leave the surface ("jumps").

For a step-by-step guide, which also works without a robot, a camera or node 2, see the **[tutorial](docs/TUTORIAL.md)**.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- PCL 1.14 (`libpcl-dev`) and VTK 9 (`libvtk9-dev`), `ros-jazzy-tf2-eigen`
- The `surface_raster_planner_msgs` package (`ComputeTrajectory` and `LoadMesh` services). It lives in a separate repository and must be cloned into the same workspace.

```bash
cd [ROS2_WORKSPACE]/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/surface_raster_planner.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/surface_raster_planner_msgs.git
cd ..
rosdep install --from-paths src/surface_raster_planner src/surface_raster_planner_msgs --ignore-src -y
colcon build --packages-up-to surface_raster_planner
source install/setup.bash
```

## Running

```bash
ros2 launch surface_raster_planner raster_planner.launch.py \
    input_mesh:=/cloud_merger_node/mesh \
    params_file:=/path/to/my_config.yaml   # optional
```

| Launch argument | Default | Description |
|---|---|---|
| `input_mesh` | `/cloud_merger_node/mesh` | `shape_msgs/Mesh` topic published by node 2 |
| `params_file` | `config/raster_planner.yaml` | parameter file |

Typical sequence:

```bash
ros2 service call /raster_planner_node/load_mesh_from_topic std_srvs/srv/Trigger
ros2 service call /raster_planner_node/compute_trajectory surface_raster_planner_msgs/srv/ComputeTrajectory \
    "{row_step: 0.02, keep_tool_orientation: false}"
```

## Algorithm

```
load_mesh_from_topic / load_mesh_from_file          compute_trajectory(row_step, keep_tool_orientation)
──────────────────────────────────────────          ───────────────────────────────────────────────────
mesh → vtkPolyData (degenerate triangles dropped)   rows parallel to e1 at v = v_min + k·row_step
 → PCA on the vertices: e1, e2, e3 = e1 × e2         → points every point_spacing along u, odd rows reversed
 → e3 oriented along reference_up                    → for each (u, v): ray from w_max + ray_margin along −e3
 → bounding box in the local frame (u, v, w)         → closest hit (vtkOBBTree::IntersectWithLine)
 → face / vertex normals                             → normal (barycentric or face), oriented towards +e3
 → vtkOBBTree (built once per mesh)                  → pose: z = normal, x = travel direction (Gram-Schmidt)
                                                     → jump detection → PoseArray + markers
```

1. **Local frame (PCA).** `e1` = major axis (long side of the serpentine), `e2` = intermediate axis (short side), `e3 = e1 × e2` (mean normal). The eigenvector signs are arbitrary, so they are fixed as follows: the largest component of `e1` is positive, and `e3` points to the same side as `reference_up` (if not, the frame is rotated by π around `e1`). The rays come from the `+e3` side, so `reference_up` must point to the side where the tool is. If `λ2 / λ1 > axis_ambiguity_ratio` the long side is not well defined (e.g. an almost square patch): a warning is logged and reported in the service response.
2. **Serpentine.** Rows parallel to `e1` from `v_min` to `v_max` with step `row_step`; along each row the points are spaced by `point_spacing` (independent of `row_step`). Odd rows are travelled backwards.
3. **Ray casting.** The ray starts at `w_max + ray_margin` and its length is `min(max_ray_length, w_max − w_min + 2·ray_margin)`. The hit closest to the ray origin is used. If the ray misses the mesh the point is discarded (`discarded_points`) and the next valid point opens a new segment.
4. **Normal.** Barycentric interpolation of the vertex normals of the hit triangle (`interpolate_normals: true`), or the triangle normal. Face normals are oriented towards `+e3` before averaging them into vertex normals, so the result does not depend on the triangle winding. The final normal always satisfies `n · e3 > 0`.
5. **Pose.** `z` = normal; guide `x` = travel direction (`e1` on even rows, `−e1` on odd rows; always `e1` if `keep_tool_orientation` is true), made orthogonal to `z` by Gram-Schmidt; `y = z × x`. If the normal is parallel to the guide direction, `e2` is used as guide.
6. **Jumps.** Pose `i > 0` is a jump if at least one of these holds:
   - some rays missed the mesh between pose `i−1` and pose `i` (also whole rows);
   - `‖p_i − p_{i−1}‖ > jump_distance_factor · max(point_spacing, row_step)` (the `max` avoids flagging the normal row change);
   - `jump_angle_threshold > 0` and the angle between `n_i` and `n_{i−1}` exceeds it.

   `jump_indices` is sorted and never contains 0; the number of continuous segments is `len(jump_indices) + 1`. Segments shorter than `min_segment_poses` are dropped (by default all segments are kept, single-pose ones included; their number is reported in the response message).

### Downstream executor

The executor node is expected to split the `PoseArray` at `jump_indices`, compute a Cartesian joint path for each segment and, between consecutive segments, retract along the normal of the last pose, move freely above the first pose of the next segment and approach it.

## Interfaces

All names are relative to the node (`/raster_planner_node/...`), except the input topic.

### Topics

| Topic | Type | Dir. | Description |
|---|---|---|---|
| `mesh` | `shape_msgs/Mesh` | sub, latched | mesh published by node 2 (remapped by `input_mesh`) |
| `~/poses` | `geometry_msgs/PoseArray` | pub, latched | last computed path |
| `~/path_markers` | `visualization_msgs/MarkerArray` | pub, latched | one colour per segment (`segments`, `points`), jumps (`jumps`, grey), PCA frame (`pca_frame`: `e1` red, `e2` green, `e3` blue) |
| `~/mesh_marker` | `visualization_msgs/Marker` | pub, latched | the loaded mesh (`TRIANGLE_LIST`) |

The "latched" topics use `transient_local` QoS. The `mesh` subscription is `transient_local` + `reliable`, as published by node 2: a volatile publisher is not compatible. In Rviz, set *Durability Policy = Transient Local*.

### Services

| Service | Type | Description |
|---|---|---|
| `~/load_mesh_from_topic` | `std_srvs/Trigger` | uses the last mesh received on `mesh`; its frame is the `mesh_frame` parameter |
| `~/load_mesh_from_file` | `surface_raster_planner_msgs/LoadMesh` | loads `.ply`, `.obj`, `.vtk` or `.stl`; frame from the request or `mesh_frame` |
| `~/compute_trajectory` | `surface_raster_planner_msgs/ComputeTrajectory` | computes the path over the loaded mesh |

The PCA frame and the OBB tree are computed once at load time and reused by all `compute_trajectory` calls until a new mesh is loaded. `compute_trajectory` takes about 50 ms for ~1000 rays on a mesh of 8000 triangles.

## Parameters

All lengths are in metres and all angles in radians. Parameters are **re-read at every service call** (`reference_up` and `axis_ambiguity_ratio` at load time), so they can be tuned at runtime with `ros2 param set`.

| Parameter | Default | Description |
|---|---|---|
| `mesh_frame` | `world` | frame of the mesh read from topic (`shape_msgs/Mesh` has no header) and default for files |
| `reference_up` | `[0, 0, 1]` | direction (mesh frame) towards which `e3` is oriented: the side the tool comes from |
| `axis_ambiguity_ratio` | `0.9` | warn if `λ2 / λ1` exceeds it |
| `point_spacing` | `0.005` | [m] distance between points along a row |
| `max_ray_length` | `0.5` | [m] upper bound of the ray length |
| `ray_margin` | `0.01` | [m] rays start at `w_max + ray_margin` |
| `interpolate_normals` | `true` | barycentric vertex normals (`true`) or triangle normal (`false`) |
| `max_raster_points` | `1000000` | requests generating more rays are refused |
| `jump_distance_factor` | `2.0` | distance threshold factor, see [Algorithm](#algorithm) |
| `jump_angle_threshold` | `0.0` | [rad] angle threshold between consecutive normals; ≤ 0 disables |
| `min_segment_poses` | `1` | segments with fewer poses are dropped |
| `marker_line_width` | `0.002` | [m] width of the path lines in Rviz |
| `mesh_color` | `[0.6, 0.6, 0.6, 0.8]` | RGBA colour of the mesh marker |

## Package layout

```
surface_raster_planner/
├── CMakeLists.txt, package.xml
├── config/raster_planner.yaml              default parameters
├── launch/raster_planner.launch.py
├── include/surface_raster_planner/
│   ├── raster_planner.hpp                  ROS-independent core (PCA, serpentine, ray casting, jumps)
│   └── raster_planner_node.hpp
├── src/raster_planner.cpp                  core implementation (library raster_planner_core)
├── src/raster_planner_node.cpp             ROS node
├── src/raster_planner_main.cpp             main (MultiThreadedExecutor)
├── scripts/make_test_mesh.py               synthetic test surface (PLY file or latched topic)
├── test/test_raster_planner.cpp            unit tests (gtest)
└── docs/TUTORIAL.md
```

Run the unit tests with `colcon test --packages-select surface_raster_planner && colcon test-result --verbose`.

## Authors

Developed by **CARI-JRL**.

Developer and maintainer: Manuel Beschi (<manuel.beschi@unibs.it>)

## License

Apache License 2.0, see [LICENSE](LICENSE).

Copyright 2026 CARI-JRL

## API documentation

The API reference of the classes, functions and scripts is in [docs/API.md](docs/API.md) (generated from the
Doxygen comments).

The code is documented with Doxygen. From the package folder run `doxygen Doxyfile` and open
`doxygen/html/index.html`.
