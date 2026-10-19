# Tutorial — generating a raster path with `raster_planner_node`

The tutorial has four parts:

1. **[Test with a synthetic mesh](#1-test-with-a-synthetic-mesh)**: no hardware and no node 2 needed.
2. **[Use with surface reconstruction](#2-use-with-node-2-surface_reconstruction)**: the mesh comes from `cloud_merger_node`.
3. **[What to check in Rviz](#3-what-to-check-in-rviz)**
4. **[Parameter tuning and common problems](#4-parameter-tuning-and-common-problems)**

Time required for part 1: about 10 minutes.

---

## 0. Build

```bash
cd ~/projects/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/surface_raster_planner src/surface_raster_planner_msgs --ignore-src -y
colcon build --packages-up-to surface_raster_planner
source install/setup.bash
```

Remember to run `source ~/projects/ros2_ws/install/setup.bash` in every new terminal.

---

## 1. Test with a synthetic mesh

### 1.1 The test mesh

`make_test_mesh.py` builds a 40 × 25 cm patch, bent along the long side and with a 3 cm bump in the middle, 5 cm above the table. Options add a hole (`--hole-radius`) or tilt the patch around its long side (`--tilt`, in radians); `--help` lists them all. The mesh can be written to a PLY file or published as a latched `shape_msgs/Mesh`, exactly as node 2 does.

```bash
ros2 run surface_raster_planner make_test_mesh.py --output /tmp/patch.ply
ros2 run surface_raster_planner make_test_mesh.py --output /tmp/patch_hole.ply --hole-radius 0.03
```

### 1.2 Start-up

**Terminal 1**, the planner:

```bash
ros2 launch surface_raster_planner raster_planner.launch.py
```

You should see:

```
[raster_planner_node]: raster_planner_node ready: mesh topic '/cloud_merger_node/mesh'
```

**Terminal 2**, Rviz:

```bash
rviz2
```

In Rviz:

1. *Global Options → Fixed Frame* = `world`
2. *Add → By topic*:
   - `/raster_planner_node/mesh_marker` → Marker: the loaded mesh.
   - `/raster_planner_node/path_markers` → MarkerArray: the path, one colour per segment.
   - `/raster_planner_node/poses` → PoseArray: set *Shape* = **Axes** to see the whole tool frame (x red, y green, z blue), with a small *Axes Length* (e.g. 0.01).
3. For all three displays set *Topic → Durability Policy* = **Transient Local**: they are "latched" topics, otherwise Rviz misses the messages published before it subscribed.

### 1.3 Load the mesh from file

In **terminal 3**:

```bash
ros2 service call /raster_planner_node/load_mesh_from_file surface_raster_planner_msgs/srv/LoadMesh \
    "{filename: '/tmp/patch.ply', frame_id: 'world'}"
```

The response describes the mesh and its PCA frame:

```
success=True, message="Mesh loaded in frame 'world': 4131 vertices, 8000 triangles, local box 0.4 x 0.25 x 0.0298 m,
e1 = [1 0 0], e3 = [0 0 1], eigenvalues [0.0137 0.00542 3.75e-05] (54 ms)"
```

`e1` is the long side (here the world x axis), `e3` the mean normal, pointing up as requested by `reference_up`. In Rviz the `pca_frame` arrows (red `e1`, green `e2`, blue `e3`) appear in the centroid of the patch.

### 1.4 Compute the path

```bash
ros2 service call /raster_planner_node/compute_trajectory surface_raster_planner_msgs/srv/ComputeTrajectory \
    "{row_step: 0.02, keep_tool_orientation: false}"
```

```
message='1053 poses in 1 segments (13 rows, 1053 rays, 0 discarded points) in 49 ms'
jump_indices=[], discarded_points=0
```

The patch has no holes, so the whole serpentine is a single continuous segment: 13 rows (25 cm / 2 cm + 1) of 81 points (40 cm / 5 mm + 1). In Rviz, the tool z axis (blue) follows the surface normal over the bump, and the x axis (red) points along the travel direction, flipping on every row.

Try `row_step: 0.05`: the rows become fewer and farther apart, the number of points per row does not change (it depends only on `point_spacing`).

### 1.5 Keep the tool orientation

```bash
ros2 service call /raster_planner_node/compute_trajectory surface_raster_planner_msgs/srv/ComputeTrajectory \
    "{row_step: 0.02, keep_tool_orientation: true}"
```

Now the x axis always points along `+e1`: on the return rows it is **opposite** to the travel direction. This is intended (tools with a working direction), but check it in Rviz before running it on the robot.

### 1.6 A surface with a hole: jumps

```bash
ros2 service call /raster_planner_node/load_mesh_from_file surface_raster_planner_msgs/srv/LoadMesh \
    "{filename: '/tmp/patch_hole.ply'}"
ros2 service call /raster_planner_node/compute_trajectory surface_raster_planner_msgs/srv/ComputeTrajectory \
    "{row_step: 0.02, keep_tool_orientation: false}"
```

```
message='1029 poses in 4 segments (13 rows, 1053 rays, 24 discarded points) in 49 ms'
jump_indices=[423, 537, 568], discarded_points=24
```

The three rows crossing the hole are interrupted: the 24 rays inside the hole miss the mesh, and each interruption opens a new segment. In Rviz every segment has a different colour and the jumps are drawn as thin grey lines. The executor splits the path at `jump_indices` and, at each jump, retracts the tool along the normal and approaches the next segment from above.

### 1.7 Reading the mesh from a topic

Surface Reconstruction Node publishes the mesh as a latched `shape_msgs/Mesh`. The test script can do the same; here the patch is also tilted by 0.35 rad (20°) around its long side:

```bash
ros2 run surface_raster_planner make_test_mesh.py --publish --topic /cloud_merger_node/mesh --tilt 0.35
```

Then, in another terminal:

```bash
ros2 service call /raster_planner_node/load_mesh_from_topic std_srvs/srv/Trigger
ros2 service call /raster_planner_node/compute_trajectory surface_raster_planner_msgs/srv/ComputeTrajectory \
    "{row_step: 0.02, keep_tool_orientation: false}"
```

The response now reports `e3 = [0 -0.342 0.94]`: the serpentine plane follows the mean inclination of the patch, so the rays hit the surface almost perpendicularly. `shape_msgs/Mesh` has no header: its frame is the `mesh_frame` parameter (`world` by default).

---

## 2. Use with Surface Reconstruction Node (`surface_reconstruction`)

Start Surface Reconstruction Node and reconstruct the surface as described in its tutorial (with `fake_camera.py` or with the real robot), then:

```bash
ros2 launch surface_raster_planner raster_planner.launch.py   # input_mesh defaults to /cloud_merger_node/mesh
ros2 service call /raster_planner_node/load_mesh_from_topic std_srvs/srv/Trigger
ros2 service call /raster_planner_node/compute_trajectory surface_raster_planner_msgs/srv/ComputeTrajectory \
    "{row_step: 0.02, keep_tool_orientation: false}"
```

Things to check:

- `mesh_frame` must be the `world_frame` of node 2 (default `world` for both).
- `reference_up` must point to the side the tool comes from, expressed in `mesh_frame`. With the table at constant z in `world`, `[0, 0, 1]` is correct.
- The mesh is read **when `load_mesh_from_topic` is called**: after a new `compute_surface` on node 2, call it again. The same holds for a saved mesh: `/cloud_merger_node/save` writes a PLY that `load_mesh_from_file` can read later, without node 2 running.

---

## 3. What to check in Rviz

| What | Where | What to look for |
|---|---|---|
| PCA frame | `path_markers`, ns `pca_frame` | red `e1` along the long side, blue `e3` towards the tool. If `e3` points into the table, fix `reference_up` |
| Normals | `poses` with *Shape = Axes* | blue z axes perpendicular to the surface and all on the tool side. Normals pointing randomly mean a bad mesh (see section 4) |
| Segments | `path_markers`, ns `segments` | one colour per segment. Many short segments on a continuous surface mean jump thresholds too tight |
| Borders | `path_markers`, ns `points` | rows near `v_min`/`v_max` and the ends of each row may have few valid points; single-pose segments are listed in the response |
| Return rows | `poses` | with `keep_tool_orientation: true` the x axis is opposite to the travel direction on odd rows (intended) |

---

## 4. Parameter tuning and common problems

### Rules of thumb

| Parameter | Suggested value |
|---|---|
| `point_spacing` | 1–2 times the mesh edge length; smaller values only interpolate the same triangles |
| `row_step` (request) | the tool working width minus the desired overlap |
| `jump_distance_factor` | 2–3: large enough to tolerate the slope of the surface, small enough to catch gaps |
| `jump_angle_threshold` | 0 (disabled), or 0.5–1.0 rad (≈ 30–60°) to split the path at sharp edges |
| `max_ray_length` | a few centimetres more than the surface height (`w` size in the load message) |

### Common problems

| Symptom | Likely cause | Fix |
|---|---|---|
| `No mesh received yet` | wrong topic, or node 2 has not computed a surface yet | `ros2 topic info -v /cloud_merger_node/mesh`; launch with `input_mesh:=...`; call `compute_surface` on node 2 |
| `no mesh loaded` | `compute_trajectory` before loading a mesh | call `load_mesh_from_topic` or `load_mesh_from_file` first |
| `no ray hit the mesh` | `max_ray_length` too short, or `reference_up` wrong | check the `w` size and `e3` in the load message |
| Warning `long side direction ambiguous` | almost square surface: `e1` may rotate between similar meshes | expected for square parts; if a fixed direction matters, check `e1` in the load message |
| Normals pointing into the part | `reference_up` points to the wrong side | set `reference_up` towards the tool |
| Noisy, irregular normals | coarse or rippled mesh, inconsistent Poisson normals | `interpolate_normals: true`; improve the mesh in node 2 (MLS radius, Poisson depth) |
| Many jumps on a continuous surface | steep regions: the 3D distance between rays exceeds the threshold | increase `jump_distance_factor`, or `jump_angle_threshold` too tight |
| Gaps not detected as jumps | Poisson filled the hole (not trimmed in node 2) | decrease `poisson_trim_distance` in node 2; or use `jump_angle_threshold` |
| Many single-pose segments at the borders | rows grazing the mesh boundary | `min_segment_poses: 2` (or more) drops them |
| `the raster would have ... points` | `point_spacing` or `row_step` too small for the surface size | increase them, or `max_raster_points` |
