# Project Component Map

The portfolio repo contains selected project source and results. Large upstream checkouts remain local; their project-specific changes are represented by the focused firmware overlay and simulation assets below.

## Included project components

| Path | Contents |
|---|---|
| `firmware/px4-overlay/src/modules/mc_l1_mo/` | Native PX4 L1 adaptive controller, Momentum Observer, ETC, parameters, and debug message. |
| `firmware/patches/px4-v1.17-l1-mo-overlay.patch` | PX4 board, ROMFS, message, DDS-topic, and module integration changes relative to the `v1.17.0-alpha1` baseline. |
| `ros2/sitl-campaign/` | ROS 2 offboard node, pickup/drop runner, scenario campaign scripts, and log-analysis tools. |
| `simulation/gazebo/` | Payload SDF models and pickup, single-drop, multi-drop, and wind worlds. |
| `middleware/README.md` | Role and upstream location of the Micro XRCE-DDS Agent dependency. |
| `experiments/README.md` | Final scenario coverage and pointers to code/results. |
| `data/final-sitl/` | 63 aggregate/per-run metric tables and selected plots from final ROS 2/PX4 SITL campaigns. |
| `program/` | MATLAB 6-DOF model, supporting optimizer/results, and figures. MATLAB supports modeling and tuning; final comparison metrics come from ROS 2/PX4 SITL with Gazebo. |

## Local checkouts excluded from the portfolio repo

- `firmware/PX4-main/` is the full PX4 checkout used to prepare the overlay. Its `.git`, generated build, and unrelated history are not copied into the portfolio repo.
- `firmware/references/`, `simulation/PX4-gazebo-models/`, `simulation/PX4-SITL_gazebo-classic/`, and `middleware/Micro-XRCE-DDS-Agent/` are separate local upstream clones.
- `ros2/ros2_ws/` contains starter/example packages. `ros2/drone_ws/` is a separate MPC/LESO project, not the final L1+MO campaign source.
- `data/sitl_campaigns/` contains the larger raw time-series archive; only curated final metrics and selected plots are included.
- `archive/` preserves older experiments and snapshots locally.

The full PX4, ROS 2, and middleware upstream trees are dependencies, not project-authored source. Use the overlay, campaign source, and assets in the included paths to review the thesis-specific work.
