# ROS 2 SITL Campaign

This is the ROS 2 offboard and experiment runner used with the PX4/Gazebo Software-in-the-Loop workflow.

- `offboard_all_scenarios.py` publishes PX4 trajectory setpoints and orchestrates hover, drop, multi-drop, and motor-fault scenarios using `rclpy` and `px4_msgs`.
- `pick_drop_runner.py` runs the payload pickup/drop scenario.
- `run_campaign.sh`, `sweep_runner.sh`, and `sweep_test.sh` launch repeatable campaigns and parameter sweeps.
- `aggregate_metrics.py`, `campaign_monitor.py`, `_landing_watchdog.py`, and `analyze_flight_logs.py` process and monitor experiment runs.

The workflow expects a PX4 SITL checkout, ROS 2 with matching `px4_msgs`, a running uXRCE-DDS Agent, and Gazebo Harmonic. Final aggregate metrics and selected plots are in [`../../data/final-sitl/`](../../data/final-sitl/).
