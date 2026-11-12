# PX4 Firmware Overlay

This folder contains the project-specific PX4 firmware contribution, not a second copy of the full PX4 upstream tree.

- `px4-overlay/src/modules/mc_l1_mo/` contains the native `ScheduledWorkItem` controller, parameters, and debug message.
- `patches/px4-v1.17-l1-mo-overlay.patch` contains the board, ROMFS, uORB/DDS, and module-registration changes needed to integrate the controller into the PX4 v1.17.0-alpha1 baseline used to prepare the overlay.

The patch is relative to the PX4 commit tagged `v1.17.0-alpha1`. From that clean PX4 checkout, apply it with `git apply --check <patch-path>` and then `git apply <patch-path>`. The source module is copied separately for easier review. Pixhawk 2.4.8 / FMUv3 is the embedded validation target.
