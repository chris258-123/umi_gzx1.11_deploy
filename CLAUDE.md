# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Universal Manipulation Interface (UMI) is a robotics manipulation system that combines:
- GoPro-based SLAM for camera pose tracking
- Diffusion Policy for imitation learning
- Real-world robot control (UR5, Franka, ARX arms)
- WSG50 gripper control
- Multi-camera observation system

## Environment Setup

Create conda environment:
```bash
mamba env create -f conda_environment.yaml
conda activate umi
```

System dependencies (Ubuntu 22.04):
```bash
sudo apt install -y libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf
sudo apt install libspnav-dev spacenavd
sudo systemctl start spacenavd
```

Grant HDMI capture card permissions:
```bash
sudo chmod -R 777 /dev/bus/usb
```

## Common Commands

### SLAM Pipeline
Process demonstration data through SLAM pipeline:
```bash
python run_slam_pipeline.py <session_dir>
```

Generate training dataset from SLAM output:
```bash
python scripts_slam_pipeline/07_generate_replay_buffer.py -o <session_dir>/dataset.zarr.zip <session_dir>
```

### Training
Single-GPU training:
```bash
python train.py --config-name=train_diffusion_unet_timm_umi_workspace task.dataset_path=<path_to_dataset.zarr.zip>
```

Multi-GPU training:
```bash
accelerate --num_processes <ngpus> train.py --config-name=train_diffusion_unet_timm_umi_workspace task.dataset_path=<path_to_dataset.zarr.zip>
```

### Real-World Deployment
Evaluate policy on real robot:
```bash
python eval_real.py --robot_config=example/eval_robots_config.yaml -i <checkpoint.ckpt> -o <output_dir>
```

Control robot with SpaceMouse (testing):
```bash
python scripts_real/control_robot_spacemouse.py
```

For Franka robots, launch server on NUC first:
```bash
python scripts_real/launch_franka_interface_server.py
```

Then test control:
```bash
python scripts_real/control_franka.py
```

## Architecture

### Core Components

**diffusion_policy/**: Diffusion policy implementation
- `workspace/`: Training workspaces for different model architectures (UNet, Transformer, etc.)
- `policy/`: Policy implementations
- `model/`: Neural network models
- `dataset/`: Dataset loading and preprocessing
- `config/`: Hydra configuration files

**umi/**: UMI-specific utilities
- `real_world/`: Real robot control interfaces
  - `bimanual_umi_env.py`: Main environment for bimanual robot control
  - `rtde_interpolation_controller.py`: UR5 robot controller (RTDE protocol)
  - `franka_interpolation_controller.py`: Franka robot controller (Polymetis)
  - `wsg_controller.py`: WSG50 gripper controller
  - `multi_uvc_camera.py`: Multi-camera capture system
- `common/`: Shared utilities (pose transforms, SLAM utilities, timing)
- `pipeline/`: Data processing pipeline components

**scripts_slam_pipeline/**: Sequential SLAM processing steps
1. `00_process_videos.py`: Extract frames from GoPro videos
2. `01_extract_gopro_imu.py`: Extract IMU data
3. `02_create_map.py`: Create SLAM map
4. `03_batch_slam.py`: Run SLAM on all demonstrations
5. `04_detect_aruco.py`: Detect ArUco markers for calibration
6. `05_run_calibrations.py`: Compute camera-robot calibrations
7. `06_generate_dataset_plan.py`: Plan dataset generation
8. `07_generate_replay_buffer.py`: Generate final training dataset

**scripts_real/**: Real-world robot control scripts
- `eval_real_*.py`: Policy evaluation scripts
- `demo_real_*.py`: Teleoperation demonstration collection
- `control_*.py`: Manual robot control utilities

### Key Design Patterns

**Robot Control Architecture**:
- Controllers implement interpolation-based control at 10Hz default frequency
- Actions are 7-DOF: [x, y, z, rx, ry, rz, gripper_width]
- Collision avoidance: table collision (height threshold) and inter-arm collision (sphere primitives)
- Latency compensation: actions timestamped and scheduled ahead

**Observation System**:
- Multi-camera RGB observations with configurable horizons
- Robot state: EEF pose (position + rotation) and gripper width
- Observations synchronized using timestamps
- Fisheye camera distortion correction supported

**Policy Interface**:
- Policies receive observation dict with camera images and robot states
- Output action sequences (action chunking with configurable horizon)
- Pose representations configurable: rotation matrix, axis-angle, quaternion, etc.
- Episode start pose used for relative action representation

**Configuration System**:
- Hydra-based configuration in `diffusion_policy/config/`
- Robot hardware config in YAML: `example/eval_robots_config.yaml`
- Key config parameters: robot_type (ur5/franka), IPs, latencies, collision thresholds

### Robot Configuration

Edit `example/eval_robots_config.yaml` for your hardware:
- `robot_type`: "ur5" or "franka"
- `robot_ip`: Robot controller IP address
- `gripper_ip`: WSG50 gripper IP address
- `height_threshold`: Table collision avoidance height
- `sphere_radius`/`sphere_center`: Inter-arm collision avoidance
- `tx_left_right`: Transformation matrix between left/right robot bases (for bimanual)

For Franka robots:
- Requires Polymetis installation on realtime kernel (NUC)
- Set payload mass to 1.8 kg
- Set Flange to Center of Mass: (0.064, -0.06, 0.03)m

For UR5 robots:
- Set payload mass to 1.81 kg
- Set center of gravity: (2, -6, 37)mm
- Switch to remote control mode on UR5e

### Data Flow

**Training Pipeline**:
1. Collect demonstrations with GoPro + robot telemetry
2. Run SLAM pipeline to get camera poses
3. Detect ArUco markers for camera-robot calibration
4. Generate replay buffer (Zarr format) with aligned observations and actions
5. Train diffusion policy with Hydra configs

**Inference Pipeline**:
1. Initialize robot controllers and camera capture
2. Load trained checkpoint
3. Human control loop: SpaceMouse teleoperation
4. Press 'C' to hand control to policy
5. Policy control loop: observe → predict actions → execute with collision avoidance
6. Press 'S' to return control to human

### Important Notes

- SLAM is the most fragile component; success rate depends on data collection quality
- Docker required for ORB_SLAM3 (use `chicheng/orb_slam3` image)
- SpaceMouse control: right button unlocks Z-axis, left button enables rotation
- Policy evaluation: press 'C' to start policy, 'S' to stop, 'Q' to quit
- Camera observation latency ~170ms, compensated in action scheduling
- Default control frequency: 10Hz (configurable with `-f` flag)
- Action horizon (steps_per_inference): 6 steps default
