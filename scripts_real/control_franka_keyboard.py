#!/usr/bin/env python3
"""
Keyboard-controlled Franka robot with native gripper.

Controls:
- W/S: Move forward/backward (X axis)
- A/D: Move left/right (Y axis)
- F/V: Move up/down (Z axis)
- I/O: Open/close gripper
- Q: Quit

Usage:
    python control_franka_keyboard.py --robot_ip 172.16.13.100
"""

import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import click
import time
import numpy as np
from multiprocessing.managers import SharedMemoryManager
import scipy.spatial.transform as st
from umi.common.precise_sleep import precise_wait
from umi.real_world.franka_interpolation_controller import FrankaInterpolationController
from umi.real_world.franka_gripper_controller import FrankaGripperController
from pynput.keyboard import Key, KeyCode, Listener
from threading import Lock


class KeyboardState:
    """Track keyboard state with press/release events."""
    def __init__(self):
        self.pressed_keys = set()
        self.lock = Lock()
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.quit_flag = False

    def on_press(self, key):
        with self.lock:
            self.pressed_keys.add(key)
            # Check for quit key
            if key == KeyCode(char='q'):
                self.quit_flag = True

    def on_release(self, key):
        with self.lock:
            self.pressed_keys.discard(key)

    def is_pressed(self, key):
        with self.lock:
            return key in self.pressed_keys

    def should_quit(self):
        with self.lock:
            return self.quit_flag

    def stop(self):
        self.listener.stop()


@click.command()
@click.option('-r', '--robot_ip', default='172.16.13.100', help='NUC IP address')
@click.option('-p', '--robot_port', type=int, default=4242, help='zerorpc port')
@click.option('-f', '--frequency', type=float, default=30, help='Control frequency (Hz)')
@click.option('--pos_speed', type=float, default=0.1, help='Position speed (m/s)')
@click.option('--gripper_speed', type=float, default=0.02, help='Gripper speed (m/s)')
def main(robot_ip, robot_port, frequency, pos_speed, gripper_speed):
    """Control Franka robot with keyboard."""

    dt = 1 / frequency
    command_latency = dt / 2
    max_gripper_width = 0.08  # Franka Hand max width in meters

    print("Starting Franka keyboard control...")
    print("\nControls:")
    print("  W/S: Move forward/backward (X axis)")
    print("  A/D: Move left/right (Y axis)")
    print("  F/V: Move up/down (Z axis)")
    print("  I/O: Open/close gripper")
    print("  Q: Quit")
    print("\nPress keys to control the robot...")

    # Create keyboard state tracker
    keyboard = KeyboardState()

    try:
        with SharedMemoryManager() as shm_manager:
            with FrankaGripperController(
                shm_manager=shm_manager,
                robot_ip=robot_ip,
                robot_port=robot_port,
                frequency=frequency,
                move_max_speed=0.2,
                move_max_force=70.0,
                receive_latency=0.0,
                verbose=False
            ) as gripper, \
            FrankaInterpolationController(
                shm_manager=shm_manager,
                robot_ip=robot_ip,
                robot_port=robot_port,
                frequency=200,
                Kx_scale=1.0,
                Kxd_scale=np.array([2.0, 1.5, 2.0, 1.0, 1.0, 1.0]),
                verbose=False
            ) as controller:

                print('\nRobot ready!')

                # Get initial state
                state = controller.get_state()
                target_pose = state['ActualTCPPose'].copy()

                # Get initial gripper position
                gripper_state = gripper.get_state()
                if gripper_state is not None:
                    gripper_target_pos = gripper_state['gripper_position']
                else:
                    gripper_target_pos = 0.04  # Default mid-range

                print(f"Initial pose: {target_pose}")
                print(f"Initial gripper width: {gripper_target_pos:.4f}m\n")

                # Initialize timing
                t_start = time.monotonic()
                gripper.restart_put(t_start - time.monotonic() + time.time())

                iter_idx = 0

                try:
                    while not keyboard.should_quit():
                        t_cycle_end = t_start + (iter_idx + 1) * dt
                        t_sample = t_cycle_end - command_latency
                        t_command_target = t_cycle_end + dt

                        # Wait until sample time
                        precise_wait(t_sample)

                        # Debug: show loop is running
                        if iter_idx % 30 == 0:  # Print every second at 30Hz
                            print(f"Loop running: iter={iter_idx}")

                        # Calculate position delta based on pressed keys
                        dpos = np.zeros(3)

                        # Debug: show pressed keys
                        with keyboard.lock:
                            if len(keyboard.pressed_keys) > 0:
                                print(f"DEBUG: pressed_keys = {keyboard.pressed_keys}", flush=True)

                        # WASD for XY movement
                        w_pressed = keyboard.is_pressed(KeyCode(char='w'))
                        if w_pressed:
                            print("DEBUG: W key detected!")
                            dpos[0] += pos_speed / frequency  # Forward (X+)
                        if keyboard.is_pressed(KeyCode(char='s')):
                            dpos[0] -= pos_speed / frequency  # Backward (X-)
                        if keyboard.is_pressed(KeyCode(char='a')):
                            dpos[1] += pos_speed / frequency  # Left (Y+)
                        if keyboard.is_pressed(KeyCode(char='d')):
                            dpos[1] -= pos_speed / frequency  # Right (Y-)

                        # FV for Z movement
                        if keyboard.is_pressed(KeyCode(char='f')):
                            dpos[2] += pos_speed / frequency  # Up (Z+)
                        if keyboard.is_pressed(KeyCode(char='v')):
                            dpos[2] -= pos_speed / frequency  # Down (Z-)

                        # Update target pose
                        target_pose[:3] += dpos

                        # Print debug info when keys are pressed
                        if np.any(dpos != 0):
                            print(f"Moving: dpos={dpos}, new_pose={target_pose[:3]}")

                        # Gripper control (IO keys)
                        gripper_dpos = 0.0
                        if keyboard.is_pressed(KeyCode(char='i')):
                            gripper_dpos = gripper_speed / frequency  # Open
                        if keyboard.is_pressed(KeyCode(char='o')):
                            gripper_dpos = -gripper_speed / frequency  # Close

                        # Update gripper target (clamp to valid range)
                        gripper_target_pos = np.clip(
                            gripper_target_pos + gripper_dpos,
                            0.0,  # Fully closed
                            max_gripper_width  # Fully open
                        )

                        # Print debug info for gripper
                        if gripper_dpos != 0:
                            print(f"Gripper: dpos={gripper_dpos:.4f}, target={gripper_target_pos:.4f}")

                        # Send commands
                        controller.schedule_waypoint(
                            target_pose,
                            t_command_target - time.monotonic() + time.time()
                        )
                        gripper.schedule_waypoint(
                            gripper_target_pos,
                            t_command_target - time.monotonic() + time.time()
                        )

                        # Wait until cycle end
                        precise_wait(t_cycle_end)
                        iter_idx += 1

                except KeyboardInterrupt:
                    print("\n\nKeyboardInterrupt received, stopping...")

                print("\nQuitting...")
                controller.terminate_current_policy()

    finally:
        keyboard.stop()
        print("Control terminated.")


if __name__ == '__main__':
    main()
