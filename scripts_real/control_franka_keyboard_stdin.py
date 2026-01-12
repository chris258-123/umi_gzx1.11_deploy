#!/usr/bin/env python3
"""
Keyboard-controlled Franka robot with native gripper (stdin version).

Controls:
- W/S: Move forward/backward (X axis)
- A/D: Move left/right (Y axis)
- F/V: Move up/down (Z axis)
- I/O: Open/close gripper
- Q: Quit

Usage:
    python control_franka_keyboard_stdin.py --robot_ip 172.16.13.100
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
import termios
import tty
import select


class KeyboardState:
    """Track keyboard state using stdin."""
    def __init__(self):
        self.pressed_keys = set()
        self.quit_flag = False
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def update(self):
        """Check for new key presses (non-blocking)."""
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == 'q':
                self.quit_flag = True
            else:
                self.pressed_keys.add(ch)
                # Auto-release after reading
                if ch in self.pressed_keys:
                    pass  # Keep it pressed for this cycle

    def is_pressed(self, key):
        return key in self.pressed_keys

    def clear_keys(self):
        """Clear pressed keys after processing."""
        self.pressed_keys.clear()

    def should_quit(self):
        return self.quit_flag

    def stop(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


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
    max_gripper_width = 0.08

    print("Starting Franka keyboard control...")
    print("\nControls:")
    print("  W/S: Move forward/backward (X axis)")
    print("  A/D: Move left/right (Y axis)")
    print("  F/V: Move up/down (Z axis)")
    print("  I/O: Open/close gripper")
    print("  Q: Quit")
    print("\nPress keys to control the robot...")

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

                state = controller.get_state()
                target_pose = state['ActualTCPPose'].copy()

                gripper_state = gripper.get_state()
                if gripper_state is not None:
                    gripper_target_pos = gripper_state['gripper_position']
                else:
                    gripper_target_pos = 0.04

                print(f"Initial pose: {target_pose}")
                print(f"Initial gripper width: {gripper_target_pos:.4f}m\n")

                t_start = time.monotonic()
                gripper.restart_put(t_start - time.monotonic() + time.time())

                iter_idx = 0

                try:
                    while not keyboard.should_quit():
                        t_cycle_end = t_start + (iter_idx + 1) * dt
                        t_sample = t_cycle_end - command_latency
                        t_command_target = t_cycle_end + dt

                        precise_wait(t_sample)

                        # Update keyboard state
                        keyboard.update()

                        # Calculate position delta
                        dpos = np.zeros(3)

                        if keyboard.is_pressed('w'):
                            dpos[0] += pos_speed / frequency
                        if keyboard.is_pressed('s'):
                            dpos[0] -= pos_speed / frequency
                        if keyboard.is_pressed('a'):
                            dpos[1] += pos_speed / frequency
                        if keyboard.is_pressed('d'):
                            dpos[1] -= pos_speed / frequency
                        if keyboard.is_pressed('f'):
                            dpos[2] += pos_speed / frequency
                        if keyboard.is_pressed('v'):
                            dpos[2] -= pos_speed / frequency

                        target_pose[:3] += dpos

                        if np.any(dpos != 0):
                            print(f"Moving: dpos={dpos}, new_pose={target_pose[:3]}")

                        # Gripper control
                        gripper_dpos = 0.0
                        if keyboard.is_pressed('i'):
                            gripper_dpos = gripper_speed / frequency
                        if keyboard.is_pressed('o'):
                            gripper_dpos = -gripper_speed / frequency

                        gripper_target_pos = np.clip(
                            gripper_target_pos + gripper_dpos,
                            0.0,
                            max_gripper_width
                        )

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

                        # Clear keys for next cycle
                        keyboard.clear_keys()

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
