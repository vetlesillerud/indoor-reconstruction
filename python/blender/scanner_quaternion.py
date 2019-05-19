import bpy
from bpy import data as D
from bpy import context as C
from mathutils import *
import csv
import random
import blensor
import os
import re
import numpy as np
from shutil import rmtree


def scan_3d_model_fragment(scanner, positions, poses, path):
    """
    Uses given scanner to generate point clouds from given positions with given poses. From each position all poses are
    visited. Writes pcd-files to /tmp/ folder. All scans in one position is equivalent to one fragment
    Args:
        scanner (bpy_types.Object): blender scanner object
        positions (list): list of cartesion coordinated (x, y, x) in blender environment
        poses (list): list of quaternion poses (q_w, q_x, q_y, q_z) in blender environment
        path (str): path to where the point clouds shoule be written

    Returns:
        None
    """
    n = 0
    for position in positions:
        scanner.location = position
        os.mkdir(path + f"{n}")
        for i, pose in enumerate(poses):
            scanner.rotation_quaternion = pose

            filename = path + f"{n}/scan_{i}.pcd"
            blensor.blendodyne.scan_advanced(scanner, rotation_speed=5, simulation_fps=24, angle_resolution=0.1,
                                             max_distance=100,
                                             evd_file=filename, noise_mu=0, noise_sigma=0.0,
                                             start_angle=-135, end_angle=135, evd_last_scan=True,
                                             add_blender_mesh=False, add_noisy_blender_mesh=False)
        n += 1


def scan_3d_model_odometry(scanner, trajectory, poses, path):
    """
    Uses given scanner to generate point clouds from given trajectory with given poses.
    Writes pcd-files to /tmp/ folder. All scans in one trajectory is equivalent to one odometry
    Args:
        scanner (bpy_types.Object): blender scanner object
        positions (list): list of cartesion coordinated (x, y, x) in blender environment
        poses (list): list of quaternion poses (q_w, q_x, q_y, q_z) in blender environment
        path (str): path to where the point clouds shoule be written

    Returns:
        None
    """

    os.mkdir(path)
    i = 0
    for position, pose in zip(trajectory, poses):
        scanner.location = position
        scanner.rotation_quaternion = pose
        filename = path + f"/scan_{i}.pcd"
        blensor.blendodyne.scan_advanced(scanner, rotation_speed=5, simulation_fps=24, angle_resolution=0.1,
                                         max_distance=100,
                                         evd_file=filename, noise_mu=0, noise_sigma=0.0,
                                         start_angle=-135, end_angle=135, evd_last_scan=True,
                                         add_blender_mesh=False, add_noisy_blender_mesh=False)
        i += 1


def write_quaternions_file_fragment(positions, poses, path):
    """
    Writes quaternions_fragment.csv file to given path (fragment folders)
    Args:
        positions (list): list of cartesian coordinates (x, y, z) in blender environment
        poses (list): list of quaternion poses (q_w, q_x, q_y, q_z) in blender environment
        path (str): path to where file should be written

    Returns:
        None
    """
    n = 0
    for position in positions:
        with open(path + f"{n}/quaternions_fragment.csv", mode="w") as f:
            writer = csv.writer(f, delimiter=",")
            writer.writerow(["q_w", "q_x", "q_y", "q_z", "x", "y", "z"])
            for pose in poses:
                writer.writerow(
                    [pose[0], pose[1], pose[2], pose[3], position[0], position[1], position[2]])
        n += 1


def write_quaternions_file_odometry(trajectory, poses, path):
    """
    Writes quaternions_fragment.csv file to given path (odometry folders)
    Args:
        positions (list): list of cartesian coordinates (x, y, z) in blender environment
        poses (list): list of quaternion poses (q_w, q_x, q_y, q_z) in blender environment
        path (str): path to where file should be written

    Returns:
        None
    """
    with open(path + "/quaternions_fragment.csv", mode="w") as f:
        writer = csv.writer(f, delimiter=",")
        writer.writerow(["q_w", "q_x", "q_y", "q_z", "x", "y", "z"])
        for position, pose in zip(trajectory, poses):
            writer.writerow([pose[0], pose[1], pose[2], pose[3], position[0], position[1], position[2]])


if __name__ == '__main__':
    print("\033[32m----- SCRIPT START -----\033[0m")
    # If the scanner is the default camera it can be accessed for example by bpy.data.objects["Camera"]
    scanner = bpy.data.objects["VLP-16"]
    dataset = "/tmp/blender"

    # remove old dataset before generating a new, except a new directory is made
    try:
        rmtree(dataset)
    except:
        os.mkdir(dataset)


    fragmentroot = "/fragments/"
    odometryroot = "/odometry/"
    fragment_path = dataset + fragmentroot + "fragment"
    odometry_path = dataset + odometryroot + "odometry"
    os.makedirs(dataset + fragmentroot)
    os.makedirs(dataset + odometryroot)

    # will scan the 3d model with the given poses in each position
    positions = [(-4.5, -1.54, 1.01)]
    trajectories = []
    d = 0  # determine the resolution of the trajectory
    for i in range(len(positions) - 1):
        trajectory = []
        trajectory_x = np.linspace(positions[i][0], positions[i + 1][0], d)
        trajectory_y = np.linspace(positions[i][1], positions[i + 1][1], d)
        trajectory_z = np.linspace(positions[i][2], positions[i + 1][2], d)
        for j in range(len(trajectory_x)):
            trajectory.append((trajectory_x[j], trajectory_y[j], trajectory_z[j]))
        trajectories.append(trajectory)
    poses = [(random.uniform(0.5, 0.5), random.uniform(0.5, 0.5), random.uniform(0.5, 0.5), random.uniform(0.5, 0.5)) for i in
             range(1)]  # poses visited to generate fragement

    trajectory_poses = [(0.5, 0.5, 0.5, 0.5) for i in range(d)]  # poses along the trajectory

    scan_3d_model_fragment(scanner=scanner, positions=positions, poses=poses, path=fragment_path)
    write_quaternions_file_fragment(positions=positions, poses=poses, path=fragment_path)
    for i, trajectory in enumerate(trajectories):
        scan_3d_model_odometry(scanner=scanner, trajectory=trajectory, poses=trajectory_poses,
                               path=odometry_path + f"{i}")
        write_quaternions_file_odometry(trajectory=trajectory, poses=trajectory_poses, path=odometry_path + f"{i}")
    print("\033[32m----- SCRIPT END -----\033[0m")
