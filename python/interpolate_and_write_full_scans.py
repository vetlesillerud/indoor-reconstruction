from multiway_registration import combine_point_clouds, find_paths, \
    make_transformation_matrices, read_quaternions_file, load_point_clouds
import json
import numpy as np
import csv
import copy
from open3d import *
from open3d.open3d.geometry import *
from open3d.open3d.registration import *
from open3d.open3d.utility import *
from open3d.open3d.visualization import *
import argparse

def parse_args():
    """
    Parses command line arguments
    Returns:
        tuple: (visualization, directory)
            visualization (bool): True if generated cloud should be visualized
            directory (str): string representing specific path to datapacket clouds that should be interpolated
    """
    parser = argparse.ArgumentParser(
        description="Reads datapacket pcds, interpolates quaternions and generates scans from dataset in config file")
    parser.add_argument("--visualization", "-v", action="store_true", help="if generated clouds should be visualized")
    parser.add_argument("--directory", "-d",
                        help="if only specified directory should be interpolated, e.g. 'fragments/fragment0'")
    args = parser.parse_args()
    return args.visualization, args.directory


def interpolate_quaternions(datapacket_quaternions, datapacket_timestamps, path):
    """
    Interpolates between quaternions that has the same value at different timestamps.
    Interpolation by drawing a straight line between quaternions that are equal, but has different timestamps.
    Args:
        datapacket_quaternions (list): list of quaternions
        datapacket_timestamps (list): list of timestamps
        path (str): string representing the path to where the quaternions file are located

    Returns:
        list: list of interpolated quaternions
    """
    print("Interpolating: " + path + "quaternions_datapacket.csv")
    previous_quaternion = np.asarray([1000, 1000, 1000, 1000])  # initialization of variable
    previous_time = 0
    interpolated_quaternions = []
    timestamp = []
    equal_quaternions = []
    for quat, time in zip(datapacket_quaternions, datapacket_timestamps):
        if np.array_equal(previous_quaternion, quat):
            if not equal_quaternions:
                equal_quaternions.append(previous_quaternion)
                interpolated_quaternions.pop()
                timestamp.append(previous_time)
            equal_quaternions.append(quat)
            timestamp.append(time)

        else:
            if equal_quaternions:
                diff_quat = quat - previous_quaternion
                diff_time = time - timestamp[0]
                a = diff_quat / diff_time  # slope
                b = equal_quaternions[0] - a * timestamp[0]  # y-intercept
                for t in timestamp:
                    q = a * t + b  # y = ax + b
                    interpolated_quaternions.append(q)
                interpolated_quaternions.append(quat)
                equal_quaternions = []
                timestamp = []
                previous_quaternion = quat[:]
                previous_time = time

            else:
                interpolated_quaternions.append(quat)
                previous_quaternion = quat[:]
                previous_time = time
    if equal_quaternions:
        # appending the last quaternions if they are all equal
        for quat in equal_quaternions:
            interpolated_quaternions.append(quat)
    return interpolated_quaternions


def write_interpolated_quaternions(interpolated_quaternions, timestamps, path):
    """
    Writes interpolated quaternions to csv file
    Args:
        interpolated_quaternions (list): list of interpolated quaternions
        timestamps (list): list of timestamps
        path (str): string describing the path where file should be written
    Returns:
        None
    """
    filename = "quaternions_interpolated.csv"
    data = [(q[0], q[1], q[2], q[3], t) for q, t in zip(interpolated_quaternions, timestamps)]
    with open(path + filename, "w+") as csvfile:
        header = [("q_w", "q_x", "q_y", "q_z", "t")]
        content = header + data
        writer = csv.writer(csvfile)
        print("Writing: " + path + filename)
        writer.writerows(content)


def combine_datapackets_to_full_scans_and_write_files(number_of_datapackets_in_scans, datapacket_clouds, quaternions,
                                                     positions, path):
    """
    Combines datapackets to full scans and writes these scans to file. Transforms datapackets using given quaternions.
    Args:
        number_of_datapackets_in_scans (list): list of numbers of datapackets in each scan
        datapacket_clouds (list): list of point cloud datapackets
        quaternions (list): list of quaternions belonging to the datapackets
        positions (list): list of positions belonging to the datapackets
        path (str): string representing the path to datapacket directory

    Returns:
        list: list of datapackets combined into scans after transformation
    """
    quaternion_filename = "quaternions_fragment.csv"
    combined_clouds = []
    with open(path + quaternion_filename, "w+") as csvfile:
        writer = csv.writer(csvfile)
        print("Writing: " + path + quaternion_filename)
        header = ("q_w", "q_x", "q_y", "q_z")
        writer.writerow(header)
        last_number = 0
        scans = []
        quats = []
        pos = []
        current_number = 0
        for number in number_of_datapackets_in_scans:
            # slicing based on the number of datapackets in scan
            current_number += number
            scans.append(datapacket_clouds[last_number:current_number])
            quats.append(quaternions[last_number:current_number])
            pos.append(positions[last_number:current_number])
            last_number += number
        for i, (q, p, scan) in enumerate(zip(quats, pos, scans)):
            transformation_matrices = make_transformation_matrices(np.asarray(q), np.asarray(p), interpolated=False)
            scan_copy = copy.deepcopy(scan)
            matrix = np.identity(4)
            for element, rel_trans in zip(scan_copy, transformation_matrices):
                matrix = np.matmul(matrix, rel_trans)  # multiplying relative rotations to get total rotation
                element.transform(matrix)
            scan_combined = combine_point_clouds(scan_copy)
            combined_clouds.append(scan_combined)
            write_point_cloud(filename=path + f"scan{i}.pcd", pointcloud=scan_combined)
            writer.writerow((1, 0, 0, 0))  # clouds are transformed to global space
        print("PCD-files written to: " + path)
    return combined_clouds


def visualize_clouds(clouds, paths):
    """
    Visualizes given point clouds.
    Args:
        clouds (list): list of point cloud objects
        paths (list): list of paths to where the point cloud was generated from

    Returns:
        None
    """

    for cloud, path in zip(clouds, paths):
        print("Visualizing scans generated from:", path)
        for scan in cloud:
            draw_geometries([scan])


def interpolate_quaternions_and_write_scans(path):
    """
    Interpolates quaternions and combines datapackets to scans. Returns these scans.
    Args:
        path (str): string representing the path to datapacket folder
    Returns:
        list: list of scans (combined datapackets)
    """
    element = "datapacket"
    datapacket_quaternions, datapacket_positions, datapacket_timestamps = read_quaternions_file(path, element=element)
    datapacket_quaternions_interpolated = interpolate_quaternions(datapacket_quaternions, datapacket_timestamps, path)
    write_interpolated_quaternions(datapacket_quaternions_interpolated, datapacket_timestamps, path)
    clouds, _, number_of_datapackets_in_scans = load_point_clouds(path, element, config[element + "_voxel_size"])
    combined_clouds = combine_datapackets_to_full_scans_and_write_files(number_of_datapackets_in_scans, clouds,
                                                                       datapacket_quaternions_interpolated,
                                                                       datapacket_positions, path)
    return combined_clouds


if __name__ == '__main__':
    # loading config file
    with open("config.json") as f:
        config = json.load(f)

    # parsing command line arguments
    visualization, directory = parse_args()

    if directory:  # if directory is specified in command line
        path = "../data/" + config["dataset_name"] + "/" + directory + "/"

        combined_clouds = interpolate_quaternions_and_write_scans(path)

        if visualization:  # if cloud should be visualized
            visualize_clouds([combined_clouds], [path])
    else:  # find all paths in dataset and generate scans
        fragment_paths, _, odometry_paths, _ = find_paths(config["dataset_name"])
        fragment_combined_clouds = []
        for f_path in fragment_paths:
            f_combined_clouds = interpolate_quaternions_and_write_scans(f_path)
            fragment_combined_clouds.append(f_combined_clouds)

        odometry_combined_clouds = []
        for o_path in odometry_paths:
            o_combined_clouds = interpolate_quaternions_and_write_scans(o_path)
            odometry_combined_clouds.append(o_combined_clouds)

        if visualization:  # if cloud should be visualized
            visualize_clouds(fragment_combined_clouds, fragment_paths)
            visualize_clouds(odometry_combined_clouds, odometry_paths)
