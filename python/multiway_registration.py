import numpy as np
from open3d import *
from open3d.open3d.geometry import *
from open3d.open3d.registration import *
from open3d.open3d.utility import *
from open3d.open3d.visualization import *
import copy
import random
import pandas as pd
import os, re
from termcolor import colored
import argparse
from quaternions import make_rotation_matrix_from_quaternion, conjugate_quaternion, normalize_quaternion, \
    multiply_quaternions
import json

with open("config.json") as f:
    config = json.load(f)


def parse_args():
    """
    Parses command line arguments
    Returns:
        tuple: (icp, visualization, verbose, odometry, generate_fragments, blensor, trajectory)
            icp (str): path representing which element inside dataset to register, eg. fragments/fragment0
            visualization (str): path representing which element inside dataset to visualize, eg. fragment/fragment0
            verbose (bool): True if Open3D VerbosityLevel.Debug should be activated
            odometry (bool): True if odometry should be estimated and fragments should be registered
            generate (bool): True if both odometry and fragments in should be generated before registering fragments.
                            False if previously generated pose graphs and clouds should be used to register fragments.
            blensor (bool): True if scans generated in blensor should be used
            trajectory (bool): True if registered clouds should be visualized with trajectory
    """
    parser = argparse.ArgumentParser(
        description="Multiway registration script for point cloud data. Supported point cloud format: .pcd")
    parser.add_argument("--icp", "-i",
                        help="if icp and optimization should be done on scans in specified folder of dataset. "
                             "Uses dataset specified in config file. Use: -i odometry/odometryX")
    parser.add_argument("--visualization", "-v",
                        help="if all scans in specified folder should be visualized using only transform from IMU data."
                             "Uses dataset specified in config file. Use: -v fragments/fragmentX")
    parser.add_argument("--verbose", "-vb", help="if open3d.VerbosityLevel.Debug should be activated",
                        action="store_true")
    parser.add_argument("--odometry", "-o", action="store_true",
                        help="if odometry should be estimated and fragments should be registered")
    parser.add_argument("--generate", "-g",
                        help="True if both odometry and fragments in should be generated before registering fragments."
                             " False if previously generated pose graphs and clouds should be used to register fragments.",
                        action="store_true")
    parser.add_argument("--blensor", "-b", help="True if scans generated in BlenSor should be used",
                        action="store_true")
    parser.add_argument("--trajectory", "-t", help="True if registered cloud should be visualized with trajectory",
                        action="store_true")
    args = parser.parse_args()
    return args.icp, args.visualization, args.verbose, args.odometry, args.generate, args.blensor, args.trajectory


def print_progress_bar(iteration, total, prefix='', suffix='', decimals=1, length=100, fill='█'):
    """
    Loop to create terminal progress bar
    Args:
        iteration (int): current iteration
        total (int): current iteration
        prefix (str): prefix string
        suffix (str): suffix string
        decimals (int): positive number of decimals in percent complete
        length (int): character length of bar
        fill (str): bar fill character

    Returns:
        None
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)
    print('\r%s |%s| %s%% %s' % (prefix, bar, percent, suffix), end='\r')
    # Print New Line on Complete
    if iteration == total:
        print()


def find_number_of_pcd_files(pattern, path):
    """
    Find the number of PCD files with given pattern in given path
    Args:
        pattern (str): pattern of the PCD filename
        path (str): path where PCD files are located

    Returns:
        int: integer representing the number of PCD files found
    """
    matches = []
    p = re.compile(pattern)
    for root, dirs, files in os.walk(path):
        for file in files:
            match = p.search(file)
            if match:
                matches.append(match.group(1))
    number_of_unique_matches = len(set(matches))
    return number_of_unique_matches


def clamp(v):
    """
    Clamps given value to 0 if value below 0, and 1 if value above 1.
    Args:
        v (float): value to clamp to range [0, 1]

    Returns:
        float: clamped value
    """
    t = 0 if v < 0 else v
    return 1 if t > 1 else t


def color_with_intensity(cloud, filename):
    """
    Colors given point cloud with belonging intensity value if intensity is in PCD header.
    Args:
        cloud (open3d.geometry.PointCloud): point cloud object to be colored
        filename (str): filename of point cloud object

    Returns:
        open3d.geometry.PointCloud: colored point cloud object
    """
    try:
        with open(filename) as pcdfile:
            intensity_header = re.compile(r"intensity")
            content = pcdfile.read()
            intensity_match = intensity_header.search(content)

        if intensity_match:  # if intensity in file, color by this value
            cloud.paint_uniform_color([0.5, 0.5, 0.5])
            pattern = re.compile(r"(\d{1,3}$)")
            for j, line in enumerate(open(filename)):
                if j >= 11:  # skip header
                    match = pattern.search(line)
                    if match:
                        intensity = match.group(1)
                        intensity_range_value = -1 + float(intensity) / 75  # colormap from -1 to 1
                        red = clamp(1.5 - abs(2 * intensity_range_value - 1))
                        green = clamp(1.5 - abs(2 * intensity_range_value))
                        blue = clamp(1.5 - abs(2 * intensity_range_value + 1))
                        cloud.colors[j - 11] = [red, green, blue]
                    else:
                        print("No intensity data found")
    except UnicodeDecodeError:  # if the point cloud data is binary
        pass
    return cloud


def load_full_scans(path, element, voxel_size, blensor):
    """
    Loads full scan PCD files from given path
    Args:
        path (str): path where scan PCD files are located
        element (str): type of element to be loaded (datapacket, scan, fragment or odometry)
        voxel_size (float): voxel downsample size
        blensor (bool): True if scans generated in BlenSor should be loaded
    Returns:
        tuple: (clouds, outlier_clouds)
            clouds (list): list of point cloud objects
            outlier_clouds (list): list of outlier point cloud objects
    """
    clouds = []
    outlier_clouds = []
    print("\nSearching:", path)
    if blensor:  # BlenSor generates a different filename
        n = find_number_of_pcd_files(pattern="(^scan_\d\d\d\d\d\d\d?\.pcd)", path=path)
    else:
        n = find_number_of_pcd_files(pattern="(^scan\d+)", path=path)
    print(n, "PCD-files found!")
    print_progress_bar(0, n, prefix="Reading PCD-files:", suffix="Complete", length=50)

    for i in range(n):
        if blensor:
            filename = path + f"scan_{i}00000.pcd"
        else:
            filename = path + f"scan{i}.pcd"
        cloud = read_point_cloud(filename)
        cloud = color_with_intensity(cloud, filename)
        cloud, outliers = outlier_removal(cloud, element=element)
        outlier_clouds.append(outliers)
        if element != "datapacket" and voxel_size:
            cloud = voxel_down_sample(cloud, voxel_size)

        estimate_normals(cloud, KDTreeSearchParamHybrid(radius=config["estimate_normals"]["radius"],
                                                        max_nn=config["estimate_normals"]["max_nn"]))
        orient_normals_towards_camera_location(cloud, camera_location=np.array([0, 0, 0]))
        clouds.append(cloud)
        print_progress_bar(i + 1, n, prefix="Reading PCD-files:", suffix="Complete", length=50)

    return clouds, outlier_clouds


def load_data_blocks(path):
    """
    Loads data packet PCD files from given path
    Args:
        path (str): path to where datapacket PCD files are located

    Returns:
        tuple: (clouds, outlier_clouds, number_of_datapackets_in_scan),
            clouds (list): list of point cloud objects
            outlier_clouds (list): list of outlier point cloud objects
            number_of_datapackets_in_scan (list): list of ints specifiying how many datapackets are the different scans
    """
    clouds = []
    outlier_clouds = []
    print("Searching:", path)
    n_total = find_number_of_pcd_files(pattern="(^scan_\d+_\d+.pcd)", path=path)
    print(f"PCD-files found: {n_total}")
    if n_total == 0:
        return [], []  # if none found
    n_scans = find_number_of_pcd_files(pattern="(^scan_\d+)", path=path)
    print(f"Full scans: {n_scans}")
    number_of_datapackets_in_scan = []
    for n in range(n_scans):
        n_datapackets = find_number_of_pcd_files(pattern=f"(^scan_{n}_\d+)", path=path)
        number_of_datapackets_in_scan.append(n_datapackets)

    print_progress_bar(0, n_total, prefix="Reading PCD-files:", suffix="Complete", length=50)
    k = 0
    for scan, datapacket in zip(range(n_total), number_of_datapackets_in_scan):
        for db in range(datapacket):
            filename = path + f"scan_{scan}_{db}.pcd"
            cloud = read_point_cloud(filename)
            cloud = color_with_intensity(cloud, filename)
            clouds.append(cloud)
            print_progress_bar(k + 1, n_total, prefix="Reading PCD-files:", suffix="Complete", length=50)
            k += 1
    return clouds, outlier_clouds, number_of_datapackets_in_scan


def load_point_clouds(path, element, voxel_size, blensor=False):
    """
    Loads point clouds from given path. Downsamples and filter outliers if specified in config. Colors by intensity
    color if available
    Args:
        path (str): path where PCD files are located
        element (str): type of element to be loaded (datapacket, scan, fragment or odometry)
        voxel_size (float or bool): voxel size in use for downsampling. False if no downsampling should be done
        blensor (bool): True if point clouds located in end of path are generated using BlenSor (default: False)

    Returns:
        tuple: (clouds, outlier_clouds, number_of_datapackets_in_scan)
            clouds (list): list of point cloud objects, without outliers
            outlier_clouds (list): list of belongning outliers
            number_of_datapackets_in_scan (list): only if element="datapacket", else empty list. List specifiying how many
                                                datapackets are contained in the different scans
    """
    if element == "datapacket":
        clouds, outlier_clouds, number_of_datapackets_in_scan = load_data_blocks(path)
        return clouds, outlier_clouds, number_of_datapackets_in_scan

    else:
        clouds, outlier_clouds = load_full_scans(path, element, voxel_size, blensor)
        return clouds, outlier_clouds, []  # full scans are not divided into datapackets, hence empty list


def outlier_removal(cloud, element):
    """
    Removes outliers from given point cloud. Either statistical or radius removal.
    Args:
        cloud (open3d.geometry.PointCloud): point cloud object
        element (str): type of element to be processed (datapacket, scan, fragment or odometry)
    Returns:
        tuple: (inlier_cloud, outlier_cloud)
            inlier_cloud (open3d.geometry.PointCloud): point cloud object without outliers
            outlier_cloud (open3d.geometry.PointCloud) point cloud object of the outliers
    """
    # Statistical outlier removal
    if config[element + "_outlier_removal"] == "statistical":
        cl, ind = statistical_outlier_removal(cloud,
                                              nb_neighbors=config[element + "_statistical"]["nb_neighbors"],
                                              std_ratio=config[element + "_statistical"]["std_ratio"])
    # Radius outlier removal
    elif config[element + "_outlier_removal"] == "radius":
        cl, ind = radius_outlier_removal(cloud,
                                         nb_points=config[element + "_radius"]["nb_points"],
                                         radius=config[element + "_radius"]["radius"])
    # No outlier removal
    else:
        return cloud, select_down_sample(cloud, [])
    # Making a object cloud of inliers
    inlier_cloud = select_down_sample(cloud, ind)
    # Making a object cloud of outliers, the invert of inliers
    outlier_cloud = select_down_sample(cloud, ind, invert=True)
    # Paint outliers red
    outlier_cloud.paint_uniform_color([1, 0, 0])
    return inlier_cloud, outlier_cloud


def pairwise_registration(source, target, transformation_matrix, element):
    """
    Performs pairwise registration between source and target point clouds using a variant of the
    Iterative Closest Point algorithm. Variant is specified in config file, and dependent upon element type
    Args:
        source (open3d.geometry.PointCloud): source point cloud object
        target (open3d.geometry.PointCloud): target point cloud object
        transformation_matrix (numpy.ndarray): initial alignment transformation matrix
        element (str): type of elements to be registered (datapacket, scan, fragment or odometry)

    Returns:
        tuple: (transformation_icp, information_icp)
            transformation_icp (numpy.ndarray): transformation matrix after ICP has been performed
            information_icp (numpy.ndarray): matrix of RMSE between correspondences in source and target after ICP,
                                            with a line process weight
    """
    method = config["local_icp_method_" + element]
    max_correspondence_distance = config[element]["icp"][method]["max_correspondence_distance"]
    if method == "point-to-point":
        icp = registration_icp(source=source, target=target, init=transformation_matrix,
                               max_correspondence_distance=config[element]["icp"][method][
                                   "max_correspondence_distance"],
                               estimation_method=TransformationEstimationPointToPoint(),
                               criteria=ICPConvergenceCriteria(
                                   max_iteration=config[element]["icp"][method]["max_iteration"],
                                   relative_fitness=config[element]["icp"][method]["relative_fitness"],
                                   relative_rmse=config[element]["icp"][method]["relative_rmse"]))
    elif method == "point-to-plane":
        icp = registration_icp(source=source, target=target, init=transformation_matrix,
                               max_correspondence_distance=config[element]["icp"][method][
                                   "max_correspondence_distance"],
                               estimation_method=TransformationEstimationPointToPoint(),
                               criteria=ICPConvergenceCriteria(
                                   max_iteration=config[element]["icp"][method]["max_iteration"],
                                   relative_fitness=config[element]["icp"][method]["relative_fitness"],
                                   relative_rmse=config[element]["icp"][method]["relative_rmse"]))
    elif method == "color":
        icp = registration_colored_icp(source=source, target=target, init=transformation_matrix,
                                       max_correspondence_distance=config[element]["icp"][method][
                                           "max_correspondence_distance"],
                                       lambda_geometric=config[element]["icp"][method]["lambda_geometric"],
                                       criteria=ICPConvergenceCriteria(
                                           max_iteration=config[element]["icp"][method]["max_iteration"],
                                           relative_fitness=config[element]["icp"][method]["relative_fitness"],
                                           relative_rmse=config[element]["icp"][method]["relative_rmse"]))
    transformation_icp = icp.transformation
    information_icp = get_information_matrix_from_point_clouds(source, target, max_correspondence_distance,
                                                               icp.transformation)
    return transformation_icp, information_icp


def full_registration(clouds, relative_transformation_matrices, verbose, element):
    """
    Performs full registration between all point clouds by using ICP in a pairwise manner to generate a pose graph.
    ICP variant is specified in config and dependent upon element type
    Args:
        clouds (list): list of point cloud objects
        relative_transformation_matrices (list): list of ndarrays of initial relative transformation between each point cloud
        verbose (bool): True if open3d VerboseLevel.Debug is activated
        element (str): type of elements to be registered (datapacket, scan, fragment or odometry)

    Returns:
        open3d.registration.PoseGraph: generated pose graph
    """
    pose_graph = PoseGraph()
    odometry = relative_transformation_matrices[0]
    pose_graph.nodes.append(PoseGraphNode(odometry))  # referenced to global space
    n_clouds = len(clouds)
    total_iterations = int(((n_clouds - 1) * n_clouds) / 2)

    # variables to display information of the current process
    method = config["local_icp_method_" + element]
    if method == "color":
        lambda_geometric = config[element]["icp"][method]["lambda_geometric"]
    else:
        lambda_geometric = "N/A"
    max_corr_dist = config[element]["icp"][method]["max_correspondence_distance"]
    max_iteration = config[element]["icp"][method]["max_iteration"]
    relative_fitness = config[element]["icp"][method]["relative_fitness"]
    relative_rmse = config[element]["icp"][method]["relative_rmse"]
    print("-- PARAMETERS --")
    print("Registration element:", element)
    print("ICP type:", method)
    print("lambda_geometric:", lambda_geometric)
    print("max_corr_dist:", max_corr_dist)
    print("max_iteration:", max_iteration)
    print("relative_fitness:", relative_fitness)
    print("relative_rmse:", relative_rmse)

    if not verbose:
        print_progress_bar(0, total_iterations, prefix="Registering point clouds:", suffix="Complete", length=50)

    i = 0
    for source_id in range(n_clouds):
        if verbose:
            print(colored(f"Point cloud {source_id + 1}/{n_clouds}", "yellow"))

        matrix = np.identity(4)

        for target_id in range(source_id + 1, n_clouds):
            if verbose:
                print(f"Registration {target_id - source_id}/{n_clouds - (source_id + 1)}")

            matrix = np.matmul(matrix,
                               relative_transformation_matrices[target_id])  # multiplying relative transformations
            transform = np.linalg.inv(matrix)  # transformation that aligns source point cloud to target point cloud
            transformation_icp, information_icp = pairwise_registration(clouds[source_id], clouds[target_id], transform,
                                                                        element)

            if target_id == source_id + 1:  # odometry case, hence uncertain=False
                odometry = np.matmul(odometry,
                                     transformation_icp)  # multiplying relative rotations to get total transformation
                pose_graph.nodes.append(PoseGraphNode(odometry))
                pose_graph.edges.append(
                    PoseGraphEdge(source_id, target_id, transformation_icp, information_icp, uncertain=False))
            else:  # loop closure case, hence, uncertain=True
                pose_graph.edges.append(
                    PoseGraphEdge(source_id, target_id, transformation_icp, information_icp, uncertain=True))
            if not verbose:
                print_progress_bar(i + 1, total_iterations, prefix="Registering point clouds:", suffix="Complete",
                                   length=50)
            i += 1

    return pose_graph


def transform_and_visualize_point_clouds(clouds, outliers, relative_transformation, path):
    """
    Transform and visualize point clouds in global space with given transformations. Writes this combined cloud to file.
    Args:
        clouds (list): list of point cloud objects
        outliers (list): list of outlier point cloud objects
        relative_transformation (list): list of numpy.ndarrays containing relative transformation between point clouds
        path (str): path to where the combined point cloud should be written

    Returns:
        None
    """

    clouds_copy = copy.deepcopy(clouds)
    outliers_copy = copy.deepcopy(outliers)
    matrix = np.identity(4)
    for cloud, rel_trans in zip(clouds_copy, relative_transformation):
        matrix = np.matmul(matrix, rel_trans)  # multiplying relative rotations to get total rotation
        cloud.transform(matrix)

    matrix = np.identity(4)
    for outlier_cloud, rel_trans in zip(outliers_copy, relative_transformation):
        matrix = np.matmul(matrix, rel_trans)  # multiplying relative rotations to get total rotation
        outlier_cloud.transform(matrix)

    cloud_combined = combine_point_clouds(clouds_copy)
    write_read_and_draw_combined_geometries(cloud_combined, "combined_cloud.pcd", path)


def make_translation_matrix(target_position, current_position, tripod_transformation, previous_tripod_position,
                            interpolated):
    """
    Creates one column vector of position resulting from tripod rotation and one column vector of relative translation
    in global space.
    Args:
        target_position (tuple): target position in global space
        current_position (tuple): current position in global space
        tripod_transformation (numpy.ndarray): transformation that transforms the offset from tripod joint to global space
        previous_tripod_position (numpy.ndarray): previous position of the LiDAR as a result of tripod rotation
        interpolated (bool): True if point clouds have been transformed with tripod translation in interpolation procedure

    Returns:
        tuple: (tripod_positon, relative_translation)
            tripod_position (numpy.ndarray): position of the LiDAR as a result of tripod rotation
            relative_translation (numpy.ndarray): translation in global space
    """
    if not interpolated:
        tripod_position = np.matmul(tripod_transformation, np.asarray([[0],
                                                                       [0],
                                                                       [0.101]]))  # LiDAR is located 0.101m from tripod joint
    else:
        tripod_position = np.matmul(tripod_transformation, np.asarray([[0],
                                                                       [0],
                                                                       [0]]))  # data is already transformed in interpolation

    x, y, z = np.asarray(target_position) - np.asarray(current_position)
    environment_translation = np.asarray([[x],
                                          [y],
                                          [z]])
    relative_translation = tripod_position - previous_tripod_position + environment_translation
    return tripod_position, relative_translation


def read_quaternions_file(path, element, blensor=False):
    """
    Reads quaternions.csv file located in given path
    Args:
        path (str): path where quaternions file is located
        element (str): what type of element that belongs to the quaternions (datapacket, scan, fragment or odometry)
        blensor (bool): True if point clouds in path are generated using BlenSor (default: False)

    Returns:
        tuple: (quaternions, positions, timestamps)
            quaternions (numpy.ndarray): array of the quaternions collected together with the point clouds
            positions (numpy.ndarray): array of the position of scan
            timestamps (list): list of timestamps if element is datapacket, else: empty list
    """
    if element == "datapacket":
        quat_path = path + "quaternions_datapacket.csv"
    else:
        quat_path = path + "quaternions_fragment.csv"
    print("Reading:", quat_path)
    df = pd.read_csv(filepath_or_buffer=quat_path)
    quaternions = []
    timestamps = []
    for _, row in df.iterrows():
        quaternions.append((row["q_w"], row["q_x"], row["q_y"], row["q_z"]))
    if element == "datapacket":  # datapacket quaternions file has a time associated with a quaternion
        for index, row in df.iterrows():
            timestamps.append(row["t"])
    if blensor:  # position is known in BlenSor -> quaternions file will have x,y,z columns
        positions = []
        for _, row in df.iterrows():
            positions.append((row["x"], row["y"], row["z"]))
    else:
        positions = [(0, 0, 0) for _ in quaternions]  # if unknown position
    return np.asarray(quaternions), np.asarray(positions), timestamps


def make_transformation_matrices(quaternions, positions, interpolated=True):
    """
    Creates transformation matrices from given quaternions and positions
    Args:
        quaternions (numpy.ndarray): quaternions collected together with the point clouds
        positions (numpy.ndarray): positions of the point clouds in global space
        interpolated (bool): True if point clouds have been transformed with tripod translation in interpolation procedure
                        (default: True)

    Returns:
        list: list of numpy.ndarrays of relative transformation between each set of quaternions and positions
    """
    relative_matrices = []
    translation_transform = []  # list with transforms, in order to transform translation to belonging frame
    for n, _ in enumerate(quaternions):
        if n == 0:  # initalizations
            relative_rotation, tripod_transformation = make_rotation_matrix(quaternions[n], quaternions[n], n)
            translation_transform.append(np.matmul(np.identity(3), relative_rotation))
            tripod_position, relative_translation = make_translation_matrix(positions[n], positions[n],
                                                                            tripod_transformation, np.asarray([[0],
                                                                                                               [0],
                                                                                                               [0]]),
                                                                            interpolated)
        else:
            relative_rotation, tripod_transformation = make_rotation_matrix(quaternions[n], quaternions[n - 1], n)
            translation_transform.append(np.matmul(translation_transform[n - 1], relative_rotation))
            tripod_position, relative_translation = make_translation_matrix(positions[n], positions[n - 1],
                                                                            tripod_transformation, tripod_position,
                                                                            interpolated)
            relative_translation = np.matmul(np.linalg.inv(translation_transform[n - 1]),
                                             relative_translation)  # transforming translation to belonging frame
        relative_matrix = np.concatenate((relative_rotation, relative_translation), axis=1)
        relative_matrix = np.vstack((relative_matrix, [0, 0, 0, 1]))  # creating homogeneous matrix
        relative_matrices.append(relative_matrix)

    return relative_matrices


def make_rotation_matrix(target_quaternion, current_quaternion, n):
    """
    Creates rotation matrices from given quaternions.
    Args:
        target_quaternion (numpy.ndarray): target quaternion
        current_quaternion (numpy.ndarray): current quaterion
        n (int): integer describing which is the current iteration in loop

    Returns:
        tuple: (relative_rotation_matrix, tripod_rotation)
            relative_rotation_matrix (numpy.ndarray): rotation matrix from current to target quaternion
            tripod_rotation (numpy.ndarray): transformation that transforms the offset from tripod joint to global space
    """
    if n == 0:  # rotation to global space
        q_current = current_quaternion
        q_current_normalized = normalize_quaternion(q_current)
        relative_rotation_matrix = make_rotation_matrix_from_quaternion(q_current_normalized)
        tripod_rotation = relative_rotation_matrix
    else:  # relative rotations
        q_target = target_quaternion
        q_current = current_quaternion
        q_current_normalized = normalize_quaternion(q_current)
        q_target_normalized = normalize_quaternion(q_target)
        tripod_rotation = make_rotation_matrix_from_quaternion(q_target_normalized)  # global orientation
        q_current_normalized_inv = conjugate_quaternion(q_current_normalized)
        q_delta = multiply_quaternions(q_current_normalized_inv, q_target_normalized)
        q_delta_normalized = normalize_quaternion(q_delta)
        relative_rotation_matrix = make_rotation_matrix_from_quaternion(q_delta_normalized)
    return relative_rotation_matrix, tripod_rotation


def read_quaternions_from_paths(paths, element, blensor):
    """
    Reads quaternions file found in given paths
    Args:
        paths (list): list of paths where quaternions file are located
        element (str): type of elements to be loaded (datapacket, scan, fragment or odometry)
        blensor (bool): True if point clouds located in end of paths are generated using BlenSor

    Returns:
        tuple: (quaternions, positions, timestamps)
            quaternions (list): list of lists of quaternions
            postions (list): list of lists of positions
            timestamps (list): list of lists of timestamps
    """
    quaternions = []
    positions = []
    timestamps = []
    for path in paths:
        quaternion, position, timestamp = read_quaternions_file(path, element, blensor)
        quaternions.append(quaternion)
        positions.append(position)
        timestamps.append(timestamp)
    return quaternions, positions, timestamps


def load_pose_graphs(paths):
    """
    Reads pose graphs named pose_graph.json from given paths.
    Args:
        paths(list): list of paths where pose graph file are located

    Returns:
        list: list of pose graphs
    """
    pose_graphs = []
    for path in paths:
        print("Reading: " + path + "pose_graph.json")
        pose_graph = read_pose_graph(filename=path + "pose_graph.json")
        pose_graphs.append(pose_graph)

    return pose_graphs


def get_estimated_translation(pose_graphs, quaternions):
    """
    Reads the estimated total translation from pose graph
    Args:
        pose_graphs (list): list of pose graphs
        quaternions (list): list of lists quaternions

    Returns:
        list: list of numpy.arrays with the total estimated translation between pose graphs
    """
    estimated_translations = []
    total_translation = np.asarray([0, 0, 0], dtype=np.float64)
    for pose_graph, quaternion in zip(pose_graphs, quaternions):
        estimated_transformation = pose_graph.nodes[
            len(quaternion) - 2].pose  # second last translation tend to be correct
        estimated_translation = copy.deepcopy(estimated_transformation[:3, 3])
        total_translation += estimated_translation
        estimated_translations.append(copy.deepcopy(total_translation))
    return estimated_translations


def load_fragments(fragment_paths, estimated_translations, voxel_down_size, combined_cloud):
    """
    Reads fragment point clouds and quaternions file from given paths. Uses estimated translations to generate positions
    for fragments in global space
    Args:
        fragment_paths (list): list of paths where fragment point clouds are located
        estimated_translations (list): list of fragment positions
        voxel_down_size (float or bool): required voxel size after downsampling. False if no downsampling should be done
        combined_cloud (bool): True if cloud generated only form IMU transform should be read, instead of registered cloud

    Returns:
        tuple: (clouds, outliers, quatenions, positions)
            clouds (list): list of point cloud objects
            outliers (list): list of outlier point cloud objects
            quaternions (list): list of quaternions
            positions (list): list of positions
    """
    clouds = []
    outliers = []
    quaternions = []
    positions = [np.asarray([0, 0, 0], dtype=np.float64)]
    for translation in estimated_translations:
        translation[2] = 0  # assuming LiDAR does not move in z-direction
        positions.append(translation)

    for f_path in fragment_paths:
        if combined_cloud:
            pcdfile = f_path + "combined_cloud.pcd"
        else:
            pcdfile = f_path + "multiway_registration.pcd"
        print("Reading:", pcdfile)
        cloud = read_point_cloud(pcdfile)
        if voxel_down_size:
            cloud = voxel_down_sample(cloud, voxel_down_size)
        cloud, outlier = outlier_removal(cloud, element="fragment")
        outliers.append(outlier)
        clouds.append(cloud)
        quaternion = (1, 0, 0, 0)  # combined_cloud and multiway_registration are already transformed to global space
        quaternions.append(quaternion)

    return clouds, outliers, quaternions, positions


def load_files_and_register_clouds(path, element, quaternions, positions, blensor):
    """
    Loads point clouds from given path and registers these according to the method specified for the element.
    Writes pose graph and point cloud to file.
    Args:
        path (str): path where the point clouds are located
        element (str): what type of element to be registered (datapacket, scan, fragment or odometry)
        quaternions (list): list of quaternion orientations collected together with the point clouds
        positions (list): list of positions where the point clouds are collected
        blensor (bool): True if point clouds in path are generated using BlenSor

    Returns:
        None
    """
    clouds, outliers, _ = load_point_clouds(path, element, voxel_size=config[element + "_voxel_size"], blensor=blensor)
    transformation_matrices = make_transformation_matrices(quaternions, positions)
    pose_graph = full_registration(clouds=clouds, relative_transformation_matrices=transformation_matrices,
                                   verbose=False, element=element)
    method = config["local_icp_method_" + element]
    optimized_pose_graph = optimize_pose_graph(pose_graph=pose_graph,
                                               max_correspondence_distance=config[element]["icp"][method][
                                                   "max_correspondence_distance"],
                                               edge_prune_threshold=config["pose_graph_optimization"][
                                                   "edge_prune_threshold"])
    transformed_clouds = transform_points(clouds=clouds, pose_graph=pose_graph)
    combined_cloud = combine_point_clouds(clouds=transformed_clouds)
    write_pose_graph(filename=path + "pose_graph.json", pose_graph=optimized_pose_graph)
    print("Writing: " + path + "pose_graph.json")
    write_read_and_draw_combined_geometries(combined_cloud, "multiway_registration.pcd", path)


def estimate_odometry_and_register_fragments(fragment_paths, odometry_paths, fragmentroot, generate, blensor):
    """
    Estimates odometry between fragments by ICP and registers fragments based on estimate
    Args:
        fragment_paths (list): list of paths to where fragments are located
        odometry_paths (list): list of paths to where odometry data are located
        fragmentroot (str): path to the root directory of fragment
        generate (bool): True if both odometry and fragments in should be generated before registering fragments.
                        False if previously generated pose graphs and clouds should be used to register fragments.
        blensor (bool): True if point clouds in paths are generated using BlenSor

    Returns:
        None
    """
    odometry_quaternions, odometry_positions, _ = read_quaternions_from_paths(paths=odometry_paths, element="odometry",
                                                                              blensor=blensor)

    if generate:
        print("Generating fragments ...")
        for o_path, o_quaternions, o_positions in zip(odometry_paths, odometry_quaternions, odometry_positions):
            load_files_and_register_clouds(o_path, "odometry", o_quaternions, o_positions, blensor)
        scan_quaternions, scan_positions, _ = read_quaternions_from_paths(paths=fragment_paths, element="scan",
                                                                          blensor=blensor)
        for s_path, s_quaternions, s_positions in zip(fragment_paths, scan_quaternions, scan_positions):
            load_files_and_register_clouds(s_path, "scan", s_quaternions, s_positions, blensor)

    pose_graphs = load_pose_graphs(paths=odometry_paths)
    estimated_translations = get_estimated_translation(pose_graphs=pose_graphs, quaternions=odometry_quaternions)
    clouds, outliers, quaternions, positions = load_fragments(fragment_paths=fragment_paths,
                                                              estimated_translations=estimated_translations,
                                                              voxel_down_size=config["fragment_voxel_size"],
                                                              combined_cloud=True)
    transformation_matrices = make_transformation_matrices(quaternions=quaternions, positions=positions)
    transform_and_visualize_point_clouds(clouds=clouds, outliers=outliers,
                                         relative_transformation=transformation_matrices, path=fragmentroot)
    pose_graph = full_registration(clouds=clouds, relative_transformation_matrices=transformation_matrices,
                                   verbose=False, element="fragment")
    optimized_pose_graph = optimize_pose_graph(pose_graph=pose_graph, max_correspondence_distance=
    config["fragment"]["icp"][config["local_icp_method_fragment"]]["max_correspondence_distance"],
                                               edge_prune_threshold=config["pose_graph_optimization"][
                                                   "edge_prune_threshold"])
    write_pose_graph(filename=fragmentroot + "pose_graph.json", pose_graph=optimized_pose_graph)

    clouds, outliers, _, _ = load_fragments(fragment_paths=fragment_paths,
                                            estimated_translations=estimated_translations, voxel_down_size=False,
                                            combined_cloud=True)
    clouds_copy = copy.deepcopy(clouds)
    transformed_clouds = transform_points(clouds=clouds_copy, pose_graph=optimized_pose_graph)
    clouds_combined = combine_point_clouds(transformed_clouds)
    write_read_and_draw_combined_geometries(cloud=clouds_combined, filename="multiway_registration.pcd",
                                            path=fragmentroot, trajectory=True, odometry_paths=odometry_paths,
                                            fragment_paths=fragment_paths)  # visualize trajectory


def write_read_and_draw_combined_geometries(cloud, filename, path, trajectory=False, odometry_paths=[],
                                            fragment_paths=[]):
    """
    Writes, reads and visualizes geometries. Visualizes written cloud due to performance issues.
    Visualizes trajectory if argument and paths are specified in funtion call.
    Args:
        cloud (open3d.geometry.PointCloud): point cloud object
        filename (str): name of point cloud to be written
        path (str): path to folder where all fragment folders are located
        trajectory (bool): True if trajectory should be visualized (default: False)
        odometry_paths (list): list of strings representing the path to odometry folders (default: empty list)
        fragment_paths (list): list of strings representing the path to fragment folders (default: empty list)

    Returns:
        None
    """
    estimate_normals(cloud, KDTreeSearchParamHybrid(radius=config["estimate_normals"]["radius"],
                                                    max_nn=config["estimate_normals"]["max_nn"]))
    orient_normals_towards_camera_location(cloud, camera_location=np.array([0, 0, 0]))
    full_path = path + filename
    print("Writing: " + full_path)
    write_point_cloud(filename=full_path, pointcloud=cloud)
    print("Reading: " + full_path)
    read_cloud = read_point_cloud(full_path)

    if trajectory:
        create_trajectory(odometry_paths, fragment_paths, read_cloud)
    else:
        print("Visualizing: " + full_path)
        draw_geometries_with_editing([read_cloud])


def create_trajectory(odometry_paths, fragment_paths, cloud):
    """
    Creates a trajectory in the given cloud based on the generated pose graph. Visualizes cloud with trajectory, and
    writes linesets and frames to given paths.
    Args:
        odometry_paths (list): list of strings represetning the path to odometry folders
        fragment_paths (list): list of strings represetning the path to fragment folders
        cloud (open3d.geometry.PointCloud): point cloud object

    Returns:
        None
    """
    pose_graphs = load_pose_graphs(odometry_paths)
    line_sets = []
    previous_last_point = np.asarray([0, 0, 0])
    origin = create_mesh_coordinate_frame(size=1, origin=[0, 0, 0])
    mesh_frames = [origin]
    for pose_graph in pose_graphs:
        poses = pose_graph.nodes
        translations = []
        for pose in poses:
            translations.append(copy.deepcopy(pose.pose[:3, 3]))  # get translation in pose graph
        points = [list(previous_last_point)]
        lines = []
        translations.pop()  # last translation tends to be erroneous. LiDAR is also mostly stationary in these cases
        translations.pop(0)  # first translation tends to be currupted. LiDAR is also mostly stationary in these cases
        for t in translations:
            p = list(previous_last_point + t)
            p[2] = 0  # assuming no z-translation
            points.append(p)
        translations[-1][2] = 0  # assuming no z-translation
        previous_last_point = previous_last_point + translations[-1]
        mesh_frame = create_mesh_coordinate_frame(size=1, origin=points[-1])
        mesh_frames.append(mesh_frame)
        for k, _ in enumerate(translations):
            line = [k, k + 1]
            lines.append(line)
        colors = [[1, 0, 0] for _ in lines]  # color trajectory red
        line_set = LineSet()  # initialize lineset
        line_set.points = Vector3dVector(points)
        line_set.lines = Vector2iVector(lines)
        line_set.colors = Vector3dVector(colors)
        line_sets.append(line_set)

    geometries = [cloud]
    for set in line_sets:
        geometries.append(set)
    for frame in mesh_frames:
        geometries.append(frame)
    print("Visualizing cloud with trajectory")
    draw_geometries(geometries)  # draw_geometries_with_editing does not support meshes and linesets

    for path, set in zip(odometry_paths, line_sets):
        print("Writing: " + path + "line_set.ply")
        write_line_set(path + "line_set.ply", set)
    for path, frame in zip(fragment_paths, mesh_frames):
        print("Writing: " + path + "frame.ply")
        write_triangle_mesh(path + "frame.ply", frame)


def visualize_trajectory(odometry_paths, fragment_paths, fragmentroot):
    """
    Reads multiway_registration.pcd in fragmentroot and visualizes written linesets and frames
    Args:
        odometry_paths (list): list of strings represetning the path to odometry folders
        fragment_paths (list): list of strings represetning the path to fragment folders
        fragmentroot (str): path to folder where all fragment folders are located

    Returns:
        None
    """
    cloud = read_point_cloud(fragmentroot + "multiway_registration.pcd")
    geometries = [cloud]
    for path in odometry_paths:
        set = read_line_set(path + "line_set.ply")
        geometries.append(set)
    for path in fragment_paths:
        frame = read_triangle_mesh(path + "frame.ply")
        geometries.append(frame)
    print("Visualizing: " + fragmentroot + "multiway_registration.pcd with trajectory")
    draw_geometries(geometries)


def optimize_pose_graph(pose_graph, max_correspondence_distance, edge_prune_threshold):
    """
    Optimizes given pose graph
    Args:
        pose_graph (open3d.registration.PoseGraph): pose graph object
        max_correspondence_distance (float): maximum distance between corresponding points in point clouds
        edge_prune_threshold (float): threshold of edge pruning

    Returns:
        open3d.registration.PoseGraph: optimized pose graph
    """
    print("Optimizing PoseGraph ...")
    option = GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance,
        edge_prune_threshold=edge_prune_threshold,
        reference_node=0)
    global_optimization(pose_graph,
                        GlobalOptimizationLevenbergMarquardt(),
                        GlobalOptimizationConvergenceCriteria(), option)
    return pose_graph


def transform_points(clouds, pose_graph):
    """
    Transforms point clouds with poses of given pose graph
    Args:
        clouds (list): list of point cloud objects
        pose_graph (open3d.registration.PoseGraph): pose graph object

    Returns:
        list: list of transformed point cloud objects
    """
    clouds_copy = copy.deepcopy(clouds)
    for point_id, cloud in enumerate(clouds_copy):
        cloud.transform(pose_graph.nodes[point_id].pose)
    return clouds_copy


def combine_point_clouds(clouds):
    """
    Combines given point clouds to one point cloud
    Args:
        clouds (list): list of point cloud objects

    Returns:
        open3d.geometry.PointCloud: point cloud object
    """
    cloud_combined = PointCloud()
    for cloud in clouds:
        cloud_combined += cloud
    return cloud_combined


def sortkey(path):
    """
    Find sorting key in path name
    Args:
        path (str): path name

    Returns:
        int: integer in path, for sorting
    """
    num = re.findall(r"\d+", path[-4:-1])  # tolerates up to 9999 different folders (4 digits)
    return int(num[0])


def find_paths(name, blensor=False):
    """
    Find the paths inside given data set folder
    Args:
        name (str): name of data set
        blensor (bool): True if point clouds located in end of paths are generated using BlenSor (default: False)

    Returns:
        tuple: (fragment_paths, fragmentroot, odometry_paths, odometryroot)
            fragment_paths (list): list of paths to where fragments are located
            fragmentroot (str): path to the root directory of fragment
            odometry_paths (list): list of paths to where odometry data are located
            odometryroot (str): path to root directory of odometry data
    """
    if blensor:
        prefix = "/tmp/"
    else:
        prefix = "../data/"

    fragmentroot = prefix + name + "/fragments/"
    odometryroot = prefix + name + "/odometry/"
    fragment_paths = []
    odometry_paths = []
    for root, dirs, files in os.walk(fragmentroot):
        for d in dirs:
            path = os.path.join(root, d) + "/"
            fragment_paths.append(path)
    for root, dirs, files in os.walk(odometryroot):
        for d in dirs:
            path = os.path.join(root, d) + "/"
            odometry_paths.append(path)
    fragment_paths.sort(key=sortkey)
    odometry_paths.sort(key=sortkey)
    print(f"Fragment folders: {len(fragment_paths)}")
    print(f"Odometry folders: {len(odometry_paths)}")
    return fragment_paths, fragmentroot, odometry_paths, odometryroot


if __name__ == "__main__":
    print("--------------------------------------------")
    print(colored(text="CONFIGURATION PARAMETERS:", color="red", attrs=["bold"]))
    print("--------------------------------------------")
    print("dataset_name:", config["dataset_name"])
    print("scan_voxel_size:", config["scan_voxel_size"])
    print("fragment_voxel_size:", config["fragment_voxel_size"])
    print("odometry_voxel_size:", config["odometry_voxel_size"])
    print("scan_outlier_removal:", config["scan_outlier_removal"])
    print("fragment_outlier_removal:", config["fragment_outlier_removal"])
    print("odometry_outlier_removal:", config["odometry_outlier_removal"])
    print("--------------------------------------------")

    icp, visualization, verbose, odometry, generate, blensor, trajectory = parse_args()
    if blensor:
        prefix = "/tmp/"
    else:
        prefix = "../data/"

    if verbose:
        print("VerbosityLevel: Debug")
        set_verbosity_level(VerbosityLevel.Debug)

    if odometry:
        print(colored(text="ODOMETRY ESTIMATION AND FRAGMENT REGISTRATION:", color="red", attrs=["bold"]))
        print("--------------------------------------------")
        fragment_paths, fragmentroot, odometry_paths, odometryroot = find_paths(config["dataset_name"], blensor)
        estimate_odometry_and_register_fragments(fragment_paths=fragment_paths, odometry_paths=odometry_paths,
                                                 fragmentroot=fragmentroot, generate=generate,
                                                 blensor=blensor)
    if trajectory:
        print(colored(text="VISUALIZATION WITH TRAJECTORY:", color="red", attrs=["bold"]))
        print("--------------------------------------------")
        fragment_paths, fragmentroot, odometry_paths, odometryroot = find_paths(config["dataset_name"], blensor)
        visualize_trajectory(odometry_paths, fragment_paths, fragmentroot)

    if visualization:
        print(colored(text="VISUALIZATION:", color="red", attrs=["bold"]))
        print("--------------------------------------------")
        argument = visualization[0]
        if argument == "f":
            element = "scan"
        else:
            element = "odometry"
        path = prefix + config["dataset_name"] + "/" + visualization + "/"
        quaternions, positions, _ = read_quaternions_file(path, element=element, blensor=blensor)
        clouds, outliers, _ = load_point_clouds(path, element=element, voxel_size=False, blensor=blensor)
        transformation_matrices = make_transformation_matrices(quaternions, positions)
        transform_and_visualize_point_clouds(clouds=clouds, outliers=outliers,
                                             relative_transformation=transformation_matrices, path=path)

    if icp:
        print(colored(text="REGISTRATION:", color="red", attrs=["bold"]))
        print("--------------------------------------------")
        argument = icp[0]
        if argument == "f":
            element = "scan"
        else:
            element = "odometry"
        path = prefix + config["dataset_name"] + "/" + icp + "/"
        quaternions, positions, _ = read_quaternions_file(path, element=element, blensor=blensor)
        clouds, outliers, _ = load_point_clouds(path, element=element, voxel_size=config[element + "_voxel_size"],
                                                blensor=blensor)
        transformation_matrices = make_transformation_matrices(quaternions, positions)
        print("Visualizing initial alignment...")
        transform_and_visualize_point_clouds(clouds=clouds, outliers=outliers,
                                             relative_transformation=transformation_matrices, path=path)
        pose_graph = full_registration(clouds, transformation_matrices, verbose, element=element)
        method = config["local_icp_method_" + element]
        optimized_pose_graph = optimize_pose_graph(pose_graph,
                                                   max_correspondence_distance=config[element]["icp"][method][
                                                       "max_correspondence_distance"],
                                                   edge_prune_threshold=config["pose_graph_optimization"][
                                                       "edge_prune_threshold"])
        print("Writing: " + path + "pose_graph.json")
        write_pose_graph(path + "/pose_graph.json", pose_graph)
        clouds, outliers, _ = load_point_clouds(path, element=element, voxel_size=False, blensor=blensor)
        transformed_clouds = transform_points(clouds, optimized_pose_graph)
        cloud_combined = combine_point_clouds(transformed_clouds)
        write_read_and_draw_combined_geometries(cloud_combined, "multiway_registration.pcd", path)
