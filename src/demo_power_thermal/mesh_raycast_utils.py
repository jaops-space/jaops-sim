# This module provides utility functions for working with meshes, raycasting, and lighting in an IsaacSim 3D stage.

import omni.isaac.core.utils.prims as prims_utils
import omni.isaac.core.utils.rotations as rotations_utils
import omni.isaac.core.utils.mesh as mesh_utils
from omni.physx import get_physx_scene_query_interface  # for the raycasting
import numpy as np


def adjust_light(stage, position, intensity=3e9, radius=0.01):
    """
    Adjusts the properties of the first SphereLight found in the given stage.

    Parameters:
    stage (Usd.Stage): The stage containing the lights.
    position (tuple): A tuple representing the new position (x, y, z) of the light.
    intensity (float, optional): The new intensity of the light. Default is 3e9.
    radius (float, optional): The new radius of the light. Default is 0.01.

    Returns:
        None
    """
    lights = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "SphereLight":
            lights.append(prim)
    if lights:
        light = lights[0]
        light_path = light.GetPath()
        # Adjust position
        translate_attr = stage.GetPrimAtPath(light_path).GetAttribute(
            "xformOp:translate"
        )
        translate_attr.Set(position)
        # Adjust intensity
        intensity_attr = stage.GetPrimAtPath(light_path).GetAttribute(
            "inputs:intensity"
        )
        intensity_attr.Set(intensity)
        # Adjust radius
        radius_attr = stage.GetPrimAtPath(light_path).GetAttribute("inputs:radius")
        radius_attr.Set(0.01)


def set_prim_orientation_euler(path, angles, degrees=False):
    """
    Set the orientation of a primitive at the given path using Euler angles.

    Args:
        path (str): The path to the primitive whose orientation is to be set.
        angles (tuple or list): The Euler angles (in radians or degrees) to set the orientation.
        degrees (bool, optional): If True, the angles are interpreted as degrees.
                                  If False, the angles are interpreted as radians. Default is False.

    Returns:
        None
    """
    prims_utils.set_prim_attribute_value(
        path,
        "xformOp:orient",
        rotations_utils.euler_angles_to_quat(angles, degrees=degrees),
    )


def get_mesh_vertices_world(stage, path):
    """
    Retrieves the vertices of a mesh in world coordinates.

    Args:
        path (str): The path to the mesh primitive in the stage.

    Returns:
        list: A list of vertices in world coordinates.
    """
    mesh_prim = stage.GetPrimAtPath(path)
    world_prim = stage.GetPrimAtPath("/World")
    points_world = mesh_utils.get_mesh_vertices_relative_to(mesh_prim, world_prim)
    return points_world


def get_rect_areas(points_coord, face_vertex_indices):
    """
    Calculate the areas of rectangles defined by vertex indices.

    This function assumes that the faces are rectangles (quads) and calculates
    their areas based on the coordinates of their vertices.

    Parameters:
    points_coord (array-like): An array of shape (N, 3) containing the coordinates of the points.
    face_vertex_indices (array-like): An array of shape (M, 4) containing the indices of the vertices that form each face.

    Returns:
    numpy.ndarray: An array of shape (M,) containing the areas of the rectangles.
    """
    from scipy.spatial.distance import pdist

    print("in get_rect_areas(): assuming rectangles")
    num_quads = len(face_vertex_indices) // 4
    quad_areas = np.zeros(num_quads)
    for i in range(num_quads):
        distance = sorted(
            pdist(points_coord[list(face_vertex_indices[i * 4 : (i + 1) * 4])]),
            reverse=True,
        )
        quad_areas[i] = (
            distance[-1] * distance[-3]
        )  # the two largest distances are the diagonals. the width and length are the four shortest
    return quad_areas


def get_rect_normals(points_coord, face_vertex_indices):
    """
    Calculate the normal vectors for a set of rectangular faces.

    This function assumes that the faces are rectangles and calculates the normal
    vectors for each face using the coordinates of the points and the indices of
    the vertices that form the faces.

    Parameters:
    points_coord (array-like): An array of shape (N, 3) containing the coordinates of the points.
    face_vertex_indices (array-like): An array of shape (M, 4) containing the indices of the vertices that form each face.

    Returns:
    np.ndarray: An array of shape (num_quads, 3) containing the normal vectors for each rectangular face.
    """
    print("in get_rect_normals(): assuming rectangles")
    num_quads = len(face_vertex_indices) // 4
    rect_normals = np.zeros((num_quads, 3))
    for i in range(num_quads):
        quad_points = points_coord[list(face_vertex_indices[i * 4 : (i + 1) * 4])]
        cross = np.cross(
            quad_points[2] - quad_points[0],
            quad_points[3] - quad_points[1],
        )
        if np.all(cross == 0):
            print("detected parallel vectors, continuing")
            cross = np.cross(
                quad_points[1] - quad_points[0],
                quad_points[3] - quad_points[2],
            )
            assert not np.all(cross == 0)
        rect_normals[i] = cross / np.linalg.norm(cross)
    return rect_normals


def get_quad_centers(points_coord, face_vertex_indices):
    """
    Calculate the centers of quadrilateral faces in a mesh.

    Parameters:
    points_coord (array-like): An array of shape (N, 3) containing the coordinates of the points.
    face_vertex_indices (array-like): An array of shape (M, 4) containing the indices of the vertices that form each face.

    Returns:
    numpy.ndarray: An array of shape (num_quads, 3) containing the coordinates of the centers of the quadrilateral faces.
    """
    num_quads = len(face_vertex_indices) // 4
    quad_centers = np.array(
        [
            np.mean(
                points_coord[list(face_vertex_indices[i * 4 : (i + 1) * 4])], axis=0
            )
            for i in range(num_quads)
        ]
    )
    return quad_centers


def compute_sun_to_mesh(mesh_face_centers, path_sun="/World/Sun", output="dotproducts"):
    """
    Uses ray-tracing to compute the dot products between the normals of mesh face centers and the direction from the Sun to each face center.

    Parameters:
    mesh_face_centers (array-like): An array of 3D coordinates representing the centers of the mesh faces.
    path_sun (str): The path to the Sun object in the scene. Default is "/World/Sun".
    output (str): The type of output to return. Options are "dotproducts", "dotproducts_normals", or "angles_deg".
                  Default is "dotproducts".

    Returns:
    numpy.ndarray or tuple: Depending on the output parameter:
        - "dotproducts": Returns an array of dot products.
        - "dotproducts_normals": Returns a tuple containing an array of dot products and an array of normals.
        - "angles_deg": Returns an array of angles in degrees.

    Notes:
    - If a mesh face center is in the shadow of an obstruction, the corresponding dot product and normal are set to NaN.
    - If no hit is detected for a mesh face center, the corresponding dot product and normal are set to NaN.
    - The function prints a message if no hit is detected for a mesh face center.
    """
    sun_coord = prims_utils.get_prim_attribute_value(path_sun, "xformOp:translate")

    dot_products = np.zeros(
        len(mesh_face_centers)
    )  # @TODO simplify: only compute dot_products at the end, based on normals
    normals = np.zeros((len(mesh_face_centers), 3))
    hit_distances = []  # for debug

    for i, center in enumerate(mesh_face_centers):
        vector_to_cell = center - sun_coord
        distance = np.linalg.norm(vector_to_cell)
        direction = vector_to_cell / distance

        # Cast a ray from the Sun to the cell center
        hit_info = get_physx_scene_query_interface().raycast_closest(
            sun_coord, direction, distance + 1
        )  # increase max distance to avoid ray not reaching the surface

        if hit_info["hit"]:
            hit_distance = np.linalg.norm(hit_info["position"] - center)
            hit_distances.append(hit_distance)
            if (
                hit_distance < 0.001
            ):  # consider that there is no obstruction between the sun and the cell center
                normals[i] = np.array(hit_info["normal"])
                dot_products[i] = np.dot(
                    normals[i], -1 * direction
                )  # direction is sun to center, so need to flip it
            else:  # cell is in shadow of obstruction
                dot_products[i] = np.nan
                normals[i] = np.nan
        else:
            print(f"no hit, angle for cell center at {center} is undefined")
            dot_products[i] = np.nan  # No hit, angle is undefined
            normals[i] = np.nan

    if output == "dotproducts":
        return dot_products
    if output == "dotproducts_normals":
        return dot_products, normals
    elif output == "angles_deg":
        sun_angles = np.arccos(dot_products) * (180 / np.pi)  # Convert to degrees
        return sun_angles
    else:
        print(
            "possible outputs are only 'dotproducts' , 'dotproducts_normals' or 'angles_deg'"
        )
        return


def color_conversion(array, norm_mode="minmax", add_tail=False):
    """
    Convert a given array of values to a color representation using normalization.

    Parameters:
    array (numpy.ndarray): The input array of values to be converted to colors.
    norm_mode (str or tuple): The normalization mode to use. If "minmax", the array is normalized
                              using its minimum and maximum values. If a tuple of two values is provided,
                              those values are used as the min and max for normalization.
    add_tail (bool): If True, appends two additional rows of zeros to the resulting color array.

    Returns:
    numpy.ndarray: An array of RGB color values corresponding to the normalized input array.

    Notes:
    - NaN values in the input array are replaced with 0.
    - The colormap used for conversion is 'jet' from matplotlib.
    """
    # adjust for visualization
    cleaned = np.nan_to_num(array)  # replace all NaN by 0
    if norm_mode == "minmax":
        normalized = np.interp(cleaned, [cleaned.min(), cleaned.max()], [0, 1])
    elif len(norm_mode) == 2:
        normalized = np.interp(cleaned, norm_mode, [0, 1])
    else:
        print(
            "norm_mode must either be 'minmax' or a tuple specifying the min and max to use for normalizing"
        )
        return

    import matplotlib.cm as cm

    colors = cm.jet(normalized)[:, :3]

    if add_tail:
        # add two additional zeros at the end to match the shape required, not sure why
        colors = np.vstack([colors, np.zeros((2, 3))])
    return colors


def color_cube_test(cube_path):
    """
    Assigns colors to the faces of a cube mesh based on points indices.
    This function tests that the mapping between point indices and cube faces is correct

    Parameters:
    cube_path (str): The path to the cube mesh.
    Returns:
    np.ndarray: An array of RGB colors assigned to each point in the cube mesh.
    """

    n_points = len(prims_utils.get_prim_attribute_value(cube_path, "points"))
    # Define six equally spaced hues in the HSV space
    hues = np.linspace(
        0, 1, 6, endpoint=False
    )  # hue = color wheel: hue 0 and hue 1 are the same so do not include endpoint
    import matplotlib.colors as mcolors

    rgb_colors = [mcolors.hsv_to_rgb((hue, 1.0, 1.0)) for hue in hues]

    # Define the index ranges based on the mesh point structure
    assert n_points >= 280
    face_colors = np.zeros((n_points, 3))
    cube_mask = [
        np.arange(0, 100),  # front
        np.arange(100, 200),  # back
        np.arange(200, 220),  # four sides
        np.arange(220, 240),
        np.arange(240, 260),
        np.arange(260, 280),
    ]
    for i, mask in enumerate(cube_mask):
        face_colors[mask] = rgb_colors[i]
    return face_colors


def quick_stats(data, round_digits=1):
    """
    Calculate common statistics for the given data.
    Usage: pprint(quick_stats(data), sort_dicts=False)

    Parameters:
    data (array-like): The input data for which statistics are to be calculated.
    round_digits (int, optional): The number of decimal places to round the results to. Default is 1.

    Returns:
    dict: A dictionary containing common statistics:
    """
    return {
        key: round(func(data), round_digits)
        for key, func in zip(
            [
                "mean",
                "median",
                "std",
                "min",
                "max",
                "25%",
                "50%",
                "75%",
                "NaN count",
                "NaN %",
            ],
            [
                np.nanmean,
                np.nanmedian,
                np.nanstd,
                np.nanmin,
                np.nanmax,
                lambda x: np.nanpercentile(x, 25),
                lambda x: np.nanpercentile(x, 50),
                lambda x: np.nanpercentile(x, 75),
                lambda x: np.isnan(x).sum(),
                lambda x: (np.isnan(x).sum() / len(x)) * 100,
            ],
        )
    }
