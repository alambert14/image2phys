import numpy as np
import open3d as o3d
import os
import open3d as o3d
import xml.etree.ElementTree as tree

from tqdm import tqdm

from manipulation.meshcat_utils import draw_open3d_point_cloud, draw_points
from manipulation.open3d_utils import create_open3d_point_cloud

from ycb_downloader import fetch_objects, objects_url
from ycb_mass import ycb_mass


base_dir = '/home/aalamber/ycb'


def load_point_cloud(obj_name: str) -> o3d.geometry.PointCloud:
    """
    :param obj_name: YCB name of object to load
    :return: a PointCloud object loaded from the file
    """
    dir = os.path.join(base_dir, obj_name, 'clouds/merged_cloud.ply')
    pcl = o3d.io.read_point_cloud(dir)

    return pcl


def load_voxels(obj_name: str) -> o3d.geometry.VoxelGrid:
    """
        :param obj_name: YCB name of object to load
        :return: a VoxelGrid object loaded from the file
        """
    dir = os.path.join(base_dir, obj_name, 'poisson/voxel.xyz')
    voxel_grid = o3d.io.read_voxel_grid(dir)
    # print(voxel_grid)
    # print(dir)
    o3d.visualization.draw_geometries([voxel_grid])

    return voxel_grid


def load_voxels_from_mesh(obj_name: str) -> o3d.geometry.VoxelGrid:
    """
        :param obj_name: YCB name of object to load
        :return: a VoxelGrid object loaded from the file
        """
    dir = os.path.join(base_dir, obj_name, 'poisson/nontextured.stl')
    # print(dir)
    mesh = o3d.io.read_triangle_mesh(dir)
    mesh.scale(1 / np.max(mesh.get_max_bound() - mesh.get_min_bound()),
               center=mesh.get_center())
    # o3d.visualization.draw_geometries([mesh])

    # print('voxelization')
    voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh,
                                                                  voxel_size=0.025)
    # o3d.visualization.draw_geometries([voxel_grid])

    return voxel_grid

def pcl_to_voxel(pcl: o3d.geometry.PointCloud, voxel_size: float = 0.001):
    """
    Convert a pointcloud from drake into a voxel grid

    :param pcl: Pointcloud from drake camera
    :param voxel_size: size of the voxel for each point in the pointcloud
    :return: A voxel grid, where each voxel corresponds to a pointcloud point
    """
    # o3d_pcl = create_open3d_point_cloud(pcl)
    # o3d.visualization.draw_geometries([pcl])
    voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(pcl, voxel_size=voxel_size)
    # o3d.visualization.draw_geometries([voxels])
    return voxels


def calculate_moment_of_inertia_origin(voxel_grid: o3d.geometry.VoxelGrid,
                                       total_mass: float, voxel_size = 0.001):
    voxels = voxel_grid.get_voxels()
    voxel_mass = total_mass / len(voxels)
    # print(len(voxels))

    # TODO: Find the center of mass, and then find the moment of inertia wrt the com
    coords = np.array([voxel.grid_index for voxel in voxels]) * voxel_size
    # print(coords.shape)
    com = np.sum(coords, axis=0) / len(voxels)  # * voxel_mass / total_mass
    # print(com)

    # TODO: vectorize this
    Ixx = 0
    Iyy = 0
    Izz = 0
    Ixy = 0
    Iyz = 0
    Ixz = 0

    Gxx = 0
    Gyy = 0
    Gzz = 0
    Gxy = 0
    Gyz = 0
    Gxz = 0
    for voxel in voxels:
        coord = (voxel.grid_index * voxel_size)
        Gxx += coord[0] ** 2
        Gyy += coord[1] ** 2
        Gzz += coord[2] ** 2
        Gxy += coord[0] * coord[1]
        Gxz += coord[0] * coord[2]
        Gyz += coord[1] * coord[2]
        # Ixx += voxel_mass * (coord[1] ** 2 + coord[2] ** 2)
        # Iyy += voxel_mass * (coord[0] ** 2 + coord[2] ** 2)
        # Izz += voxel_mass * (coord[0] ** 2 + coord[1] ** 2)
        # Ixy -= voxel_mass * coord[0] * coord[1]
        # Iyz -= voxel_mass * coord[1] * coord[2]
        # Ixz -= voxel_mass * coord[0] * coord[2]

    Gxx /= len(voxels)
    Gyy /= len(voxels)
    Gzz /= len(voxels)
    Gxy /= len(voxels)
    Gyz /= len(voxels)
    Gxz /= len(voxels)
    Gxz *= total_mass

    Gxx *= total_mass
    Gyy *= total_mass
    Gzz *= total_mass
    Gxy *= total_mass
    Gyz *= total_mass

    print(np.array([total_mass, com[0], com[1], com[2], Gxx, Gyy, Gzz, Gxy, Gxz, Gyz]))
    return np.array([total_mass, com[0], com[1], com[2], Ixx, Iyy, Izz, Ixy, Ixz, Iyz])

def calculate_inertia_tensor_from_voxels(voxel_grid: o3d.geometry.VoxelGrid,
                                         total_mass: float, voxel_size = 0.001):
    """
    Find the inertia tensor of a voxelized object given a per-unit mass
    :param voxel_grid: Voxelized object
    :param voxel_mass: Mass of each voxel
    :return: np.array - intertia tensor
    """
    voxels = voxel_grid.get_voxels()
    voxel_mass = total_mass / len(voxels)
    # print(len(voxels))

    # TODO: Find the center of mass, and then find the moment of inertia wrt the com
    coords = np.array([voxel.grid_index for voxel in voxels]) * voxel_size
    # print(coords.shape)
    com = np.sum(coords, axis=0) / len(voxels)  #  * voxel_mass / total_mass
    # print(com)

    # TODO: vectorize this
    # Gxx = 0
    # Gyy = 0
    # Gzz = 0
    # Gxy = 0
    # Gyz = 0
    # Gxz = 0
    Ixx = 0
    Iyy = 0
    Izz = 0
    Ixy = 0
    Iyz = 0
    Ixz = 0
    for voxel in voxels:
        coord = (voxel.grid_index * voxel_size) - com
        # coord = (voxel.grid_index * voxel_size)
        # Gxx += coord[0] ** 2
        # Gyy += coord[1] ** 2
        # Gzz += coord[2] ** 2
        # Gxy += coord[0] * coord[1]
        # Gxz += coord[0] * coord[2]
        # Gyz += coord[1] * coord[2]
        Ixx += voxel_mass * (coord[1] ** 2 + coord[2] ** 2)
        Iyy += voxel_mass * (coord[0] ** 2 + coord[2] ** 2)
        Izz += voxel_mass * (coord[0] ** 2 + coord[1] ** 2)
        Ixy -= voxel_mass * coord[0] * coord[1]
        Iyz -= voxel_mass * coord[1] * coord[2]
        Ixz -= voxel_mass * coord[0] * coord[2]
    # Gxx /= len(voxels)
    # Gyy /= len(voxels)
    # Gzz /= len(voxels)
    # Gxy /= len(voxels)
    # Gyz /= len(voxels)
    # Gxz /= len(voxels)
    #
    # Gxx *= total_mass
    # Gyy *= total_mass
    # Gzz *= total_mass
    # Gxy *= total_mass
    # Gyz *= total_mass
    # Gxz *= total_mass

    # print(np.array([total_mass, com[0], com[1], com[2], Ixx, Iyy, Izz, Ixy, Ixz, Iyz]))
    return total_mass, com[0], com[1], com[2], Ixx, Iyy, Izz, Ixy, Ixz, Iyz


def calculate_ground_truth_parameters(filename):
    pcl = o3d.io.read_point_cloud(filename)
    voxels = pcl_to_voxel(pcl)

    return calculate_inertia_tensor_from_voxels(voxels, 0.603000)


def calculate_inertial_parameters(obj_name):
    voxels = load_voxels_from_mesh(obj_name)
    mass = ycb_mass[obj_name]
    return calculate_inertia_tensor_from_voxels(voxels, mass)


def generate_obj_sdf(obj_name) -> None:
    """
    Given an object model file, create an SDF definition for simulation use.
    Saves the SDF file in '/home/aalamber/{obj_name}/poisson/model.sdf'
    :param obj_name: The name of the object to convert to SDF
    :return: None
    """
    sdf = tree.Element('sdf', attrib={'version': '1.7'})
    sdf.text = '\n  '
    model = tree.SubElement(sdf, 'model', attrib={'name': obj_name})
    model.text = '\n    '
    model.tail = '\n'
    link = tree.SubElement(model, 'link', attrib={'name': 'base_link'})
    link.text = '\n      '
    link.tail = '\n  '
    # Inertial data
    m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz = \
        calculate_inertial_parameters(obj_name)  # Implement this lol
    inertial = tree.SubElement(link, 'inertial')
    inertial.text = '\n        '
    inertial.tail = '\n      '
    mass = tree.SubElement(inertial, 'mass')
    mass.text = f'{m:.3f}'
    mass.tail = '\n        '
    com = tree.SubElement(inertial, 'pose')
    com.text = f'{cx:.6f} {cy:.6f} {cz:.6f} 0. 0. 0.'
    com.tail = '\n        '

    inertia = tree.SubElement(inertial, 'inertia')
    inertia.text = '\n          '
    inertia.tail = '\n      '
    ixx = tree.SubElement(inertia, 'ixx')
    ixx.text = f'{Ixx:.6f}'
    ixx.tail = '\n          '
    iyy = tree.SubElement(inertia, 'iyy')
    iyy.text = f'{Iyy:.6f}'
    iyy.tail = '\n          '
    izz = tree.SubElement(inertia, 'izz')
    izz.text = f'{Izz:.6f}'
    izz.tail = '\n          '
    ixy = tree.SubElement(inertia, 'ixy')
    ixy.text = f'{Ixy:.6f}'
    ixy.tail = '\n          '
    ixz = tree.SubElement(inertia, 'ixz')
    ixz.text = f'{Ixz:.6f}'
    ixz.tail = '\n          '
    iyz = tree.SubElement(inertia, 'iyz')
    iyz.text = f'{Iyz:.6f}'
    iyz.tail = '\n        '

    # Visual data
    visual = tree.SubElement(link, 'visual', attrib={'name': 'base_link'})
    visual.text = '\n        '
    visual.tail = '\n      '
    pose = tree.SubElement(visual, 'pose')
    pose.text = '0. 0. 0. 0. 0. 0.'
    pose.tail = '\n        '
    geometry = tree.SubElement(visual, 'geometry')
    geometry.text = '\n          '
    geometry.tail = '\n      '
    mesh = tree.SubElement(geometry, 'mesh')
    mesh.text = '\n            '
    mesh.tail = '\n        '
    uri = tree.SubElement(mesh, 'uri')
    uri.text = os.path.join(base_dir, obj_name, 'poisson/textured.obj')
    uri.tail = '\n          '

    # TODO: Collision
    collision = tree.SubElement(link, 'collision', attrib={'name': 'obj_collision'})
    collision.text = '\n        '
    collision.tail = '\n    '
    geometry2 = tree.SubElement(collision, 'geometry')
    geometry2.text = '\n          '
    geometry2.tail = '\n      '
    mesh2 = tree.SubElement(geometry2, 'mesh')
    mesh2.text = '\n            '
    mesh2.tail = '\n        '
    uri2 = tree.SubElement(mesh2, 'uri')
    uri2.text = os.path.join(base_dir, obj_name, 'poisson/textured.obj')
    uri2.tail = '\n            '
    convex = tree.SubElement(mesh2, 'drake:declare_convex', attrib={'xmlns:drake': 'drake.mit.edu'})
    convex.tail = '\n          '

    element_tree = tree.ElementTree(sdf)
    output_file = os.path.join(base_dir, obj_name, 'poisson/model.sdf')
    element_tree.write(output_file, xml_declaration=True)

if __name__ == '__main__':
    # # Open the laserscan
    # pcl = o3d.io.read_point_cloud('nontextured.ply')
    # voxels = pcl_to_voxel(pcl)
    # print(type(voxels))
    # print(calculate_interia_tensor_from_voxels(voxels, 0.603000))

    objects = fetch_objects(objects_url)

    inertias = {}
    exclude = {'023_wine_glass'}
    for obj in tqdm(objects):
        print(obj)
        try:
            mass = ycb_mass[obj]
        except KeyError:
            print(f'key error: {obj}')
            continue

        # if os.path.exists(os.path.join(base_dir, obj, 'poisson/model.sdf')):
        #     continue
        # voxels = load_voxels_from_mesh(obj)
        # inertias[obj] = calculate_inertia_tensor_from_voxels(voxels, mass * 0.001)
        generate_obj_sdf(obj)

    # print(inertias)

    # generate_obj_sdf('001_chips_can')