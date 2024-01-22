# -*- encoding: utf-8 -*-
"""
@File    :   visualize.py
@Time    :   2024/01/09 16:53:03
@Author  :   wujinbo
@Version :   1.0
@Contact :   wujinbo01@baidu.com
@Desc    :   None
"""

import os
import trimesh
import numpy as np
import open3d as o3d
import copy
import argparse


def visualize_pts(npy):
    pts = np.load(npy)
    pct = trimesh.PointCloud(pts)
    trimesh.Scene(pct).show()

def visualize_camera(camera_ex, mesh=None):
    """
    Args:
        camera_ex: w2c
    """
    points = [
    [-1, -1, -1],
    [1, -1, -1],
    [-1, 1, -1],
    [1, 1, -1],
    [-1, -1, 1],
    [1, -1, 1],
    [-1, 1, 1],
    [1, 1, 1],
    ]
    lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),

    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    #worlld coordinate
    WIDTH = 1280
    HEIGHT = 720
    vizualizer = o3d.visualization.Visualizer()
    vizualizer.create_window()
    vizualizer.create_window(width=WIDTH, height=HEIGHT)
    world_axisframe = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    for i in range(camera_ex.shape[0]):
        camera_axisframe = copy.deepcopy(world_axisframe).transform(camera_ex[i])
        vizualizer.add_geometry(camera_axisframe)
        # break
    vizualizer.add_geometry(world_axisframe)
    vizualizer.add_geometry(line_set)
    if mesh is not None:
        vizualizer.add_geometry(mesh)
    vizualizer.run()
    import pdb; pdb.set_trace()

def read_camera_pose(data_path):
    cam_lst = os.listdir(data_path)
    cam_lst.sort()
    cam_ex = []
    for i in range(len(cam_lst)):
        cam = np.eye(4)
        loaded_cam = np.load(os.path.join(data_path, cam_lst[i]))
        cam[:3, :4] = loaded_cam
        cam_ex.append(np.linalg.inv(cam))
    cam_ex = np.array(cam_ex)
    return cam_ex

def read_mesh(mesh_path):
    pcd_npy = np.load(mesh_path)
    import pdb; pdb.set_trace()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_npy)
    # pcd.colors = 
    return pcd

   
    return mesh

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Visualize Everything')

    parser.add_argument('--camera_path', type=str, help='input camera path')
    parser.add_argument('--mesh_path', type=str, default=None, help='input point cloud path')
    parser.add_argument('--vis_type', type=str, choices=['pcd', 'mesh', 'camera'], default='camera', help='input visualize type')
    args = parser.parse_args()

    if args.vis_type == 'camera':
        mesh = None
        if args.mesh_path is not None:
            mesh = read_mesh(args.mesh_path)
        cam_ex = read_camera_pose(args.camera_path)
        visualize_camera(cam_ex, mesh=mesh)
        
    elif args.vis_type == 'pcd' or args.vis_type == 'mesh':
        pass
    else:
        pass


    # data_path = sys.argv[1]
    # d
    