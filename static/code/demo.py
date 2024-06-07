import numpy as np
import open3d as o3d

color = o3d.io.read_image("gate.jpg")
depth = o3d.io.read_image("gate_depth_16bit.png")
depth2 = np.asarray(depth) / 65535.0
depth2 = o3d.geometry.Image(depth2.astype(np.float32))

pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(800, 500, 460, 460, 400, 250)

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth2, convert_rgb_to_intensity = False)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)

# flip the orientation, so it looks upright, not upside-down
pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

o3d.visualization.draw_geometries([pcd]) 
o3d.io.write_point_cloud("gate.pcd", pcd)