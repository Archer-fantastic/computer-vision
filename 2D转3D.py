import cv2
import open3d as o3d
import numpy as np

def fun(filepath):
    # 读取RGB图像
    image = cv2.imread(filepath,flags=cv2.IMREAD_COLOR)

    # 估计深度信息
    # 可以使用深度学习模型、立体视觉算法等方法
    # 这里使用简单的方法，假设深度与图像亮度成正比
    depth_map = np.mean(image, axis=2)  # 假设深度与图像亮度成正比，简化处理
    # depth_map = (depth_map - np.min(depth_map)) / (np.max(depth_map) - np.min(depth_map))  # 归一化

    # 生成点云
    rows, cols = depth_map.shape
    points = []
    for y in range(rows):
        for x in range(cols):
            depth = depth_map[y, x]
            # 这里使用默认的相机参数
            focal_length_x = 700  # 假设相机焦距为500
            focal_length_y = 700  # 假设相机焦距为500
            if depth > 0:  # 排除无效深度
                # 根据相机模型将像素坐标转换为相机坐标系下的坐标
                camera_x = (x - cols / 2) * depth / focal_length_x
                camera_y = (y - rows / 2) * depth / focal_length_y
                camera_z = depth
                points.append([camera_x, camera_y, camera_z])

    # 创建Open3D点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # 可以对点云进行滤波、去除离群点等处理
    # 例如，使用半径滤波器进行滤波
    point_cloud = point_cloud.voxel_down_sample(voxel_size=0.05)  # 简单的体素下采样

    # 保存点云
    o3d.io.write_point_cloud("output_point_cloud.ply", point_cloud)

    # 可以使用Open3D可视化点云
    o3d.visualization.draw_geometries([point_cloud])

def fun2(path):
    import cv2
    import open3d as o3d

    # 读取RGB电路板图像
    image = cv2.imread(path)

    # 进行图像预处理，如边缘检测
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    cv2.imshow("img",image)
    cv2.waitKey(0)
    # cv2.imshow("edges",edges)
    # cv2.waitKey(0)

    # # 提取轮廓
    # contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 生成点云和颜色
    points = []
    colors = []
    a,b = edges.shape
    for x in range(a):
        for y in range(b):
            # 将轮廓点映射到3D空间中，可以简单地假设深度为0
            if edges[x][y]!=0:
                points.append([x, y, 0])
                # 获取像素点的颜色作为点云的颜色
                color = image[x, y] / 255  # 注意颜色顺序为BGR
                colors.append([color[2], color[1], color[0]])  # 颜色顺序转换为RGB

    # 创建Open3D点云对象，并设置点和颜色
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)

    # 可以对点云进行滤波、去除离群点等处理
    # 例如，使用半径滤波器进行滤波
    # point_cloud = point_cloud.random_down_sample(sampling_ratio=0.6)  # 简单的下采样

    # 保存点云
    o3d.io.write_point_cloud("output_point_cloud_with_color.ply", point_cloud)

    # 可以使用Open3D可视化点云
    o3d.visualization.draw_geometries([point_cloud])


fun2('apple.png')
# fun2("test.png")
fun2("grape.png")
fun2("circuit.png")

