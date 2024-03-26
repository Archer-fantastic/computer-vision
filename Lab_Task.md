# 2D RGB图像转3D点云数据

>   中午老板给了这个任务，老板要的不是具体的3D数据，而是想要这样的效果。

## 思路

如果按照正常套路的话应该是走3D重建，上模型来搞，不过因为我之前搞过一些opencv，所以就用了另外一种笨拙但有效的方法。

>   RGB image ==> Gray image ==> Cannny边缘检测 ==>3D的z坐标置零 ==> 保留原来的像素点颜色 ==> 生成点云

## 代码

```python
def transform(path):
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
                colors.append([color[0], color[1], color[2]])  # 颜色顺序转换为RGB

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
```

## 效果

![image-20240326195105869](G:\TyporaProjects\D_实验室文件\imgs\Lab_Task_imgs\image-20240326195105869.png)

![image-20240326195233679](G:\TyporaProjects\D_实验室文件\imgs\Lab_Task_imgs\image-20240326195233679.png)

![image-20240326195516644](G:\TyporaProjects\D_实验室文件\imgs\Lab_Task_imgs\image-20240326195516644.png)