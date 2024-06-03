import pyrealsense2 as rs  
import numpy as np  
import cv2  
  
# 配置RealSense相机  
pipeline = rs.pipeline()  
config = rs.config()  
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  
  
# 启动流  
profile = pipeline.start(config)  
  
# 获取深度和彩色流  
depth_sensor = profile.get_device().first_depth_sensor()  
depth_scale = depth_sensor.get_depth_scale()  
colorizer = rs.colorizer()  
  
# 定义ROI（这里是一个示例，你可能需要根据你的需求调整）  
roi = (100, 100, 300, 200)  # (x, y, width, height)  
  
# 获取帧  
frames = pipeline.wait_for_frames()  
depth_frame = frames.get_depth_frame()  
color_frame = frames.get_color_frame()  
  
if not depth_frame or not color_frame:  
    print("No valid frames detected!")  
    exit()  
  
# 将深度帧转换为点云  
points = rs.depth_frame_to_points(depth_frame, camera_intrinsics=depth_sensor.get_intrinsics())  
  
# 获取彩色图像和深度图像  
color_image = np.asanyarray(color_frame.get_data())  
depth_image = np.asanyarray(depth_frame.get_data())  
  
# 将深度图像归一化到0-255  
depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)  
depth_image_normalized = depth_image_normalized.astype(np.uint8)  
  
# 提取ROI区域的彩色图像和深度图像  
color_roi = color_image[roi[1]:roi[1] + roi[3], roi[0]:roi[0] + roi[2]]  
depth_roi = depth_image_normalized[roi[1]:roi[1] + roi[3], roi[0]:roi[0] + roi[2]]  
  
# 根据深度图像计算ROI区域的点云  
# 注意：我们只对ROI内的点感兴趣，因此需要过滤掉ROI外的点  
points_roi = []  
  
for i in range(roi[1], roi[1] + roi[3]):  
    for j in range(roi[0], roi[0] + roi[2]):  
        # 计算ROI内每个像素对应的点云坐标  
        x, y = j, i  
        z = depth_image[i, j] * depth_scale  
        if z != 0:  # 忽略深度为0的点  
            point = points.get_vertex(x, y)  
            point = (point[0], point[1], point[2])  
            points_roi.append(point)  
  
# 将点云坐标转换为NumPy数组  
points_roi = np.array(points_roi)  
  
# 停止流  
pipeline.stop()  
  
# 可以根据需要进一步处理点云数据，例如使用PCL（Point Cloud Library）  
# 或者可视化点云（例如使用Open3D）  
  
# 这里只是输出ROI区域的点云数量作为示例  
print(f"Number of points in the ROI: {len(points_roi)}")