#!/usr/bin/env python

import rospy
import copy
import numpy as np
import cv2
import sensor_msgs
import std_msgs
import sensor_msgs.point_cloud2 as pc2
import pyrealsense2 as rs2
import struct
import open3d as o3d
import tf
import math

from cv_bridge import CvBridge, CvBridgeError
from time import sleep

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CompressedImage as msg_CompressedImage
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
from sensor_msgs.msg import PointField  
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import CameraInfo
from struct import pack, unpack
from geometry_msgs.msg import Pose, PoseArray,PoseStamped

from scipy.interpolate import interp1d,splrep, splev,UnivariateSpline


from limb_recg.srv import *




class LimbSuctionPoints:
	def __init__(self):
		rospy.init_node('limb_suction_pose_node', anonymous=True)

		rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.colorCameraInfoCallback)
		# rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.depthCameraInfoCallback)
		rospy.Subscriber('/camera/color/image_raw', msg_Image, self.imageColorCallback)
		rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', msg_Image, self.imageDepthCallback)
		rospy.Subscriber('/camera/depth/color/points', msg_PointCloud2, self.pointscloudCallback)
		rospy.Subscriber('/imu/data', msg_Imu, self.imuCallback)
		# rospy.Subscriber('/imu/data', msg_Imu, self.imuCallback)
		self.pc_pub_ = rospy.Publisher('/limb_point_cloud',msg_PointCloud2, queue_size=10)
		self.pc_suction_pub_ = rospy.Publisher('/limb_suction_points',msg_PointCloud2, queue_size=10)
		self.pc_suction_traj_pub_ = rospy.Publisher('/limb_suction_traj',PoseArray, queue_size=10)


		self.rgb_image_lock_ = True
		self.depth_image_lock_ = True


		self.rgb_image_ = msg_Image()
		self.depth_image_ = msg_Image()
		self.imu_data_ = msg_Imu()
		# self.points_roi_msg_ = msg_PointCloud2()
		# self.suction_points_msg_ = msg_PointCloud2()

		self.depth_img_acquire_nums_ = 20
		self.depth_img_acquire_flag_ = False
		self.depth_image_array_ = [self.depth_image_ for x in range(self.depth_img_acquire_nums_)]
		self.depth_image_array_index_ = 0

		self.suction_route_fit_line_ = None

		self.points_roi_ = None
		self.suction_points_ = None
		self.bridge_ = CvBridge()
		self.rgb_image_obtained_ = None
		self.depth_image_obtained_ = None
		self.imu_data_obtained_ = None

		self.camera_color_intrinsics_ = None # 对齐后的深度图与彩色图有相同intrinsics
		self.suction_point_3d_ = None
		

		self.suction_dis_ = 50 # Unit: mm
		self.norm_long_ = 0.032 # Unit: m
		self.norm_short_ = 0.024 # Unit: m
		self.camera_link_ = "camera_link"
		self.fields_ = [
			PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
			PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
			PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
			PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
		]
		# self.cam_color_info_obtain_flag_ = False
		# self.camera_color_info_ = None
		# self.camera_color_matrix_ = None
		# self.dist_color_coeffs_ = None

		# self.cam_depth_info_obtain_flag_ = False
		# self.camera_dpeth_info_ = None
		# self.camera_dpeth_matrix_ = None
		# self.dist_dpeth_coeffs_ = None
		sleep(0.1)


	def colorCameraInfoCallback(self,cameraInfo):
		if self.camera_color_intrinsics_:
			return
		try:
			self.camera_color_intrinsics_ = rs2.intrinsics()
			self.camera_color_intrinsics_.width = cameraInfo.width
			self.camera_color_intrinsics_.height = cameraInfo.height
			self.camera_color_intrinsics_.ppx = cameraInfo.K[2]
			self.camera_color_intrinsics_.ppy = cameraInfo.K[5]
			self.camera_color_intrinsics_.fx = cameraInfo.K[0]
			self.camera_color_intrinsics_.fy = cameraInfo.K[4]
			if cameraInfo.distortion_model == 'plumb_bob':
				self.camera_color_intrinsics_.model = rs2.distortion.brown_conrady
			elif cameraInfo.distortion_model == 'equidistant':
				self.camera_color_intrinsics_.model = rs2.distortion.kannala_brandt4
			self.camera_color_intrinsics_.coeffs = [i for i in cameraInfo.D]
			print("img width*height :",self.camera_color_intrinsics_.width, self.camera_color_intrinsics_.height)
		except CvBridgeError as e:
			print(e)
			return

	def depthCameraInfoCallback(self,msg):
		if not self.cam_depth_info_obtain_flag_:
			self.camera_dpeth_info_ = msg
			self.camera_dpeth_matrix_ = np.array(self.camera_dpeth_info_.K).reshape((3, 3))
			# self.dist_dpeth_coeffs_ = np.array(self.camera_dpeth_info_.D)
			self.cam_depth_info_obtain_flag_ = True
		return

	def imageColorCallback(self,data):
		while not self.rgb_image_lock_:
			# print("Econding:",data.encoding)
			self.rgb_image_lock_ = True
		if self.rgb_image_lock_:
			self.rgb_image_=data
			self.rgb_image_lock_ = False
			# sleep(0.5)
		
	def imageDepthCallback(self,data):
		while not self.depth_image_lock_:
			# print("Econding:",data.encoding)
			self.depth_image_lock_ = True
		if self.depth_image_lock_:
			# self.depth_image_=data
			self.depth_image_array_[self.depth_image_array_index_] = data
			self.depth_image_lock_ = False
			self.depth_image_array_index_ +=1
			if self.depth_image_array_index_ == self.depth_img_acquire_nums_:
				self.depth_img_acquire_flag_ = True
				self.depth_image_array_index_ =0
				
			# sleep(0.5)

	def getDepthFlag(self):
		return self.depth_img_acquire_flag_

	def resetDepthAcquire(self):
		self.depth_img_acquire_flag_ = False
		self.depth_image_array_index_ = 0

	def pointscloudCallback(self,data):
		pass
	def imuCallback(self,data):
		pass
	
	def getColorFigure(self):
		while not self.rgb_image_lock_:
			# print("getColorFigure",self.rgb_image_)
			self.rgb_image_lock_ = True
		if self.rgb_image_lock_:
			# self.rgb_image_obtained_ = copy.deepcopy(self.rgb_image_) 
			try:
				cv_img = self.bridge_.imgmsg_to_cv2(self.rgb_image_ , 'bgr8')
				# if not cv_img:  
				#	 print("No valid cv_img detected!")  
				# cv2_img = self.bridge_.imgmsg_to_cv2(self.rgb_image_obtained_ , self.rgb_image_obtained_.encoding)
				# cv2.imshow('img',cv_img)
				# cv2.waitKey(30)
			except CvBridgeError as e:
				print(e)
				return
			self.rgb_image_lock_ = False
		return cv_img

	def getDepthFigure(self,skin_mask):
		while not self.depth_image_lock_:
			# print(" self.depth_image_.encoding: ", self.depth_image_.encoding)
			self.depth_image_lock_ = True
		if self.depth_image_lock_:
			# self.rgb_image_obtained_ = copy.deepcopy(self.rgb_image_) 

			try:
				# cv_depth_img = self.bridge_.imgmsg_to_cv2(self.depth_image_ , self.depth_image_.encoding)
				for i in range(self.depth_img_acquire_nums_):
					if i==0:
						cv_depth_img_sum = self.bridge_.imgmsg_to_cv2(self.depth_image_array_[i] , self.depth_image_array_[0].encoding)
					else:
						cv_depth_img_sum += self.bridge_.imgmsg_to_cv2(self.depth_image_array_[i] , self.depth_image_array_[0].encoding)

				cv_depth_img = cv_depth_img_sum/self.depth_img_acquire_nums_


				# cv_depth_img = self.bridge_.imgmsg_to_cv2(self.depth_image_ , '16UC1')
				# if not cv_depth_img:  
				#	 print("No valid cv_depth_img detected!")  
				# cv2_img = self.bridge_.imgmsg_to_cv2(self.rgb_image_obtained_ , self.rgb_image_obtained_.encoding)
				# cv2.imshow('img',cv_depth_img)
				# cv2.waitKey(30)
			except CvBridgeError as e:
				print(e)
				return
			self.depth_image_lock_ = False
			# print("cv_depth_img",cv_depth_img)


		# # 确保深度图像是浮点型  
		cv_depth_img = cv_depth_img.astype(np.float32)
		
		# # 应用高斯滤波  
		# blurred_depth = cv2.GaussianBlur(depth_image, (5, 5), 0)  
		
		# # 如果需要，将过滤后的深度图像转换回8位图像  
		# blurred_depth_8bit = (blurred_depth * 255).astype(np.uint8)  


		# 应用双边滤波  
		diameter = 3  # 滤波器的直径  
		sigmaColor = 75  # 颜色空间的标准差  
		sigmaSpace = 75  # 坐标空间的标准差  
		cv_depth_img = cv2.bilateralFilter(cv_depth_img, diameter, sigmaColor, sigmaSpace)

		cv_depth_img = cv_depth_img.astype(np.uint16)

		depth_with_roi = cv2.bitwise_and(cv_depth_img,cv_depth_img, mask = skin_mask)

		return depth_with_roi

	def skinDetect(self, img):
	
		YCrCb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB) #转换至YCrCb空间
		(y,cr,cb) = cv2.split(YCrCb) #拆分出Y,Cr,Cb值
		cr1 = cv2.GaussianBlur(cr, (3,3), 0)
		_, skin = cv2.threshold(cr1, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU) #Ostu处理
		# print("skin:",skin)
		res = cv2.bitwise_and(img,img, mask = skin)
		# cv2.imshow('img',res)
		# cv2.waitKey(30)
		# return res
		gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		_, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

		# gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
		# dst = cv2.Laplacian(gray, cv2.CV_16S, ksize = 3)
		# Laplacian = cv2.convertScaleAbs(dst)

		# 定义腐蚀的核
		kernel = np.ones((5,5),np.uint8)
		img_erode = cv2.erode(binary, kernel, iterations=1, borderType=None, borderValue=None)

		# 膨胀操作
		kernel = np.ones((3,3),np.uint8) 
		img_dilate = cv2.dilate(img_erode,kernel,iterations = 1)

		kernel = np.ones((5,5),np.uint8)
		img_erode2 = cv2.erode(img_dilate, kernel, iterations=1, borderType=None, borderValue=None)

		# 获取连通区域的统计信息
		# num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_dilate)


		# 找到所有的轮廓
		h = cv2.findContours(img_erode2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) #寻找轮廓

		contour = h[0]
		contour = sorted(contour, key = cv2.contourArea, reverse=True)#已轮廓区域面积进行排序
		contourmax = contour[0][:, 0, :]#保留区域面积最大的轮廓点坐标

		# 获取ROI
		roi = cv2.boundingRect(contour[0])


		bg = np.ones(img.shape, np.uint8) *0

		# print("image shape : ",img.shape)

		# 填充最大的轮廓
		# limb_color_img = cv2.drawContours(img_dilate, contour, 0, (255,255,255), cv2.FILLED)

		ret1 = cv2.drawContours(bg,contour,0,(255,255,255),cv2.FILLED) #绘制白色区域，不传list不会填充
		# limb_color_img = cv2.drawContours(bg,contour[0],-1,(255,255,255),thickness=-1) #绘制白色轮廓,
		# return limb_color_img

		limb_color_gray2 = cv2.cvtColor(ret1, cv2.COLOR_BGR2GRAY)
		_, mask_white = cv2.threshold(limb_color_gray2, 100, 255, cv2.THRESH_BINARY) #Ostu处理
		limb_color_img = cv2.bitwise_and(img,img, mask = mask_white)

		# print("mask_white : ",mask_white)

		# cv2.rectangle(limb_color_img, (roi[0], roi[1]), (roi[0] + roi[2], roi[1] + roi[3]), (255,255,255), 2)

		# cv2.imshow('img0',res)
		# # cv2.waitKey(0)
		# cv2.imshow('img1',limb_color_img)
		# cv2.waitKey(0)
		return limb_color_img,roi,mask_white,limb_color_gray2


	def rgb_to_float(self, img_color):
		# ''' Stack uint8 rgb image into a single float array (efficiently) for ros compatibility '''
		r = np.ravel(img_color[:, :, 0]).astype(int)
		g = np.ravel(img_color[:, :, 1]).astype(int)
		b = np.ravel(img_color[:, :, 2]).astype(int)
		color = np.left_shift(r, 16) + np.left_shift(g, 8) + b
		packed = pack('%di' % len(color), *color)
		unpacked = unpack('%df' % len(color), packed)
		return np.array(unpacked)

	def getLink8ToProbeEE(self):
		listener = tf.TransformListener()

		listener.waitForTransform('/panda_link8', '/probe_ee', rospy.Time(), rospy.Duration(4.0))
		
		try:
			(trans1,q) = listener.lookupTransform('/panda_link8', '/probe_ee',rospy.Time(0)) # trans1:x,y,z; rot1:x,y,z,w
			roteuler1=tf.transformations.euler_from_quaternion(q,axes='rxyz')

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

		return trans1,q


	def getPonitCloud(self,color_img,depth_img,img_roi):
		roi_x, roi_y, roi_w, roi_h = img_roi

		rgb = self.rgb_to_float(color_img[roi_y:roi_y + roi_h,roi_x: roi_x + roi_w])

		points_roi = []
		trans_matrix = np.array([[0,0,1], [-1, 0,0],[0,-1,0]])

		for i in range(roi_y, roi_y + roi_h):  
			for j in range(roi_x, roi_x + roi_w):  
				# 计算ROI内每个像素对应的点云坐标  
				x, y = j, i  
				z = depth_img[i, j]/1000
				if z > 0.25 and z < 0.6:  # 忽略深度为0的点  
					point_tmp = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
					
					color = color_img[y, x]
					b = color[0]
					g = color[1]
					r = color[2]
					a = 255
					rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

					point= [point_tmp[2],-point_tmp[0],-point_tmp[1],rgb] # 需要对应坐标系，旋转一下
					points_roi.append(point) 


		pc2_msg = msg_PointCloud2()

		# 设定点云数据  
		header = rospy.Header()  
		header.stamp = rospy.Time.now()  
		header.frame_id = "camera_link"  # 根据实际情况设置frame_id  

		pc2_msg = pc2.create_cloud(header,self.fields_, points_roi)


		return pc2_msg

	def getPonitCloud2(self,color_img,depth_img,img_roi, suction_point):
		roi_x, roi_y, roi_w, roi_h = img_roi

		rgb = self.rgb_to_float(color_img[roi_y:roi_y + roi_h,roi_x: roi_x + roi_w])

		points_roi = []
		trans_matrix = np.array([[0,0,1], [-1, 0,0],[0,-1,0]])

		for i in range(roi_y, roi_y + roi_h):  
			for j in range(roi_x, roi_x + roi_w):  
				# 计算ROI内每个像素对应的点云坐标  
				x, y = j, i  
				z = depth_img[i, j]/1000
				if z != 0 and z < 0.6:  # 忽略深度为0的点  
					point_tmp = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
					
					color = color_img[y, x]
					if [x,y] in suction_point:
						print("Suction Point")
						b = 0
						g = 0
						r = 255
					else:
						b = color[0]
						g = color[1]
						r = color[2]

					a = 255
					rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

					point= [point_tmp[2],-point_tmp[0],-point_tmp[1],rgb] # 需要对应坐标系，旋转一下
					points_roi.append(point) 


		pc2_msg = msg_PointCloud2()

		# 设定点云数据  
		header = rospy.Header()  
		header.stamp = rospy.Time.now()  
		header.frame_id = "camera_link"  # 根据实际情况设置frame_id  

		pc2_msg = pc2.create_cloud(header,self.fields_, points_roi)


		return pc2_msg

	def getPonitCloud3(self,color_img,depth_img,img_roi, suction_point_2d):
		roi_x, roi_y, roi_w, roi_h = img_roi

		rgb = self.rgb_to_float(color_img[roi_y:roi_y + roi_h,roi_x: roi_x + roi_w])

		points_roi = []
		trans_matrix = np.array([[0,0,1], [-1, 0,0],[0,-1,0]])

		suction_points_3d = []
		# print("suction_point_2d : ",suction_point_2d)

		for i in range(roi_y, roi_y + roi_h):  #width
			for j in range(roi_x, roi_x + roi_w):  #height
				# 计算ROI内每个像素对应的点云坐标  
				x, y = j, i  
				z = depth_img[i, j]
				if z != 0 and z < 600:  # 忽略深度为0和大于600 mm的的点 
					point_tmp = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
					
					color = color_img[y, x]
					b = color[0]
					g = color[1]
					r = color[2]
					a = 255
					rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

					if [x,y] in suction_point_2d: # For lists
					# if np.array([x,y]) in  suction_point_2d.all():# For array

						suction_points_3d.append([point_tmp[2]/1000,-point_tmp[0]/1000,-point_tmp[1]/1000])

					point= [point_tmp[2]/1000,-point_tmp[0]/1000,-point_tmp[1]/1000,rgb] # 需要对应坐标系，旋转一下
					points_roi.append(point) 


		# visualizer1 = o3d.visualization.Visualizer()
		# visualizer2 = o3d.visualization.Visualizer()
		# print("points_roi : ",points_roi)

		array_points_roi = np.array(points_roi)
		

		# 创建Open3D的点云对象  
		pcd = o3d.geometry.PointCloud()  
		pcd.points = o3d.utility.Vector3dVector(array_points_roi[:,:3]) 
		filter_pcd,_ = pcd.remove_statistical_outlier(50,0.3)

		# o3d.visualization.draw_geometries([pcd], window_name="pcd")
		
		downpcd = filter_pcd.voxel_down_sample(voxel_size=5e-3)
		points_roi_size = len(downpcd.points)

		array_suction_points_3d_tmp = np.array(suction_points_3d)

		
		# array_suction_points_3d = np.sort(array_suction_points_3d_tmp, axis=0)[::-1]


		# 对第二列进行排序，获取排序后的索引, 并使用排序后的索引对原始数组进行排序
		array_suction_points_3d = array_suction_points_3d_tmp[np.argsort(array_suction_points_3d_tmp[:, 1])[::-1]]
		
		suction_points_size = array_suction_points_3d.shape[0]

		# o3d.visualization.draw_geometries([downpcd], window_name="downpcd")

		# print("suction_points_size : ",suction_points_size)
		# print("points_roi_size : ",points_roi_size)

		downpcd.points.extend(array_suction_points_3d)  # 添加的点的坐标
		# downpcd.points.append(o3d.utility.Vector3dVector(array_suction_points_3d))  # 添加的点的坐标
		# downpcd.points = o3d.core.Tensor(array_suction_points_3d, dtype=o3d.core.Dtype.Float32)

		# o3d.visualization.draw_geometries([downpcd], window_name="downpcd with normals")




		# 估计法线  
		downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

		# 访问法向量
		# normals = np.asarray(downpcd.normals)  # 法向量数组
		# print("normals : ",normals.shape[0])

		suction_pcd = downpcd.select_by_index(list(range(points_roi_size,points_roi_size+suction_points_size)))
		# suction_pcd = downpcd.select_by_index(list(range(1,10)))
		normals = np.asarray(suction_pcd.normals)  # 法向量数组

		geometries = [filter_pcd, suction_pcd] 

		# 可视化点云和法向量
		# o3d.visualization.draw_geometries(geometries,point_show_normal=True)
		# o3d.visualization.draw_geometries([pcd], window_name="法线估计",
		#						   point_show_normal=True,
		#						   width=800,  # 窗口宽度
		#						   height=600)  # 窗口高度

		# mode = input("Press x to show the points:")
		# if mode == 'x':
		# 	print("suction_point_2d : ",suction_point_2d)
		# 	print("array_suction_points_3d : ",array_suction_points_3d)

		# 	visualizer1.create_window()
		# 	visualizer2.create_window()

		# 	# 将点云添加到可视化器中
		# 	visualizer1.add_geometry(downpcd)
		# 	visualizer2.add_geometry(pcd)
		# 	visualizer2.add_geometry(suction_pcd)
			
		# 	# 启动布局
		# 	# visualizer1.get_render_option().background_color = o3d.utility.Vector3d(1, 1, 1)
		# 	visualizer1.get_view_control().change_field_of_view(100)
		# 	visualizer1.run()  # 在第一个窗口显示pcd1
			
		# 	# visualizer2.get_render_option().background_color = o3d.utility.Vector3d(1, 1, 1)
		# 	visualizer2.get_view_control().change_field_of_view(100)
		# 	visualizer2.run()  # 在第二个窗口显示pcd2

		# 	# o3d.visualization.draw_geometries([downpcd],point_show_normal=True,window_name="1",)
		# 	# o3d.visualization.draw_geometries(geometries,point_show_normal=True,window_name="2",)
		# visualizer1.destroy_window()
		# visualizer2.destroy_window()

		pc2_msg = msg_PointCloud2()

		# 设定点云数据  
		header = rospy.Header()  
		header.stamp = rospy.Time.now()  
		header.frame_id = "camera_link"  # 根据实际情况设置frame_id  

		pc2_msg = pc2.create_cloud(header,self.fields_, points_roi)

		self.pc_pub_.publish(pc2_msg)



		return points_roi,array_suction_points_3d,normals


	def getPonitCloud4(self, img_roi, suction_points_3d,suction_point_2d,norms_construction_2d_point):
		roi_x, roi_y, roi_w, roi_h = img_roi

		rgb = self.rgb_to_float(self.rgb_image_obtained_[roi_y:roi_y + roi_h,roi_x: roi_x + roi_w])

		points_roi = []

		# print("norms_construction_2d_point : ",np.array(norms_construction_2d_point))

		norms_construction_3d_points_array =  np.array(np.zeros((len(norms_construction_2d_point),3),dtype=np.float32))

		for j in range(roi_x, roi_x + roi_w):  #height
			for i in range(roi_y, roi_y + roi_h):  #width
				# 计算ROI内每个像素对应的点云坐标  
				x, y = j, i  
				z = self.depth_image_obtained_[i, j]
				if z != 0 and z < 600:  # 忽略深度为0和大于600 mm的的点 
					point_tmp = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
					
					color = self.rgb_image_obtained_[y, x]
					b = color[0]
					g = color[1]
					r = color[2]
					a = 255
					rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

					if [x,y] in norms_construction_2d_point: # For lists
						
						index = norms_construction_2d_point.index([x,y])
						# print("[x,y] : ",x,y,index)
						norms_construction_3d_points_array[index,:]=[point_tmp[2]/1000,-point_tmp[0]/1000,-point_tmp[1]/1000]


					point= [point_tmp[2]/1000,-point_tmp[0]/1000,-point_tmp[1]/1000,rgb] # 需要对应坐标系，旋转一下
					points_roi.append(point) 


		# visualizer1 = o3d.visualization.Visualizer()
		# visualizer2 = o3d.visualization.Visualizer()
		# print("points_roi : ",points_roi)

		array_points_roi = np.array(points_roi)
		

		# 创建Open3D的点云对象  
		pcd = o3d.geometry.PointCloud()  
		pcd.points = o3d.utility.Vector3dVector(array_points_roi[:,:3]) 
		filter_pcd,_ = pcd.remove_statistical_outlier(50,0.3)

		# o3d.visualization.draw_geometries([pcd], window_name="pcd")
		
		downpcd = filter_pcd.voxel_down_sample(voxel_size=5e-3)
		points_roi_size = len(downpcd.points)

		array_suction_points_3d_tmp = np.array(suction_points_3d)
		
		# array_suction_points_3d = np.sort(array_suction_points_3d_tmp, axis=0)[::-1]


		# 对第二列进行排序，获取排序后的索引, 并使用排序后的索引对原始数组进行排序
		array_suction_points_3d = array_suction_points_3d_tmp[np.argsort(array_suction_points_3d_tmp[:, 1])[::-1]]
		
		suction_points_size = array_suction_points_3d.shape[0]

		# o3d.visualization.draw_geometries([downpcd], window_name="downpcd")

		# print("suction_points_size : ",suction_points_size)
		# print("points_roi_size : ",points_roi_size)

		downpcd.points.extend(array_suction_points_3d)  # 添加的点的坐标
		downpcd.points.extend(norms_construction_3d_points_array)  # 添加的点的坐标

		# print("length: ",norms_construction_3d_points_array.shape[0])

		# 建立KDTree
		pcd_tree = o3d.geometry.KDTreeFlann(downpcd)

		normal_vectors=[]
		# 使用半径R近邻，将第1500个点半径（0.02）范围内的点设置为红色
		norm_3d_pts_mean_array = np.array(np.zeros((4*len(norms_construction_2d_point),3),dtype=np.float32))
		for num in range(array_suction_points_3d.shape[0]):
			for i in range(4):
				# print("point: ",norms_construction_3d_points_array[4*num+i,:])
				[_, idx_radius, _] = pcd_tree.search_radius_vector_3d(norms_construction_3d_points_array[4*num+i,:] , 0.005)   # 返回邻域点的个数和索引
				norm_3d_pts_mean_array[4*num+i,:] = np.mean(np.asarray(downpcd.points)[idx_radius[1:], :], axis=0)
				# print("num: ",4*num+i, norm_3d_pts_mean_array[4*num+i,:])

			norm_x = norm_3d_pts_mean_array[4*num+1,:] - norm_3d_pts_mean_array[4*num+0,:]
			norm_y = norm_3d_pts_mean_array[4*num+2,:] - norm_3d_pts_mean_array[4*num+3,:]
			# print("norm_x : ",norm_x)
			# print("norm_y : ",norm_y)
			normal_vector = np.cross(norm_x,norm_y)
			standard_normal_vector = -normal_vector/np.linalg.norm(normal_vector)
			# print("standard_normal_vector : ",standard_normal_vector)

			normal_vectors.append(standard_normal_vector)

		suction_pcd = o3d.geometry.PointCloud()  
		suction_pcd.points = o3d.utility.Vector3dVector(array_suction_points_3d) 
		suction_pcd.normals = o3d.utility.Vector3dVector(np.array(np.multiply(normal_vectors, -1)))

		normals = np.asarray(normal_vectors)  # 法向量数组




		# downpcd.points.extend(array_suction_points_3d)  # 添加的点的坐标
		# # downpcd.points.append(o3d.utility.Vector3dVector(array_suction_points_3d))  # 添加的点的坐标
		# # downpcd.points = o3d.core.Tensor(array_suction_points_3d, dtype=o3d.core.Dtype.Float32)

		# # o3d.visualization.draw_geometries([downpcd], window_name="downpcd with normals")

		# # 估计法线  
		# downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

		# # 访问法向量
		# # normals = np.asarray(downpcd.normals)  # 法向量数组
		# # print("normals : ",normals.shape[0])

		# suction_pcd = downpcd.select_by_index(list(range(points_roi_size,points_roi_size+suction_points_size)))
		# # suction_pcd = downpcd.select_by_index(list(range(1,10)))
		# normals = np.asarray(suction_pcd.normals)  # 法向量数组






		# Display

		# geometries = [filter_pcd, suction_pcd] 
		# o3d.visualization.draw_geometries(geometries,point_show_normal=True)

		# 可视化点云和法向量
		# o3d.visualization.draw_geometries(geometries,point_show_normal=True)
		# o3d.visualization.draw_geometries([pcd], window_name="法线估计",
		#						   point_show_normal=True,
		#						   width=800,  # 窗口宽度
		#						   height=600)  # 窗口高度

		# mode = input("Press x to show the points:")
		# if mode == 'x':
		# 	print("suction_point_2d : ",suction_point_2d)
		# 	print("array_suction_points_3d : ",array_suction_points_3d)

		# 	visualizer1.create_window()
		# 	visualizer2.create_window()

		# 	# 将点云添加到可视化器中
		# 	visualizer1.add_geometry(downpcd)
		# 	visualizer2.add_geometry(pcd)
		# 	visualizer2.add_geometry(suction_pcd)
			
		# 	# 启动布局
		# 	# visualizer1.get_render_option().background_color = o3d.utility.Vector3d(1, 1, 1)
		# 	visualizer1.get_view_control().change_field_of_view(100)
		# 	visualizer1.run()  # 在第一个窗口显示pcd1
			
		# 	# visualizer2.get_render_option().background_color = o3d.utility.Vector3d(1, 1, 1)
		# 	visualizer2.get_view_control().change_field_of_view(100)
		# 	visualizer2.run()  # 在第二个窗口显示pcd2

		# 	# o3d.visualization.draw_geometries([downpcd],point_show_normal=True,window_name="1",)
		# 	# o3d.visualization.draw_geometries(geometries,point_show_normal=True,window_name="2",)
		# visualizer1.destroy_window()
		# visualizer2.destroy_window()

		pc2_msg = msg_PointCloud2()

		# 设定点云数据  
		header = rospy.Header()  
		header.stamp = rospy.Time.now()  
		header.frame_id = "camera_link"  # 根据实际情况设置frame_id  

		pc2_msg = pc2.create_cloud(header,self.fields_, points_roi)

		self.pc_pub_.publish(pc2_msg)



		return points_roi,array_suction_points_3d,normals




	def getSuctionRoute(self,img_roi,rgb_gray):
		roi_x, roi_y, roi_w, roi_h = img_roi

		# gray_values = np.array(image)
		
		# # 获取灰度值大于1的点的坐标
		# coordinates = np.argwhere(gray_values > 1)

		points = []
		for j in range(roi_x, roi_x + roi_w): 
			zone_y = rgb_gray[:,j]>0
			coordinates = np.argwhere(zone_y == True)
			coordinates=np.array(coordinates)

			if coordinates.shape[0] > 50:
				line_x_center_y = int(np.median(coordinates))
				points.append([j,line_x_center_y])

		points_arr = np.array(points)

		# 平滑滤波
		# windowsize = 50
		# window = np.ones(int(windowsize)) / float(windowsize)

		# start_point_array=np.array(points_arr[0,:])

		# start_points_array = np.repeat(points_arr[0:1], windowsize, axis=0)
		# end_points_array = np.repeat(points_arr[roi_w-1:roi_w], windowsize, axis=0)


		# points_arr = np.concatenate((start_points_array, points_arr), axis=0)
		# points_arr = np.concatenate((points_arr, end_points_array), axis=0)

		# col_y = points_arr[:,1]
		# col_y_after_filter = np.convolve(col_y, window, 'same')
		
		# points_arr[:,1] = col_y_after_filter

		# points_arr =points_arr[windowsize:windowsize+roi_w,:]

		# print("points_arr : ",points_arr.shape)


		# 插值曲线拟合
		# points_arr1 = points_arr.copy()
		# points_arr2 = np.array(points)
		# # 使用线性插值生成插值函数f_linear
		# f_linear = interp1d(points_arr2[:,0], points_arr2[:,1], kind='linear')
		# # 使用样条插值生成插值函数f_spline
		# # f_spline = interp1d(points_arr[:,0], points_arr[:,1], kind='cubic')
		
		# # 使用线性插值函数f_linear对x_new进行插值，得到y_linear
		# points_arr1[:,1] = f_linear(points_arr1[:,0])
		# # 使用样条插值函数f_spline对x_new进行插值，得到y_spline
		# # points_arr1[:,1] = f_spline(points_arr1[:,0])
		# # y_spline = f_spline(x_new)

		# B样条曲线拟合
		# points_control = np.array([points[i] for i in range(0, len(points), int(roi_w/5))])

		# # 计算B样条曲线的节点和系数
		# tck = splrep(points_control[:,0], points_control[:,1])

		# points_arr1 = points_arr.copy()
		
		# # 计算插值后的新数据点的值
		# points_arr1[:,1] = splev(points_arr1[:,0], tck)


		# 贝塞尔曲线拟合
		points_control = np.array([points[i] for i in range(0, len(points), int(roi_w/5))])
		# points_arr1 = points_arr.copy()

		# 计算B样条曲线的节点和系数
		self.suction_route_fit_line_ = UnivariateSpline(points_control[:,0], points_control[:,1], k=3)
		points_arr[:,1] = self.suction_route_fit_line_(points_arr[:,0])

		# print("points_arr : ",points_arr.transpose())

		# 验证
		# # print("points : ",points)
		# color_image = cv2.cvtColor(rgb_gray, cv2.COLOR_GRAY2BGR)
		# # color_image = copy.deepcopy(self.rgb_image_obtained_)
		# for point2 in points_arr:
		# 	 cv2.circle(color_image, point2, 2, (0,255,0), 1)
		# # for point in points_arr1:
		# # 	 cv2.circle(color_image, point, 2, (255,0,0), 1)
		# # for point3 in points_arr:
		# # 	 cv2.circle(color_image, point3, 2, (0,0,255), 1)

		# cv2.imshow('img2',color_image)
		# cv2.waitKey(30)
		return points_arr.tolist()


	def getSuctionPoints(self,route_2d,depth_img,dis):

		points_3d = []
		for point2d in route_2d:
			x, y = point2d[0],point2d[1]
			z = depth_img[y, x]/1000
			if z!=0:
				point_3d = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
				points_3d.append(point_3d)
				# print("point2d : ",point2d)
				# print("z : ",z)
				# print("point_3d : ",point_3d)
				# input()


		suction_points = []
		msg_points = []

		anchor_pose = points_3d[0]

		for point_pose in points_3d:
			# if np.sqrt(np.sum((np.array(point_pose) - np.array(anchor_pose)) ** 2)) >= dis:
			if np.sqrt(np.sum((np.array([point_pose[:2]]) - np.array(anchor_pose[:2])) ** 2)) >= dis:
				# print("dis : ",np.sqrt(np.sum((np.array([point_pose[:2]]) - np.array(anchor_pose[:2])) ** 2)))
				# print("anchor_pose : ",anchor_pose)
				# print("point_pose : ",point_pose)
				anchor_pose = point_pose
				rgb = struct.unpack('I', struct.pack('BBBB', 0, 0, 255, 255))[0]
				suction_point = [point_pose[2],-point_pose[0],-point_pose[1]] 
				suction_points.append(suction_point)
				msg_point = [point_pose[2],-point_pose[0],-point_pose[1],rgb] 
				msg_points.append(msg_point)


		# print("suction_points : ",suction_points)

		pc2_msg = msg_PointCloud2()

		# 设定点云数据  
		header = rospy.Header()  
		header.stamp = rospy.Time.now()  
		header.frame_id = "camera_link"  # 根据实际情况设置frame_id  

		pc2_msg = pc2.create_cloud(header,self.fields_, msg_points)


		return pc2_msg,suction_points
		# return pc2_msg,msg_points[:,:3]


	def getSuctionPoints2(self,route_2d,depth_img,dis): # dis unit: mm

		points_3d = []
		suction_points_2d = []
		
		anchor_num = 0
		num = 0
		for point2d in route_2d:
			x, y = point2d[0],point2d[1]
			z = depth_img[y, x]
			point_3d = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
			points_3d.append(point_3d)

			anchor_pose = points_3d[anchor_num]
			# if np.sqrt(np.sum((np.array([point_3d[:2]]) - np.array(anchor_pose[:2])) ** 2)) >= dis:
			if np.sqrt(np.sum((np.array([point_3d[0:1]]) - np.array(anchor_pose[0:1])) ** 2)) >= dis:
				anchor_num = num
				suction_points_2d.append(point2d)
			num+=1
		

		# print("suction_point_2d : ",suction_point_2d)

		# color_image = copy.deepcopy(self.rgb_image_obtained_)
		# cv2.imshow('img3',color_image)
		# cv2.waitKey(30)
		# for point in suction_point_2d:
		# 	 cv2.circle(color_image, point, 5, (255,0,0), 1)

		# cv2.imshow('img4',color_image)
		# cv2.waitKey(30)

		return suction_points_2d

	def getSuctionPoints3(self,route_2d,depth_img,dis): #deprecated

		points_3d = []
		for point2d in route_2d:
			x, y = point2d[0],point2d[1]
			z = depth_img[y, x]/1000
			point_3d = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
			points_3d.append(point_3d)


		suction_points = []
		msg_points = []

		anchor_pose = points_3d[0]

		for point_pose in points_3d:
			# if np.sqrt(np.sum((np.array(point_pose) - np.array(anchor_pose)) ** 2)) >= dis:
			if np.sqrt(np.sum((np.array([point_pose[:2]]) - np.array(anchor_pose[:2])) ** 2)) >= dis:
				anchor_pose = point_pose
				rgb = struct.unpack('I', struct.pack('BBBB', 0, 0, 255, 255))[0]
				suction_point = [point_pose[2],-point_pose[0],-point_pose[1]] 
				suction_points.append(suction_point)
				msg_point = [point_pose[2],-point_pose[0],-point_pose[1],rgb] 
				msg_points.append(msg_point)

		# print("suction_points : ",suction_points)

		# 创建Open3D的点云对象  
		pcd = o3d.geometry.PointCloud()  
		pcd.points = o3d.utility.Vector3dVector(suction_points)  

		# 估计法线  
		pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

		# 可视化点云和法向量
		o3d.visualization.draw_geometries([pcd], window_name="法线估计",
								  point_show_normal=True,
								  width=800,  # 窗口宽度
								  height=600)  # 窗口高度

		pc2_msg = msg_PointCloud2()

		# 设定点云数据  
		header = rospy.Header()  
		header.stamp = rospy.Time.now()  
		header.frame_id = "camera_link"  # 根据实际情况设置frame_id  

		pc2_msg = pc2.create_cloud(header,self.fields_, msg_points)

		return pc2_msg,suction_points


	def getSuctionPoints4(self,route_2d,depth_img,dis): # dis unit: mm

		points_3d = []
		suction_points_2d = []
		suction_points_3d = []
		anchor_num = 0
		num = 0
		for point2d in route_2d:
			if point2d[0] < 840 and point2d[1] < 480 :
				x, y = point2d[0],point2d[1]
				z = depth_img[y, x]
				point_3d = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
				points_3d.append(point_3d)

				anchor_pose = points_3d[anchor_num]
				if np.sqrt(np.sum((np.array([point_3d[:2]]) - np.array(anchor_pose[:2])) ** 2)) >= dis:
				# if np.sqrt(np.sum((np.array([point_3d[0:1]]) - np.array(anchor_pose[0:1])) ** 2)) >= dis:
					if num >10 and num < len(route_2d)-50:
						anchor_num = num
						# print("anchor_num : ",anchor_num)
						# print("dis : ",np.sqrt(np.sum((np.array([point_3d[:2]]) - np.array(anchor_pose[:2])) ** 2)))
						suction_points_2d.append(point2d)
						suction_points_3d.append([point_3d[2]/1000,-point_3d[0]/1000,-point_3d[1]/1000])
				num+=1
		
		norm_construction_points = np.array(np.zeros((4*len(suction_points_2d),2),dtype=np.int16))
		for point_num in range(len(suction_points_2d)):
			norm_construction_points[4*point_num+0,0] = suction_points_2d[point_num][0] - math.ceil(self.norm_long_*self.camera_color_intrinsics_.fx)
			norm_construction_points[4*point_num+1,0] = suction_points_2d[point_num][0] + math.ceil(self.norm_long_*self.camera_color_intrinsics_.fx)
			norm_construction_points[4*point_num+0:4*point_num+2,1] = self.suction_route_fit_line_(norm_construction_points[4*point_num+0:4*point_num+2,0])

			# Calculate the bias angles
			if abs(norm_construction_points[4*point_num+1,1] - norm_construction_points[4*point_num+0,1]) > 5:
				tan_theta = math.atan((norm_construction_points[4*point_num+1,1] - norm_construction_points[4*point_num+0,1])/(norm_construction_points[4*point_num+1,0] - norm_construction_points[4*point_num+0,0]))
			else:
				tan_theta = 0

			up_dx = self.norm_short_*math.sin(tan_theta)*self.camera_color_intrinsics_.fx
			up_dy = self.norm_short_*math.cos(tan_theta)*self.camera_color_intrinsics_.fy

			norm_construction_points[4*point_num+2,0] =  suction_points_2d[point_num][0] + up_dx
			norm_construction_points[4*point_num+2,1] =  suction_points_2d[point_num][1] - up_dy

			norm_construction_points[4*point_num+3,0] =  suction_points_2d[point_num][0] - up_dx
			norm_construction_points[4*point_num+3,1] =  suction_points_2d[point_num][1] + up_dy



		# print("norm_construction_points : ",norm_construction_points)

		# color_image = copy.deepcopy(self.rgb_image_obtained_)
		# for point in norm_construction_points.tolist():
		# 	 cv2.circle(color_image, point, 5, (0,0,255), 1)

		# cv2.imshow('img4',color_image)
		# cv2.waitKey(30)

		return suction_points_3d,suction_points_2d,norm_construction_points.tolist()


	# Not done
	def getRouteFromSuctionPoints(self,route_2d,img_roi,rgb_gray,dis): # dis unit: mm  


		points_3d = []
		suction_points_2d = []
		suction_points_3d = []
		anchor_num = 0
		num = 0

		roi_x, roi_y, roi_w, roi_h = img_roi

		rgb = self.rgb_to_float(self.rgb_image_obtained_[roi_y:roi_y + roi_h,roi_x: roi_x + roi_w])

		points_roi = []


		for j in range(roi_x, roi_x + roi_w):  #height
			for i in range(roi_y, roi_y + roi_h):  #width
				# 计算ROI内每个像素对应的点云坐标  
				x, y = j, i  
				z = self.depth_image_obtained_[i, j]
				if z != 0 and z < 600:  # 忽略深度为0和大于600 mm的的点 
					point_tmp = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
					
					color = self.rgb_image_obtained_[y, x]
					b = color[0]
					g = color[1]
					r = color[2]
					a = 255
					rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

					point= [point_tmp[2]/1000,-point_tmp[0]/1000,-point_tmp[1]/1000,rgb] # 需要对应坐标系，旋转一下
					points_roi.append(point) 

					anchor_pose = points_roi[anchor_num]
					if np.sqrt(np.sum((np.array([point_tmp[:2]]) - np.array(anchor_pose[:2])) ** 2)) >= dis:
						if num >10 and num < len(route_2d)-50:
							anchor_num = num
							suction_points_2d.append([x,y])
							suction_points_3d.append([point_tmp[2]/1000,-point_tmp[0]/1000,-point_tmp[1]/1000])
					num+=1

		array_points_roi = np.array(points_roi)
		
		# 创建Open3D的点云对象  
		pcd = o3d.geometry.PointCloud()  
		pcd.points = o3d.utility.Vector3dVector(array_points_roi[:,:3]) 
		filter_pcd,_ = pcd.remove_statistical_outlier(50,0.3)

		downpcd = filter_pcd.voxel_down_sample(voxel_size=5e-3)
		points_roi_size = len(downpcd.points)

		# 优化吸附点

		array_suction_points_3d_tmp = np.array(suction_points_3d)
		# 对第二列进行排序，获取排序后的索引, 并使用排序后的索引对原始数组进行排序
		array_suction_points_3d = array_suction_points_3d_tmp[np.argsort(array_suction_points_3d_tmp[:, 1])[::-1]]
		
		suction_points_size = array_suction_points_3d.shape[0]

		# 建立KDTree
		pcd_tree = o3d.geometry.KDTreeFlann(downpcd)

		for i in range(array_suction_points_3d.shape[0]):
			[_, idx_radius, _] = pcd_tree.search_radius_vector_3d(array_suction_points_3d[i,:] , 0.005)   # 返回邻域点的个数和索引
			suction_bkup_array = np.asarray(downpcd.points)[idx_radius[1:], :]
			min_index = np.argmin(suction_bkup_array[:, 2])
			tmp_suction_3d_point = suction_bkup_array[min_index,:]
			
			suction_points_2d[i][0] = suction_points_2d[i][0] + math.ceil((tmp_suction_3d_point[0]-array_suction_points_3d[i,0])*self.camera_color_intrinsics_.fy)
			suction_points_2d[i][1] = suction_points_2d[i][1] + math.ceil((tmp_suction_3d_point[1]-array_suction_points_3d[i,1])*self.camera_color_intrinsics_.fx)

			array_suction_points_3d[i,:] = tmp_suction_3d_point










		# Find normals
		norm_construction_points = np.array(np.zeros((4*len(suction_points_2d),2),dtype=np.int16))
		for point_num in range(len(suction_points_2d)):
			norm_construction_points[4*point_num+0,0] = suction_points_2d[point_num][0] - math.ceil(self.norm_long_*self.camera_color_intrinsics_.fx)
			norm_construction_points[4*point_num+1,0] = suction_points_2d[point_num][0] + math.ceil(self.norm_long_*self.camera_color_intrinsics_.fx)
			norm_construction_points[4*point_num+0:4*point_num+2,1] = self.suction_route_fit_line_(norm_construction_points[4*point_num+0:4*point_num+2,0])

			# Calculate the bias angles
			if abs(norm_construction_points[4*point_num+1,1] - norm_construction_points[4*point_num+0,1]) > 5:
				tan_theta = math.atan((norm_construction_points[4*point_num+1,1] - norm_construction_points[4*point_num+0,1])/(norm_construction_points[4*point_num+1,0] - norm_construction_points[4*point_num+0,0]))
			else:
				tan_theta = 0

			up_dx = self.norm_short_*math.sin(tan_theta)*self.camera_color_intrinsics_.fx
			up_dy = self.norm_short_*math.cos(tan_theta)*self.camera_color_intrinsics_.fy

			norm_construction_points[4*point_num+2,0] =  suction_points_2d[point_num][0] + up_dx
			norm_construction_points[4*point_num+2,1] =  suction_points_2d[point_num][1] - up_dy

			norm_construction_points[4*point_num+3,0] =  suction_points_2d[point_num][0] - up_dx
			norm_construction_points[4*point_num+3,1] =  suction_points_2d[point_num][1] + up_dy



		# print("norm_construction_points : ",norm_construction_points)

		# color_image = copy.deepcopy(self.rgb_image_obtained_)
		# for point in norm_construction_points.tolist():
		# 	 cv2.circle(color_image, point, 5, (0,0,255), 1)

		# cv2.imshow('img4',color_image)
		# cv2.waitKey(30)

		return suction_points_3d,suction_points_2d,norm_construction_points.tolist()


	def transferSuctionPose(self,norms,points): # Rotation along the route
		# print("ponits size : ", points.shape)
		br = tf.TransformBroadcaster()
		nums = norms.shape[0]

		listener = tf.TransformListener()
		listener.waitForTransform('/world', '/camera_link', rospy.Time(), rospy.Duration(4.0))

		suction_poses = PoseArray()
		suction_poses.header.frame_id = self.camera_link_ 
		suction_poses.header.stamp = rospy.Time.now()

		angle_roll = 0
		angle_pitch = 0
		angle_yawl = 0

		for num in range(nums):
		# num = 5
			norm = norms[num,:]

			if num != nums-1:
				if points[num+1,1]-points[num,1] ==0:
					angle_roll = 0
				else:
					angle_roll = math.atan((points[num+1,2]-points[num,2])/(points[num+1,1]-points[num,1]))


			if norm[0] != 0: #注意旋转方向
				angle_pitch = -math.atan(norm[2]/norm[0])
				angle_yawl = math.atan(norm[1]/norm[0])
				if abs(angle_pitch) > math.pi/18:
					angle_pitch = angle_pitch/abs(angle_pitch)* math.pi/18
				if abs(angle_yawl) > math.pi/9:
					angle_yawl = angle_yawl/abs(angle_yawl)* math.pi/9
			else:
				angle_pitch = 0
				angle_yawl = 0
			if num ==4 or num ==5:
				print("angle_pitch : ", num,angle_pitch)
				print("angle_yawl : ", num, angle_yawl)

			q11 = tf.transformations.quaternion_from_euler(0, angle_pitch, angle_yawl)
			q12 = tf.transformations.quaternion_from_euler(angle_roll, 0, 0)

			q1 = tf.transformations.quaternion_multiply(q11, q12)

			# q2 = tf.transformations.quaternion_from_euler(0, 0, 0)
			q2 = tf.transformations.quaternion_from_euler(math.pi, -math.pi/2, 0)

			q3 = tf.transformations.quaternion_multiply(q1, q2)

			pose_in_camera_link = self.arrayToPose(points[num,:],q3)

			pose_stamped = geometry_msgs.msg.PoseStamped()
			pose_stamped.header.frame_id = "/camera_link"
			pose_stamped.pose = pose_in_camera_link 

			br.sendTransform((points[num,0], points[num,1], points[num,2]),
							 q3,
							 rospy.Time.now(),
							 "Cam_norm_"+str(num),
							 "camera_link")

			# pose_in_probe = listener.transformPose("/probe_ee", pose_stamped)

			# pose_in_probe.header.frame_id = "/panda_link8" # transform to the panda_link8

			pose_in_world = listener.transformPose("/world", pose_stamped)

			suction_poses.poses.append(pose_in_world.pose)

		return suction_poses


	def arrayToPose(self,pos,quat):
		suction_pose = Pose()

		suction_pose.position.x = pos[0]
		suction_pose.position.y = pos[1]
		suction_pose.position.z = pos[2]

		suction_pose.orientation.x = quat[0]
		suction_pose.orientation.y = quat[1]
		suction_pose.orientation.z = quat[2]
		suction_pose.orientation.w = quat[3]
		return suction_pose

	def colorImgProcess(self):
		# if self.camera_color_intrinsics_:
		#		 return
		rgb_image = self.getColorFigure()

		self.rgb_image_obtained_, roi, recog_mask, rgb_gray_image = self.skinDetect(rgb_image)

		self.depth_image_obtained_  = self.getDepthFigure(recog_mask)

		# The route from the 2D image 
		
		suck_2d_route = self.getSuctionRoute(roi,rgb_gray_image)

		suction_points_3d,suction_points_2d,norm_construction_2d_points = self.getSuctionPoints4(suck_2d_route,self.depth_image_obtained_ ,self.suction_dis_)

		self.points_roi_,suction_points,suction_norms = self.getPonitCloud4(roi,suction_points_3d,suction_points_2d,norm_construction_2d_points)

		suction_poses_msg = self.transferSuctionPose(suction_norms,suction_points)

		return suction_poses_msg





		# skin color obtained
		# self.rgb_image_obtained_, roi, recog_mask, rgb_gray_image = self.skinDetect(rgb_image)





		# test 1
		# suction_point_2d = self.getSuctionPoints2(suck_2d_route,self.depth_image_obtained_ ,self.suction_dis_)

		# self.points_roi_,suction_points,suction_norms = self.getPonitCloud3(self.rgb_image_obtained_,self.depth_image_obtained_,roi,suction_point_2d)


		# suction_poses_msg = self.transferSuctionPose(suction_norms,suction_points)

		# return suction_poses_msg



		# desperated 1
		# self.sendSuctionTraj(1, 15, 2, suction_poses_msg)
		
		# self.suction_points_msg_,self.suction_point_3d_ = self.getSuctionPoints(suck_2d_route,self.depth_image_obtained_ ,self.suction_dis_)

		# self.points_roi_msg_ = self.getPonitCloud(self.rgb_image_obtained_,self.depth_image_obtained_,roi)

		# self.pc_pub_.publish(self.points_roi_msg_)
		# self.pc_suction_pub_.publish(self.suction_points_msg_)
		# self.pc_suction_traj_pub_.publish(suction_poses_msg)

	def sendSuctionTraj(self,suction_mode, suction_force, suction_time, pose_array):
		send_flag = input("Press d to send suction traj: ")

		if send_flag == 'd':
			rospy.wait_for_service('make_suctions')
			try:
				make_suction_client = rospy.ServiceProxy('make_suctions', SuctionPose)
				return make_suction_client(suction_mode, suction_force, suction_time, pose_array)
			except rospy.ServiceException as e:
				print("Make suction service call failed: %s"%e)

def main():
	
	# sleep(3)
	limb_suction_detec = LimbSuctionPoints()
	
	rate = rospy.Rate(10)

	i=0
	input("Press Enter to start the limb recognization...")
	
	try:
		while not rospy.is_shutdown():
			 # input()
			 rate.sleep() 
			 limb_suction_detec.colorImgProcess()


		# mode =''
		# while mode != 'c':
		# 	if i<=10:
		# 		i+=1
		# 	else:
		# 		avg_pose_z = 1
		# 		if limb_suction_detec.getDepthFlag():
		# 			while avg_pose_z > 0.15:
		# 				suction_poses_msg = limb_suction_detec.colorImgProcess()
		# 				tmp_poses = suction_poses_msg.poses
		# 				num = len(tmp_poses)
		# 				sum_pose_z = 0
		# 				for pose in tmp_poses:
		# 					sum_pose_z += pose.position.z
		# 				avg_pose_z = sum_pose_z/num
		# 				print("avg_pose_z : ", avg_pose_z)
					
		# 			limb_suction_detec.sendSuctionTraj(1, 10, 1, suction_poses_msg)
		# 			limb_suction_detec.resetDepthAcquire()
		# 			mode = input("Press c to stop moving...")


			# rate.sleep() 
			

	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()
