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


        self.rgb_image_lock = True
        self.depth_image_lock = True


        self.rgb_image_ = msg_Image()
        self.depth_image_ = msg_Image()
        self.imu_data_ = msg_Imu()
        self.points_roi_ = msg_PointCloud2()

        self.bridge = CvBridge()
        self.rgb_image_obtained_ = None
        self.depth_image_obtained_ = None
        self.imu_data_obtained_ = None

        self.camera_color_intrinsics_ = None # 对齐后的深度图与彩色图有相同intrinsics

        # self.cam_color_info_obtain_flag_ = False
        # self.camera_color_info_ = None
        # self.camera_color_matrix_ = None
        # self.dist_color_coeffs_ = None

        # self.cam_depth_info_obtain_flag_ = False
        # self.camera_dpeth_info_ = None
        # self.camera_dpeth_matrix_ = None
        # self.dist_dpeth_coeffs_ = None

    def colorCameraInfoCallback(self,cameraInfo):
        try:
            if self.camera_color_intrinsics_:
                return
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
        while not self.rgb_image_lock:
            # print("Econding:",data.encoding)
            self.rgb_image_lock = True
        if self.rgb_image_lock:
            self.rgb_image_=data
            self.rgb_image_lock = False
            sleep(0.5)
        
    def imageDepthCallback(self,data):
        while not self.depth_image_lock:
            # print("Econding:",data.encoding)
            self.depth_image_lock = True
        if self.depth_image_lock:
            self.depth_image_=data
            self.depth_image_lock = False
            sleep(0.5)

    def pointscloudCallback(self,data):
        pass
    def imuCallback(self,data):
        pass
    
    def getColorFigure(self):
        while not self.rgb_image_lock:
            # print("getColorFigure",self.rgb_image_)
            self.rgb_image_lock = True
        if self.rgb_image_lock:
            # self.rgb_image_obtained_ = copy.deepcopy(self.rgb_image_) 
            try:
                cv_img = self.bridge.imgmsg_to_cv2(self.rgb_image_ , 'bgr8')
                # if not cv_img:  
                #     print("No valid cv_img detected!")  
                # cv2_img = self.bridge.imgmsg_to_cv2(self.rgb_image_obtained_ , self.rgb_image_obtained_.encoding)
                # cv2.imshow('img',cv_img)
                # cv2.waitKey(30)
            except CvBridgeError as e:
                print(e)
                return
            self.rgb_image_lock = False
        return cv_img

    def getDepthFigure(self,skin_mask):
        while not self.depth_image_lock:
            # print("getColorFigure",self.rgb_image_)
            self.depth_image_lock = True
        if self.depth_image_lock:
            # self.rgb_image_obtained_ = copy.deepcopy(self.rgb_image_) 
            try:
                cv_depth_img = self.bridge.imgmsg_to_cv2(self.depth_image_ , self.depth_image_.encoding)
                # cv_depth_img = self.bridge.imgmsg_to_cv2(self.depth_image_ , '16UC1')
                # if not cv_depth_img:  
                #     print("No valid cv_depth_img detected!")  
                # cv2_img = self.bridge.imgmsg_to_cv2(self.rgb_image_obtained_ , self.rgb_image_obtained_.encoding)
                # cv2.imshow('img',cv_depth_img)
                # cv2.waitKey(30)
            except CvBridgeError as e:
                print(e)
                return
            self.depth_image_lock = False
            # print("cv_depth_img",cv_depth_img)

        depth_with_roi = cv2.bitwise_and(cv_depth_img,cv_depth_img, mask = skin_mask)

        return depth_with_roi

    def skinDetect(self, img):
    
        YCrCb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB) #转换至YCrCb空间
        (y,cr,cb) = cv2.split(YCrCb) #拆分出Y,Cr,Cb值
        cr1 = cv2.GaussianBlur(cr, (1,1), 0)
        _, skin = cv2.threshold(cr1, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU) #Ostu处理
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

        
        # 获取连通区域的统计信息
        # num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_dilate)


        # 找到所有的轮廓
        h = cv2.findContours(img_dilate,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) #寻找轮廓

        contour = h[0]
        contour = sorted(contour, key = cv2.contourArea, reverse=True)#已轮廓区域面积进行排序
        contourmax = contour[0][:, 0, :]#保留区域面积最大的轮廓点坐标

        # 获取ROI
        roi = cv2.boundingRect(contour[0])


        bg = np.ones(img.shape, np.uint8) *0

        # print("image shape : ",img.shape)

        # 填充最大的轮廓
        # ret = cv2.drawContours(img_dilate, contour, 0, (255,255,255), cv2.FILLED)

        ret1 = cv2.drawContours(bg,contour,0,(255,255,255),cv2.FILLED) #绘制白色区域，不传list不会填充
        # ret = cv2.drawContours(bg,contour[0],-1,(255,255,255),thickness=-1) #绘制白色轮廓,
        # return ret

        gray2 = cv2.cvtColor(ret1, cv2.COLOR_BGR2GRAY)
        _, mask_white = cv2.threshold(gray2, 100, 255, cv2.THRESH_BINARY) #Ostu处理
        ret = cv2.bitwise_and(img,img, mask = mask_white)

        cv2.rectangle(ret, (roi[0], roi[1]), (roi[0] + roi[2], roi[1] + roi[3]), (255,255,255), 2)

        # cv2.imshow('img0',img_dilate)
        # # cv2.waitKey(0)
        # cv2.imshow('img1',ret)
        # cv2.waitKey(1)
        return ret,roi,mask_white


    def rgb_to_float(self, img_color):
        # ''' Stack uint8 rgb image into a single float array (efficiently) for ros compatibility '''
        r = np.ravel(img_color[:, :, 0]).astype(int)
        g = np.ravel(img_color[:, :, 1]).astype(int)
        b = np.ravel(img_color[:, :, 2]).astype(int)
        color = np.left_shift(r, 16) + np.left_shift(g, 8) + b
        packed = pack('%di' % len(color), *color)
        unpacked = unpack('%df' % len(color), packed)
        return np.array(unpacked)

    def getPonitCloud(self,color_img,depth_img,img_roi):
        roi_x, roi_y, roi_w, roi_h = img_roi

        rgb = self.rgb_to_float(color_img[roi_y:roi_y + roi_h,roi_x: roi_x + roi_w])

        points_roi = []
        # color_roi = []
        trans_matrix = np.array([[0,0,1], [-1, 0,0],[0,-1,0]])
        p=0
        # pointx = []
        # pointy = []
        # pointz = []
        for i in range(roi_y, roi_y + roi_h):  
            for j in range(roi_x, roi_x + roi_w):  
                # 计算ROI内每个像素对应的点云坐标  
                x, y = j, i  
                z = depth_img[i, j]/1000
                if z != 0 and z < 0.6:  # 忽略深度为0的点  
                    point_tmp = rs2.rs2_deproject_pixel_to_point(self.camera_color_intrinsics_, [x, y], z)
                    # point_t = trans_matrix*np.transpose(point)
                    # point = np.transpose(point_t)
                   
                    # pointx.append(point[0])
                    # pointy.append(point[1])
                    # pointz.append(point[2])
                    
                    color = color_img[y, x]
                    b = color[0]
                    g = color[1]
                    r = color[2]
                    a = 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

                    point= [point_tmp[2],-point_tmp[0],-point_tmp[1],rgb]
                    points_roi.append(point) 
                    # color_rgb = color[::-1]  # cv2.imread读取的是BGR格式，转换为RGB格式
                    # color_roi.append(color_rgb)

        # color_roi = np.array(color_roi)

        # points_roi = np.dot(points_roi, trans_matrix.T) # PC 旋转
        

        # 将点云数据转换为numpy数组  
        # points_array = np.array(points_roi, dtype=np.float32).reshape(-1,3)  
        # points_array = np.array(points_roi)
        # color_array = np.array(color_roi, dtype=np.uint8).reshape(-1,3) 

        # point_roi_rgb = np.hstack((points_array,color_array))
        # point_roi_with_color =np.concatenate((points_array,color_array),axis=1)
        # point_roi_with_color = np.hstack((points_array,color_roi))

        # point_roi_rgb = np.array(point_roi_with_color, dtype=np.uint8).reshape(-1) 
        # pointx = np.array(pointx, dtype=np.float32).reshape(-1) 
        # pointy = np.array(pointy, dtype=np.float32).reshape(-1) 
        # pointz = np.array(pointz, dtype=np.float32).reshape(-1) 
        # print("pointx : ",pointx.shape)
        # print("pointy : ",pointy.shape)
        # print("pointz : ",pointz.shape)
        # print("rgb : ",rgb.shape)
        # print("color_roi : ",color_roi.shape)
        # print("point_roi_with_color : ",point_roi_with_color.shape)


        # 将点和颜色信息填充到消息中
        # point_data = []
        # for i in range(len(points_roi)):
        #     x, y, z = points_roi[i]
        #     r, g, b = color_roi[i]
        #     rgb = (int(r) << 16) | (int(g) << 8) | int(b)
        #     point_data.append([x, y, z, color_roi[i]])
        # point_roi_with_color = np.transpose(np.vstack((pointx, pointy, pointz, rgb)))


        # print("point_data : ",point_data[0])
        # print("points_array : ",points_array.shape)
        # print("point_roi_rgb : ",point_roi_with_color.shape)





        pc2_msg = msg_PointCloud2()

        # header = header = std_msgs.header.Header(frame_id="camera_link")
        # 设定点云数据  
        header = rospy.Header()  
        header.stamp = rospy.Time.now()  
        header.frame_id = "camera_link"  # 根据实际情况设置frame_id  

        # points_array = np.array(points_roi, dtype=np.float32).reshape(-1)  
        # fields = [PointField('x', 0, PointField.FLOAT32, 1),
        #             PointField('y', 4, PointField.FLOAT32, 1),
        #             PointField('z', 8, PointField.FLOAT32, 1)]
        # pc2_msg.header = header
        # pc2_msg.fields = fields  
        # pc2_msg.width = len(points_roi)  
        # pc2_msg.height = 1  
        # pc2_msg.is_bigendian = False  
        # pc2_msg.point_step = len(fields) * 4  
        # pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width  
        # pc2_msg.is_dense = True  
        # pc2_msg.data = struct.pack('%sf' % len(points_array), *points_array) 
        # # pc2_msg.data = np.asarray(points_array, np.float32).tostring() 



        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            # PointField(name='rgb', offset=12, datatype=PointField.UINT8, count=3)
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]


        # pc2_msg.header = header
        # pc2_msg.fields = fields  
        # pc2_msg.width = len(points_roi)  
        # pc2_msg.height = 1  
        # pc2_msg.is_bigendian = False  
        # pc2_msg.point_step = 16
        # pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width  
        # pc2_msg.is_dense = False  
        # pc2_msg.data = struct.pack('%sf' % len(point_roi_rgb), *point_roi_rgb) 
        # pc2_msg.data = point_data.tobytes()   #np.asarray(point_data, dtype=np.float32).tobytes()
        # pc2_msg.data = np.float32(point_roi_with_color).tostring()  
        

        # pc2_msg = pc2.create_cloud_xyz32(header, point_roi_with_color)
        pc2_msg = pc2.create_cloud(header,fields, points_roi)


        return pc2_msg


    # def findSuctionPos(self,img,contour):

    def colorImgProcess(self):
        rgb_image = self.getColorFigure()
        self.rgb_image_obtained_, roi, recog_mask = self.skinDetect(rgb_image)
        self.depth_image_obtained_  = self.getDepthFigure(recog_mask)

        # self.depth_image_obtained_ = cv2.normalize(limb_depth_img, None, 0, 255, cv2.NORM_MINMAX)  
        # self.depth_image_obtained_ = self.depth_image_obtained_.astype(np.uint8)  
        self.points_roi_ = self.getPonitCloud(self.rgb_image_obtained_,self.depth_image_obtained_,roi)
        self.pc_pub_.publish(self.points_roi_)

def main():
    
    # sleep(3)
    limb_suction_detec = LimbSuctionPoints()
    
    
    rate = rospy.Rate(10)


    try:
        while not rospy.is_shutdown():
            
            rate.sleep() 
            limb_suction_detec.colorImgProcess()
        # mode =''
        # while mode != 'c':
        #     mode = input("Press c to stop showing the figure...")
        
        #     limb_suction_detec.colorImgProcess()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
