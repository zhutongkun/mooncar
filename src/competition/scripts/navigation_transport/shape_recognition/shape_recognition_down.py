#!/usr/bin/python3
#coding=utf8

# 通过深度图识别物体的外形进行分类
# 具身智能任务赛识别模块 - 第二十八届中国机器人及人工智能大赛
# 支持识别：球体(sphere)、正方体(cube)、圆柱体(cylinder)

from sklearn.linear_model import LinearRegression
import cv2
import os
import tone
import math
import queue
import rospy
import threading
import numpy as np
import sdk.common as common
import message_filters
import transforms3d as tfs
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool,Trigger,TriggerResponse
from servo_msgs.msg import MultiRawIdPosDur
from interfaces.srv import GetRobotPose
from sdk import pid, fps
from servo_controllers.bus_servo_control import set_servos
from kinematics import kinematics_control

def xyz_quat_to_mat(xyz, quat):
    mat = tfs.quaternions.quat2mat(np.asarray(quat))
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def xyz_euler_to_mat(xyz, euler, degrees=True):
    if degrees:
        mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
    else:
        mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def mat_to_xyz_euler(mat, degrees=True):
    t, r, _, _ = tfs.affines.decompose(mat)
    if degrees:
        euler = np.degrees(tfs.euler.mat2euler(r))
    else:
        euler = tfs.euler.mat2euler(r)
    return t, euler

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])

class RgbDepthImageNode:
    def __init__(self):
        rospy.init_node('shape_recognition', anonymous=True)
        self.fps = fps.FPS()
        self.last_shape = "none"
        self.rgb_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.sync = None
        self.moving = False
        self.count = 0 
        self.close = False
        self.endpoint = None
        self.shape = None
        self.calibration_flat = False
        self.calibration_dist = False
        self.pick_state = False
        offset = rospy.get_param('/offset')
        self.shape_dist = rospy.get_param('/shape_dist')
        self.offset_x = offset[0]
        self.offset_y = offset[1]
        self.offset_z = offset[2]

        self.target_shape = "None"
        self.queue = queue.Queue(maxsize=1)
        rospy.set_param('~status', 'start')
        rospy.set_param('~detected_shape', 'None')  # 新增：识别结果输出
        
        self.hand2cam_tf_matrix = [[0.0,0.0,1.0,-0.105],
                                   [-1.0,0.0,0.0,0.0],
                                   [0.0,-1.0,0.0,0.044],
                                   [0.0,0.0,0.0,1.0]]
        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(3)
        
        rospy.set_param('~target_shape', 'box')
        rospy.Service('~pick', Trigger, self.pick_callback)
        rospy.Service('~calibration_flat', Trigger, self.calibration_flat_callback)
        rospy.Service('~calibration_dist', Trigger, self.calibration_dist_callback)
        rospy.Service('~stop', Trigger, self.stop_callback)
        rospy.Service('~start', Trigger, self.start_callback)
        rospy.Service('~colse', Trigger, self.colse_callback)
        rospy.sleep(2)
        
        rospy.wait_for_service('/gemini_camera/set_ldp')
        rospy.ServiceProxy('/gemini_camera/set_ldp', SetBool)(False)
        
        self.line_compensation = LinearRegression()
        shape_flat = rospy.get_param('/shape_flat')
        self.line_compensation.fit([[20],[200],[350]],[[shape_flat[0]],[1],[shape_flat[1]]])
        self.line_depth_compensation = []
        
        for i in range(399):
            self.line_depth_compensation.append(self.line_compensation.predict([[i]]))

    def start_callback(self,msg):
        threading.Thread(target=self.goto_default, args=()).start()
        self.rgb_sub = message_filters.Subscriber('/gemini_camera/rgb/image_raw', RosImage, queue_size=1)
        self.depth_sub = message_filters.Subscriber('/gemini_camera/depth/image_raw', RosImage, queue_size=1)
        self.info_sub = message_filters.Subscriber('/gemini_camera/depth/camera_info', CameraInfo, queue_size=1)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub], 3, 0.03)
        self.sync.registerCallback(self.multi_callback)
        return TriggerResponse(success=True)

    def stop_callback(self,msg):
        self.rgb_sub.unregister()
        self.depth_sub.unregister()
        self.info_sub.unregister()
        return TriggerResponse(success=True)

    def colse_callback(self,msg):
        self.close = True
        return TriggerResponse(success=True)

    def pick_callback(self,msg):
        self.target_shape = rospy.get_param('/shape_recognition/target_shape',"box")
        self.pick_state = True
        rospy.set_param('~pick', True)
        rospy.loginfo('开始形状识别任务，目标形状: %s', self.target_shape)
        return TriggerResponse(success=True)

    def calibration_dist_callback(self,msg):
        self.target_shape = rospy.get_param('/shape_recognition/target_shape',"None")
        set_servos(self.servos_pub, 1, ((1, 500), (2, 500), (3, 150), (4, 130), (5, 500), (10, 200)))
        rospy.sleep(2)
        self.calibration_dist = True
        rospy.set_param('~calibration', True)
        return TriggerResponse(success=True)

    def calibration_flat_callback(self,msg):
        self.target_shape = rospy.get_param('/shape_recognition/target_shape',"box")
        set_servos(self.servos_pub, 1, ((1, 500), (2, 500), (3, 150), (4, 130), (5, 500), (10, 200)))
        rospy.sleep(2)
        self.calibration_flat = True
        rospy.set_param('~calibration', True)
        return TriggerResponse(success=True)
    
    def goto_default(self):
        while not rospy.is_shutdown():
            endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)()
            pose_t = endpoint.pose.position
            pose_r = endpoint.pose.orientation
            self.endpoint = xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z]) 

    def move(self, shape, pose_t, angle):  
        rospy.sleep(0.5)
        pose_t[2] += 0.02
        ret1 = kinematics_control.set_pose_target(pose_t, 85)
        if len(ret1[1]) > 0:
            set_servos(self.servos_pub, 1.5, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]),(5, ret1[1][4])))
            rospy.sleep(1.5)
        pose_t[2] -= 0.05
        ret2 = kinematics_control.set_pose_target(pose_t, 85)
        if angle != 0 and len(ret2[1]) > 0:
            angle = angle % 180
            angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
            angle = 500 + int(1000 * (angle + ret2[3][-1]) / 240)
        else:
            angle = 500
        if len(ret2[1]) > 0:
            set_servos(self.servos_pub, 0.5, ((5, angle),))
            rospy.sleep(0.5)
            set_servos(self.servos_pub, 1, ((1, ret2[1][0]), (2, ret2[1][1]), (3, ret2[1][2]), (4, ret2[1][3]),(5, angle)))
            rospy.sleep(1)
            set_servos(self.servos_pub, 0.6, ((10, 750),))
            rospy.sleep(0.6)
        if len(ret1[1]) > 0:
            set_servos(self.servos_pub, 1, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]),(5, angle)))
            rospy.sleep(1)
        set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
        rospy.sleep(1)
        rospy.set_param('~status', 'stop')
        self.pick_state = False
        self.moving = False

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.queue.empty():
            self.queue.put_nowait((ros_rgb_image, ros_depth_image, depth_camera_info))
            self.image_proc()
        if self.close :
            rospy.signal_shutdown('shutdown')

    def image_proc(self):
        try:
            ros_rgb_image, ros_depth_image, depth_camera_info = self.queue.get(block=True)
            
            if self.pick_state:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)

                ih, iw = depth_image.shape[:2]
                depth_image = depth_image.copy()
                
                for j in range(399):
                    depth_image[j] = depth_image[j]*self.line_depth_compensation[j]
                
                depth_image[:, 0:50] = np.array([[1000,]*50]* 400)
                depth_image[:, 590:640] = np.array([[1000,]*50]* 400)
                depth_image[320:400, :] = np.array([[1000,]*640]* 80)
                
                depth = np.copy(depth_image).reshape((-1, ))
                depth[depth<=0] = 55555
                min_index = np.argmin(depth)
                min_y = min_index // iw
                min_x = min_index - min_y * iw

                min_dist = depth_image[min_y, min_x]
                sim_depth_image = np.clip(depth_image, 0, 300).astype(np.float64) / 300 * 255
                depth_image = np.where(depth_image > min_dist + 17, 0, depth_image)
                sim_depth_image_sort = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255                
                depth_gray = sim_depth_image_sort.astype(np.uint8)
                depth_gray = cv2.GaussianBlur(depth_gray, (3, 3), 0)
                _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
                depth_bit = cv2.erode(depth_bit, np.ones((3, 3), np.uint8))
                depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8))

                contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                shape = 'None'
                contour = None
                
                for obj in contours:
                    if min_dist > self.shape_dist-2:
                        break
                    area = cv2.contourArea(obj)
                    if area < 3000 or area > 15000 or self.moving is True:
                        continue
                    
                    perimeter = cv2.arcLength(obj, True)
                    approx = cv2.approxPolyDP(obj, 0.04 * perimeter, True)
                    CornerNum = len(approx)
                    circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0

                    x, y, w, h = cv2.boundingRect(approx)
                    x_contour_depth = depth_image[y + int(h / 2) - 2: y + int(h / 2) + 2, x: x + w]
                    y_contour_depth = depth_image[y : y + h, x + int(w / 2) - 2: x + int(w / 2)+2 ]
                    
                    x_depth = np.where(x_contour_depth == 0, np.nan, x_contour_depth)
                    y_depth = np.where(y_contour_depth == 0, np.nan, y_contour_depth)
                    x_depth_std = np.nanstd(x_depth)
                    y_depth_std = np.nanstd(y_depth)

                    print(w, h, circularity, CornerNum, x_depth_std, y_depth_std)
                    
                    # 比赛规则：识别球体、正方体、圆柱体
                    if circularity > 0.7 and abs(w - h) < 30 and x_depth_std > 1.0:
                        objType = "sphere"
                        self.shape = 'sphere'
                    elif CornerNum >= 4 and abs(w - h) < 20 and x_depth_std <= 1.0 and y_depth_std <= 1.2:
                        objType = "cube"
                        self.shape = 'cube'
                    elif circularity > 0.5 and x_depth_std > 0.8:
                        objType = "cylinder"
                        self.shape = 'cylinder'
                    elif CornerNum >= 4:
                        objType = "cube"
                        self.shape = 'cube'
                    else:
                        objType = "cylinder"
                        self.shape = 'cylinder'
                    
                    shape = objType
                    print('识别结果:', shape)
                    contour = obj

                    if self.shape == rospy.get_param('/shape_recognition/target_shape',"box"):
                        break
                    else:
                        self.shape = "None"

                # 连续识别确认逻辑
                if self.last_shape == shape and shape != 'None':
                    self.count += 1
                else:
                    self.count = 1
                
                self.last_shape = shape
                
                # 连续3次识别到有效形状，输出识别结果
                if self.count >= 3 and self.shape != 'None':
                    rospy.set_param('~detected_shape', self.shape)
                    
                    # 比赛规则：识别结果映射为中文
                    shape_display = {
                        'sphere': '球体',
                        'cube': '正方体',
                        'cylinder': '圆柱体',
                        'box': '正方体'
                    }
                    rospy.set_param('/shape_recognition/display_shape', shape_display.get(self.shape, self.shape))
                    rospy.loginfo('识别完成！结果: %s', self.shape)
                    
                    if self.target_shape != "None" and self.shape == self.target_shape:
                        self.pick_state = False
                        rospy.set_param('~status', 'stop')
                        rospy.loginfo('识别到目标形状，任务完成')
                    
                    self.count = 0

                self.fps.update()

            elif self.calibration_flat :
                config_path = os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../..')), 'config/config.yaml')
                transition_depth_image = np.zeros((400, 640), dtype=float)
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                
                calibration_20 = float(np.max(depth_image[200])/np.max(depth_image[20]))
                calibration_350 = float(np.max(depth_image[200])/np.max(depth_image[350]))
                calibration_compensation = LinearRegression()
                calibration_compensation.fit([[20],[200],[350]],[[calibration_20],[1],[calibration_350]])
                calibration_depth_compensation = []
                
                for i in range(399):
                    calibration_depth_compensation.append(calibration_compensation.predict([[i]]))
                for j in range(399):
                    transition_depth_image[j] = depth_image[j]*calibration_depth_compensation[j]

                print('正在保存参数shape_flat')
                config = common.get_yaml_data(config_path)
                config['shape_flat'][0] = calibration_20
                config['shape_flat'][1] = calibration_350
                common.save_yaml_data(config, config_path)
                rospy.sleep(2)
                print('保存完毕shape_flat')
                self.rgb_sub.unregister()
                self.depth_sub.unregister()
                self.info_sub.unregister()

            elif self.calibration_dist :
                config_path = os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../..')), 'config/config.yaml')
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)

                ih, iw = depth_image.shape[:2]
                depth_image = depth_image.copy()
                for j in range(399):
                    depth_image[j] = depth_image[j]*self.line_depth_compensation[j]

                depth_image[:, 0:50] = np.array([[1000,]*50]* 400)
                depth_image[:, 590:640] = np.array([[1000,]*50]* 400)
                depth_image[320:400, :] = np.array([[1000,]*640]* 80)
                
                depth = np.copy(depth_image).reshape((-1, ))
                depth[depth<=0] = 55555
                min_index = np.argmin(depth)
                min_y = min_index // iw
                min_x = min_index - min_y * iw

                min_dist = depth_image[min_y, min_x]
                print(min_dist)
                print('正在保存参数shape_dist')
                config = common.get_yaml_data(config_path)
                config['shape_dist'] = float(min_dist)
                common.save_yaml_data(config, config_path)
                rospy.sleep(2)
                print('保存完毕shape_flat')
                self.rgb_sub.unregister()
                self.depth_sub.unregister()
                self.info_sub.unregister()
            else:
                rospy.sleep(0.01)
        except Exception as e:
            rospy.logerr('callback error:', str(e))


if __name__ == "__main__":
    node = RgbDepthImageNode()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
