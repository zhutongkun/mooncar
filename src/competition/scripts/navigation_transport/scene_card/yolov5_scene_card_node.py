#!/usr/bin/env python3
# encoding: utf-8
# 月球场景卡片 YOLOv5 目标检测
import os
import cv2
import queue
import rospy
import signal
import threading
import numpy as np
import sdk.fps as fps
from sdk import common
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from interfaces.msg import ObjectInfo, ObjectsInfo
from yolov5_trt_scene_card import YoLov5TRT
#此处为必须，用于初始化cuda
import pycuda.driver as cuda
import pycuda.driver as drv 
import pycuda.autoinit
MODE_PATH = os.path.split(os.path.realpath(__file__))[0]


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class Yolov5Node:
    def __init__(self, name):
        rospy.init_node(name)  #初始化节点
        self.start = False     #是否开始识别
        self.running = True    #是否继续循环识别
        self.yolov5 = None     #给yolov5模型预赋一个属性
        self.image_sub = None  #相机节点
        self.image_queue = queue.Queue(maxsize=1) #图像队列
        
        self.engine = rospy.get_param('~engine', 'model/scene_card.engine')
        self.calibration = _as_bool(rospy.get_param('~calibration', False))
        self.lib = rospy.get_param('~lib', 'model/scene_card_libmyplugins.so')
        self.conf_thresh = rospy.get_param('~conf_thresh', 0.8) #置信度
        self.camera_name = rospy.get_param('~camera_name', 'astra_camera')
        self.publish_object_image = _as_bool(rospy.get_param('~publish_object_image', False))
        self.auto_start = _as_bool(rospy.get_param('~auto_start', False))
        self._latest_annotated = None
        self._detect_lock = threading.Lock()
        self.result_image_pub = None
        self.show_annotation = self.calibration or self.publish_object_image
        if self.publish_object_image or self.calibration:
            self._ensure_image_publisher(force=True)
        classes_file = rospy.get_param('~classes_file', 'model/classes.names')
        if classes_file:
            if not os.path.isabs(classes_file):
                classes_file = os.path.join(MODE_PATH, classes_file)
            with open(classes_file, 'r', encoding='utf-8') as fp:
                self.classes = [line.strip() for line in fp if line.strip()]
        else:
            self.classes = rospy.get_param('/classes') #识别的类别
        rospy.set_param('~close', False) #设置此节点close参数，方便后续关闭节点
        rospy.set_param('~shape', 'None') #设置此节点shape参数，方便将识别到的内容传输
        self.close = rospy.get_param('~close') #得到目前close参数
        camera_srv = '/%s/set_ldp' % self.camera_name
        try:
            rospy.wait_for_service(camera_srv, timeout=60)
        except rospy.ROSException:
            rospy.logwarn('等待相机服务超时: %s，将继续尝试订阅图像话题', camera_srv)
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 开始识别
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 关闭识别并退出节点
        rospy.Service('~calibration', Trigger, self.calibration_srv_callback)  # 标定显示
        rospy.set_param('~init_finish', True) #设置初始化状态参数
        signal.signal(signal.SIGINT, self.shutdown)
        if self.result_image_pub is not None:
            rospy.Timer(rospy.Duration(0.03), self._publish_view)
        if self.auto_start:
            rospy.Timer(rospy.Duration(0.5), self._auto_start_callback, oneshot=True)

    def _auto_start_callback(self, _event):
        self._begin_detect()

    def _ensure_image_publisher(self, force=False):
        if self.result_image_pub is not None:
            return
        if not force and not self.show_annotation:
            return
        self.result_image_pub = rospy.Publisher(
            'object_image', Image, queue_size=1, latch=True
        )
        rospy.loginfo('发布检测画面: /%s/object_image', rospy.get_name().lstrip('/'))

    def _begin_detect(self):
        if self.start:
            return
        rospy.loginfo("start yolov5 detect")
        self.close = False
        rospy.set_param('~close', False)
        rospy.set_param('~shape', 'None')
        self.yolov5 = YoLov5TRT(
            os.path.join(MODE_PATH, self.engine),
            os.path.join(MODE_PATH, self.lib),
            self.classes,
            self.conf_thresh,
        )
        drv.Context.pop()
        image_topic = '/%s/rgb/image_raw' % self.camera_name
        rospy.loginfo('订阅相机话题: %s', image_topic)
        self._ensure_image_publisher()
        self.image_sub = rospy.Subscriber(
            image_topic,
            Image,
            self.image_callback,
            queue_size=1,
        )
        self.start = True

    def _annotate_image(self, image, boxes, scores, classid):
        annotated = image.copy()
        for box, cls_conf, cls_id in zip(boxes, scores, classid):
            cls_id = int(cls_id)
            if not 0 <= cls_id < len(self.classes):
                continue
            color = common.colors(cls_id, True)
            common.plot_one_box(
                box,
                annotated,
                color=color,
                label="{}:{:.2f}".format(self.classes[cls_id], cls_conf),
            )
        return annotated

    def _get_display_frame(self):
        with self._detect_lock:
            if self._latest_annotated is not None:
                return self._latest_annotated.copy()
        try:
            return self.image_queue.get(block=False).copy()
        except queue.Empty:
            return np.zeros((480, 640, 3), dtype=np.uint8)

    def _publish_view(self, _event):
        if self.result_image_pub is None:
            return
        self.result_image_pub.publish(
            common.cv2_image2ros(self._get_display_frame(), frame_id='yolov5_scene_card')
        )

    #开始识别
    def calibration_srv_callback(self, msg):
        self.calibration = True
        self.show_annotation = True
        self._ensure_image_publisher()
        return TriggerResponse(success=True)

    def start_srv_callback(self, msg):
        self._begin_detect()
        return TriggerResponse(success=True)
        
    #暂停并退出节点
    def stop_srv_callback(self, msg):
        rospy.loginfo('stop yolov5 detect')
        self.start = False
        self.close = True
        return TriggerResponse(success=True)
    #图像回调函数
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)
        #识别
        self.image_proc()
   
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
    #识别函数
    def image_proc(self):
        # while self.running :
        try:
            if self.close :  #根据参数close判断是否需要关闭此节点
                self.image_sub.unregister()
                # self.yolov5.destroy() 
                rospy.signal_shutdown('shutdown')
            elif self.start:   #根据参数close判断是否需要进行识别
                image = self.image_queue.get(block=True) #得到队列内图像
                h, w = image.shape[:2]  #读取图像高宽
                boxes, scores, classid = self.yolov5.infer(image) #得到识别内容
                for box, cls_conf, cls_id in zip(boxes, scores, classid):
                    cls_id = int(cls_id)
                    if not 0 <= cls_id < len(self.classes):
                        continue
                    rospy.set_param('~shape', self.classes[cls_id])
                    print("shape", self.classes[cls_id])
                if self.show_annotation:
                    if boxes:
                        annotated = self._annotate_image(image, boxes, scores, classid)
                    else:
                        annotated = image.copy()
                    with self._detect_lock:
                        self._latest_annotated = annotated
            else:
                rospy.sleep(0.01)
        except BaseException as e:
            rospy.logerr('yolov5 检测失败: %s', e)

if __name__ == "__main__":
    node = Yolov5Node('yolov5_scene_card')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
