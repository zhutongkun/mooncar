#!/usr/bin/env python3
# encoding: utf-8
# 语音控制导航
import os
import json
import rospy
import signal
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
from xf_mic_asr_offline import voice_play
from geometry_msgs.msg import Twist, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from ros_robot_controller.msg import BuzzerState
from servo_controllers import bus_servo_control
from servo_msgs.msg import MultiRawIdPosDur


# 将rpy转换成qua
def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation


class VoiceControlNavNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.words = None  # 语音识别到的内容
        self.running = True  # 循环检测任务开关
        self.move_base_status = 1  # 导航状态 1 是还在运动中，3是导航完毕
        self.pick_location_time = rospy.get_param('/pick_location_time', 3)  # map节点名
        self.up_ramp_time = rospy.get_param('/up_ramp_time', 3.5)  # map节点名
        self.scene_card_results = []
        self.scene_card_timeout = float(rospy.get_param('~scene_card_timeout', 8.0))
        self.scene_card_retry = int(rospy.get_param('~scene_card_retry', 2))
        self.scene_card_settle_time = float(rospy.get_param('~scene_card_settle_time', 0.8))
        self.scene_card_stop_after_task = rospy.get_param('~scene_card_stop_after_task', True)

        rospy.Service('~pick', Trigger, self.start_pick_callback)  # 夹取测试
        rospy.Service('~place', Trigger, self.start_place_callback)  # 放置测试
        rospy.Service('~detect', Trigger, self.start_detect_callback)  # 检测测试
        rospy.Service('~scene_card', Trigger, self.start_scene_card_callback)  # 月球环境识别测试
        rospy.Service('~back', Trigger, self.start_back_callback)  # 回到起始点测试
        rospy.Service('~test', Trigger, self.test_callback)  # 不通过语音识别启动
        rospy.Service('~aligning', Trigger, self.start_aligning_callback)  # 不通过语音识别启动

        rospy.set_param('~target_shape', 'None')  # 设置目标形状
        rospy.set_param('~status', 'start')  # 设置状态

        self.language = os.environ['ASR_LANGUAGE']  # 读取语言
        self.costmap = '/move_base/local_costmap/costmap'  # costmap节点名
        self.map_frame = rospy.get_param('~map_frame', '/map')  # map节点名
        self.slope_surface = rospy.get_param('~slope_surface', True)  # map节点名
        # 麦轮运动控制节点
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        # 舵机控制
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        # 等待舵机节点开启
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/servo_manager/init_finish') and rospy.get_param(
                        '/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        # 初始状态
        bus_servo_control.set_servos(self.joints_pub, 2,
                                     ((1, 500), (2, 760), (3, 15), (4, 150), (5, 500), (10, 200)))
        rospy.sleep(2)
        # 导航点发布
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # 订阅路径规划返回话题
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_callback)
        # 等待语音识别节点启动
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/voice_control/init_finish'):
                    break
            except:
                rospy.sleep(0.1)

        self.vc_sub = rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        rospy.loginfo('唤醒口令: 小迈小迈')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 开始执行任务 / 开始安全任务')
        # 等待导航启动
        rospy.wait_for_message(self.costmap, OccupancyGrid)

        # ===== 新增：激光雷达和里程计订阅（用于倒车上坡纠偏）=====
        self.left_rear_dist = 2.0   # 后方左侧最近距离
        self.right_rear_dist = 2.0  # 后方右侧最近距离
        self.current_pose = Pose()  # 里程计当前位置
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # ===================================================

        self.play('running')
        signal.signal(signal.SIGINT, self.shutdown)
        # 开始任务
        self.run()

    # ===== 新增：激光雷达回调，提取后方两侧距离 =====
    def scan_callback(self, msg):
        """
        从激光雷达数据中提取后方左右两侧的区域最小距离
        左侧后方：角度 150° ~ 180° (正前方为0°，逆时针为正)
        右侧后方：角度 -150° ~ -180° (或 210° ~ 180°)
        """
        try:
            # 后方左侧区域 (150° ~ 180°)
            left_ranges = []
            for i, angle in enumerate(msg.ranges):
                deg = math.degrees(msg.angle_min + i * msg.angle_increment)
                if 150 <= deg <= 180 and msg.ranges[i] > 0.1:
                    left_ranges.append(msg.ranges[i])
            if left_ranges:
                self.left_rear_dist = min(left_ranges)
            else:
                self.left_rear_dist = 2.0

            # 后方右侧区域 (-150° ~ -180°) 等价于 180° ~ 210°
            right_ranges = []
            for i, angle in enumerate(msg.ranges):
                deg = math.degrees(msg.angle_min + i * msg.angle_increment)
                # 转换到 [0,360) 方便处理
                if deg < 0:
                    deg += 360
                if 180 <= deg <= 210 and msg.ranges[i] > 0.1:
                    right_ranges.append(msg.ranges[i])
            if right_ranges:
                self.right_rear_dist = min(right_ranges)
            else:
                self.right_rear_dist = 2.0
        except Exception as e:
            rospy.logwarn_throttle(5, "scan_callback error: %s", e)

    # ===== 新增：里程计回调，记录当前位置 =====
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    # ===== 新增：基于激光雷达的倒车上坡闭环控制 =====
    def reverse_up_ramp_with_laser(self, distance=1.3, speed=0.25, Kp=0.6):
        """
        倒车上坡，同时利用激光雷达保持车身与坡道平行（两侧距离相等）
        :param distance:  需要倒退的直线距离 (米)
        :param speed:     倒退线速度 (米/秒，正值表示前进，内部取负)
        :param Kp:        比例控制系数
        """
        rospy.loginfo("Starting reverse up ramp with laser correction, distance=%.2f m", distance)

        # 取消 move_base 当前目标，避免潜在冲突
        try:
            self.goal_pub.publish(PoseStamped())  # 发布空目标
            rospy.loginfo("Cancelled move_base goal")
        except Exception as e:
            rospy.logwarn("Failed to cancel move_base goal: %s", e)

        # 记录起始里程计位置
        start_pose = self.current_pose
        traveled = 0.0

        rate = rospy.Rate(20)  # 20Hz 控制频率
        twist = Twist()
        twist.linear.x = -speed   # 倒车

        # 简单比例控制，防止积分饱和
        while not rospy.is_shutdown() and traveled < distance:
            # 计算已移动距离（欧氏距离）
            dx = self.current_pose.position.x - start_pose.position.x
            dy = self.current_pose.position.y - start_pose.position.y
            traveled = math.sqrt(dx*dx + dy*dy)
            rospy.logdebug("Traveled: %.3f / %.3f m", traveled, distance)

            # 横向纠偏：期望两侧距离相等（误差=0）
            error = self.left_rear_dist - self.right_rear_dist
            # 限幅，避免过大转向
            twist.angular.z = Kp * error
            twist.angular.z = max(-0.5, min(0.5, twist.angular.z))

            self.mecanum_pub.publish(twist)
            rate.sleep()

        # 停止机器人
        self.mecanum_pub.publish(Twist())
        rospy.loginfo("Reverse up ramp finished, traveled %.3f m", traveled)

    # ===== 原有方法（未改动）=====
    # 不通过语音进行识别
    def test_callback(self, msg):
        self.words = '开始安全任务'
        return TriggerResponse(success=True)

    def start_pick_callback(self, msg):
        rospy.loginfo("=== start_pick_callback called ===")
        try:
            rospy.wait_for_service('/shape_recognition/start', timeout=3.0)
            rospy.wait_for_service('/shape_recognition/pick', timeout=3.0)
            rospy.wait_for_service('/shape_recognition/stop', timeout=3.0)
        except rospy.ROSException as e:
            rospy.logerr("Shape recognition services not available: %s", e)
            return TriggerResponse(success=False, message="Service unavailable")

        current = rospy.get_param('/shape_recognition/status', 'start')
        if current != 'start':
            rospy.set_param('/shape_recognition/status', 'start')
            rospy.sleep(1.3)

        try:
            rospy.ServiceProxy('/shape_recognition/start', Trigger)()
        except Exception as e:
            rospy.logerr("Failed to call /shape_recognition/start: %s", e)
            return TriggerResponse(success=False, message="start failed")

        rospy.sleep(1.5)

        try:
            rospy.ServiceProxy('/shape_recognition/pick', Trigger)()
        except Exception as e:
            rospy.logerr("Failed to call /shape_recognition/pick: %s", e)
            return TriggerResponse(success=False, message="pick failed")

        rospy.sleep(1.5)

        success = self.wait_pick_status(timeout=60.0)
        if success:
            self.play('7')
        else:
            rospy.logerr("Pick timeout in start_pick_callback")

        try:
            rospy.ServiceProxy('/shape_recognition/stop', Trigger)()
        except Exception as e:
            rospy.logerr("Failed to call /shape_recognition/stop: %s", e)

        rospy.set_param('/shape_recognition/status', 'start')
        rospy.loginfo("=== start_pick_callback finished ===")
        return TriggerResponse(success=success)

    def start_place_callback(self, msg):
        self.control(0, 0, 1, "place")
        return TriggerResponse(success=True)

    def start_detect_callback(self, msg):
        self.control(0, 0, 1, "detect")
        return TriggerResponse(success=True)

    def start_scene_card_callback(self, msg):
        ok = self.run_scene_card_task(report=True)
        return TriggerResponse(success=ok)

    def start_aligning_callback(self, msg):
        rospy.ServiceProxy('/position_correction/start', Trigger)()
        rospy.ServiceProxy('/position_correction/' + 'pick' + "_1", Trigger)()
        rospy.sleep(1)
        self.wait_correction_status()
        rospy.ServiceProxy('/position_correction/' + 'pick' + "_2", Trigger)()
        rospy.sleep(1)
        self.wait_correction_status()
        twist = Twist()
        twist.linear.x = 0.05
        self.mecanum_pub.publish(twist)
        rospy.sleep(float(self.pick_location_time[0]))
        twist = Twist()
        self.mecanum_pub.publish(twist)
        return TriggerResponse(success=True)

    def start_back_callback(self, msg):
        rospy.ServiceProxy('/ramp/start', Trigger)()
        rospy.ServiceProxy('/ramp/up', Trigger)()
        rospy.sleep(1)
        self.wait_ramp_status()
        twist = Twist()
        twist.linear.y = -0.1
        self.mecanum_pub.publish(twist)
        rospy.sleep(0.6)
        twist = Twist()
        twist.angular.z = -0.5
        self.mecanum_pub.publish(twist)
        rospy.sleep(0.2)
        twist = Twist()
        twist.linear.x = 0.3
        self.mecanum_pub.publish(twist)
        rospy.sleep(float(self.up_ramp_time[0]))
        twist = Twist()
        twist.angular.z = 0.5
        self.mecanum_pub.publish(twist)
        rospy.sleep(6)
        self.mecanum_pub.publish(Twist())
        return TriggerResponse(success=True)

    def play(self, name):
        try:
            voice_play.play(name, language=self.language)
        except Exception as e:
            rospy.logwarn('语音播放失败: %s, %s', name, e)

    def call_trigger(self, service_name, timeout=1.5):
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            return rospy.ServiceProxy(service_name, Trigger)()
        except Exception as e:
            rospy.logwarn('服务调用跳过: %s, %s', service_name, e)
            return None

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            self.words = self.words.replace(' ', '')
        print('words:', self.words)

        if self.words is not None and self.words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                                         '失败10次(Fail-10-times']:
            pass
        elif self.words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

    def move_callback(self, msg):
        print(msg)
        try:
            if msg.status.status == 3:
                self.move_base_status = msg.status.status
            else:
                self.move_base_status = 1
        except:
            self.move_base_status = 1

    def nav_position(self, x, y, w):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = rpy2qua(math.radians(0), math.radians(0), math.radians(w))
        self.goal_pub.publish(pose)

    def wait_nav_status(self):
        while True:
            if self.move_base_status == 3:
                self.move_base_status = 1
                break
            else:
                rospy.sleep(2)

    def wait_correction_status(self):
        while True:
            correction_status = rospy.get_param('/position_correction/status', "stop")
            print('correction_status:', correction_status)
            if correction_status == "stop":
                break
            else:
                rospy.sleep(2)

    def wait_ramp_status(self):
        while True:
            correction_status = rospy.get_param('/ramp/status', "stop")
            print('ramp:', correction_status)
            if correction_status == "stop":
                break
            else:
                rospy.sleep(2)

    def wait_pick_status(self, timeout=30.0):
        start_time = rospy.Time.now()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            pick_status = rospy.get_param('/shape_recognition/status', "start")
            rospy.logdebug("grab_status: %s", pick_status)
            if pick_status == "stop":
                rospy.set_param('/shape_recognition/status', "start")
                rospy.loginfo("Pick status changed to stop, reset to start.")
                return True
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("wait_pick_status timeout after %.1f seconds, forcing continue.", timeout)
                rospy.set_param('/shape_recognition/status', "start")
                return False
            rate.sleep()

    def wait_yolo_status(self):
        while True:
            shape = rospy.get_param('/yolov5/shape', 'None')
            print('yolo_shape:', shape)
            if shape != "None":
                break
            else:
                rospy.sleep(0.5)

    def get_scene_card_points(self):
        default_points = [
            {'name': 'scene_card_1', 'x': 1.45, 'y': -1.05, 'w': 90},
            {'name': 'scene_card_2', 'x': 1.45, 'y': -1.55, 'w': 90},
            {'name': 'scene_card_3', 'x': 1.45, 'y': -2.05, 'w': 90},
        ]
        points = rospy.get_param('~scene_card_points', default_points)
        if not isinstance(points, list) or len(points) == 0:
            points = default_points

        normalized = []
        for i, point in enumerate(points[:3]):
            fallback = default_points[i]
            if isinstance(point, dict):
                normalized.append({
                    'name': str(point.get('name', fallback['name'])),
                    'x': float(point.get('x', fallback['x'])),
                    'y': float(point.get('y', fallback['y'])),
                    'w': float(point.get('w', point.get('yaw', fallback['w']))),
                })
            elif isinstance(point, (list, tuple)) and len(point) >= 3:
                normalized.append({
                    'name': fallback['name'],
                    'x': float(point[0]),
                    'y': float(point[1]),
                    'w': float(point[2]),
                })

        while len(normalized) < 3:
            normalized.append(default_points[len(normalized)])
        return normalized[:3]

    def normalize_scene_card(self, raw_shape):
        if raw_shape is None:
            return None
        shape = str(raw_shape).strip().lower()
        if shape in ['', 'none', 'null', 'unknown', 'no']:
            return None
        known = [
            'astronaut', 'lunar_crater', 'meteorite', 'satellite', 'lunar_rover',
            'space_station', 'rocket', 'earth', 'moon', 'lunar_soil',
        ]
        if shape in known:
            return shape
        rospy.logwarn('未知月球环境类别: %s', raw_shape)
        return shape

    def detect_scene_card(self, task_index):
        for attempt in range(max(1, self.scene_card_retry)):
            rospy.set_param('/yolov5_scene_card/shape', 'None')
            rospy.sleep(self.scene_card_settle_time)
            start = rospy.Time.now()
            while not rospy.is_shutdown() and self.running:
                raw_shape = rospy.get_param('/yolov5_scene_card/shape', 'None')
                shape = self.normalize_scene_card(raw_shape)
                if shape:
                    rospy.loginfo('第%d个任务点月球环境识别结果: %s', task_index, shape)
                    return shape
                if (rospy.Time.now() - start).to_sec() > self.scene_card_timeout:
                    break
                rospy.sleep(0.2)
            rospy.logwarn('第%d个任务点第%d次月球环境识别超时', task_index, attempt + 1)
        return 'unknown'

    def run_scene_card_task(self, report=False):
        self.scene_card_results = []
        points = self.get_scene_card_points()

        self.call_trigger('/yolov5/stop', timeout=1.0)
        self.call_trigger('/yolov5_scene_card/start', timeout=3.0)
        try:
            for index, point in enumerate(points, 1):
                rospy.loginfo('导航到第%d个月球环境识别任务点: %s', index, point)
                self.nav_position(point['x'], point['y'], point['w'])
                rospy.sleep(2)
                self.wait_nav_status()
                result = self.detect_scene_card(index)
                self.scene_card_results.append(result)
        finally:
            if self.scene_card_stop_after_task:
                self.call_trigger('/yolov5_scene_card/stop', timeout=1.0)

        if report:
            self.report_scene_card_results()
        return len(self.scene_card_results) == 3

    def report_scene_card_results(self):
        if not self.scene_card_results:
            rospy.logwarn('没有月球环境识别结果，跳过播报')
            return
        for index, result in enumerate(self.scene_card_results, 1):
            name = result if result else 'unknown'
            self.play('scene_card/card%d_%s' % (index, name))
            rospy.sleep(0.25)

    def control(self, x, y, w, set_status):
        # ----- 修改：pick2 跳过通用导航，避免因固定角度导致的转圈 -----
        if set_status != 'pick2':
            self.nav_position(x, y, w)
            rospy.sleep(2)
            self.wait_nav_status()
        # ---------------------------------------------------------

        if set_status == 'pick1':
            rospy.loginfo("Navigating to pick1 point (1.50, -3.12, 0)")
            self.nav_position(1.50, -3.12, 0)
            rospy.sleep(2)
            self.wait_nav_status()

            twist = Twist()
            twist.linear.x = 0.108
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())

            self.safe_pick()

            twist = Twist()
            twist.linear.x = -0.12
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())

        elif set_status == 'pick2':
            rospy.loginfo("Navigating to pick2 point (0.84, -3.17)")

            # ----- 修改：容差判断，避免不必要的转圈，同时保证朝向正确 -----
            target_yaw = -180.0   # 期望目标角度
            yaw_tolerance = 10.0  # 容差，单位：度

            try:
                q = self.current_pose.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                current_yaw = math.degrees(yaw_rad)

                # 计算当前航向与目标航向的最小角度差（处理角度环绕）
                diff = abs(current_yaw - target_yaw)
                diff = min(diff, 360.0 - diff)

                if diff <= yaw_tolerance:
                    # 航向已经接近目标，使用当前航向避免转圈
                    rospy.loginfo("Current yaw %.2f is within tolerance (%.1f°), using it to avoid rotation.",
                                  current_yaw, diff)
                    nav_yaw = current_yaw
                else:
                    # 航向偏差较大，需要修正到目标角度
                    rospy.loginfo("Current yaw %.2f differs from target %.2f by %.1f°, will correct.",
                                  current_yaw, target_yaw, diff)
                    nav_yaw = target_yaw
            except Exception as e:
                rospy.logwarn("Failed to get current yaw, using target yaw as fallback: %s", e)
                nav_yaw = target_yaw

            # 发布导航点
            self.nav_position(0.84, -3.17, nav_yaw)
            # ---------------------------------------------------------

            rospy.sleep(2)
            self.wait_nav_status()

            twist = Twist()
            twist.linear.x = 0.24
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())

            self.safe_pick()

            twist = Twist()
            twist.linear.x = -0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(3)
            self.mecanum_pub.publish(Twist())

        elif set_status == "place":
            twist = Twist()
            twist.linear.x = 0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(1.7)
            self.mecanum_pub.publish(Twist())
            rospy.ServiceProxy('/position_correction/' + set_status + "_3", Trigger)()
            rospy.sleep(1)
            self.wait_correction_status()
            self.play('9')
            twist = Twist()
            twist.linear.x = -0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())
        elif set_status == "detect":
            if self.slope_surface is not True:
                self.play('reached_explosion-proof_warehouse')
            else:
                self.play('2')

            rospy.sleep(1)
            self.wait_yolo_status()
            shape = rospy.get_param('/yolov5/shape', 'None')

            if self.slope_surface is not True:
                if shape == "cube":
                    self.play('ruled_out_cuboid_explosive')
                elif shape == "box":
                    self.play('ruled_out_cube_explosive')
                else:
                    self.play('ruled_out_cylinder_explosive')
            else:
                if shape == "cube":
                    self.play('3')
                elif shape == "box":
                    self.play('4')
                else:
                    self.play('5')

            rospy.set_param('/shape_recognition/target_shape', shape)
            rospy.sleep(2)
        elif set_status == "back":
            rospy.set_param('~status', 'stop')
            if self.slope_surface:
                rospy.ServiceProxy('/ramp/up', Trigger)()
                rospy.sleep(1)
                self.wait_ramp_status()
                twist = Twist()
                twist.linear.y = -0.1
                self.mecanum_pub.publish(twist)
                rospy.sleep(0.6)
                twist = Twist()
                twist.angular.z = -0.5
                self.mecanum_pub.publish(twist)
                rospy.sleep(0.1)
                twist = Twist()
                twist.linear.x = 0.3
                self.mecanum_pub.publish(twist)
                rospy.sleep(float(self.up_ramp_time[0]))
                twist = Twist()
                twist.angular.z = 0.5
                self.mecanum_pub.publish(twist)
                rospy.sleep(6)
                self.mecanum_pub.publish(Twist())

    def safe_pick(self):
        rospy.loginfo("=== safe_pick start ===")
        rospy.loginfo("Stopping depth camera (YOLOv5) to free USB bandwidth...")
        try:
            rospy.wait_for_service('/yolov5/stop', timeout=3.0)
            rospy.ServiceProxy('/yolov5/stop', Trigger)()
            rospy.loginfo("Depth camera stopped.")
        except Exception as e:
            rospy.logwarn("Failed to stop depth camera: %s (continuing anyway)", e)

        try:
            rospy.wait_for_service('/shape_recognition/start', timeout=5.0)
            rospy.wait_for_service('/shape_recognition/pick', timeout=5.0)
            rospy.wait_for_service('/shape_recognition/stop', timeout=5.0)
        except rospy.ROSException as e:
            rospy.logerr("Shape recognition services not available: %s", e)
            self._restart_yolo()
            return

        current_status = rospy.get_param('/shape_recognition/status', 'start')
        if current_status != 'start':
            rospy.logwarn("Resetting /shape_recognition/status to 'start' before pick.")
            rospy.set_param('/shape_recognition/status', 'start')
            rospy.sleep(0.5)

        try:
            rospy.ServiceProxy('/shape_recognition/start', Trigger)()
            rospy.loginfo("Called /shape_recognition/start")
        except Exception as e:
            rospy.logerr("Failed to call /shape_recognition/start: %s", e)
            self._restart_yolo()
            return

        rospy.sleep(1.5)

        try:
            rospy.ServiceProxy('/shape_recognition/pick', Trigger)()
            rospy.loginfo("Called /shape_recognition/pick")
        except Exception as e:
            rospy.logerr("Failed to call /shape_recognition/pick: %s", e)
            self._restart_yolo()
            return

        rospy.sleep(1.0)

        success = self.wait_pick_status(timeout=30.0)

        if success:
            self.play('7')
        else:
            rospy.logerr("Pick timeout in safe_pick")

        try:
            rospy.ServiceProxy('/shape_recognition/stop', Trigger)()
            rospy.loginfo("Called /shape_recognition/stop")
        except Exception as e:
            rospy.logerr("Failed to call /shape_recognition/stop: %s", e)

        rospy.set_param('/shape_recognition/status', 'start')
        rospy.sleep(0.5)

        self._restart_yolo()
        rospy.loginfo("=== safe_pick end ===")

    def _restart_yolo(self):
        rospy.loginfo("Restarting depth camera (YOLOv5)...")
        try:
            rospy.wait_for_service('/yolov5/start', timeout=5.0)
            rospy.ServiceProxy('/yolov5/start', Trigger)()
            rospy.loginfo("Depth camera restarted successfully.")
        except Exception as e:
            rospy.logerr("Failed to restart depth camera: %s", e)

    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.words is not None:
                if self.slope_surface is not True:
                    str_data = "开始安全任务"
                else:
                    str_data = "开始执行任务"

                if self.words == str_data:
                    print('>>>>>>>>>>>>>>>>>>>> 开始任务<<<<<<<<<<<<<<<<<')
                    self.vc_sub.unregister()

                    rospy.loginfo("启动摄像头检测...")
                    try:
                        rospy.ServiceProxy('/yolov5/start', Trigger)()
                    except Exception as e:
                        rospy.logerr("启动YOLOv5服务失败: %s", e)

                    self.play('1')
                    twist = Twist()
                    twist.linear.x = 0.3
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(3)
                    self.mecanum_pub.publish(Twist())
                    twist = Twist()
                    twist.angular.z = -0.5
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(3)
                    self.mecanum_pub.publish(Twist())
                    print("go")
                    self.move_base_status = 1

                    self.control(1.5, -0.15, 90, "detect")
                    self.control(0.94, -3.124, 0, "pick1")
                    self.control(1.34, -0.22, 40, "place")
                    self.control(0.94, -3.124, -180, "pick2")   # 这里的 -180 不会被使用，因为 control 开头已跳过通用导航
                    self.control(1.34, -0.22, 40, "place")

                    try:
                        rospy.ServiceProxy('/position_correction/close', Trigger)()
                    except:
                        rospy.logwarn("Failed to call /position_correction/close")
                    try:
                        rospy.ServiceProxy('/shape_recognition/close', Trigger)()
                    except:
                        rospy.logwarn("Failed to call /shape_recognition/close")

                    self.run_scene_card_task(report=False)

                    # ===== 修改后的上坡段：使用激光雷达纠偏倒车 =====
                    if self.slope_surface:
                        rospy.loginfo("导航至坡前点 (1.3, 0, 0)")
                        self.nav_position(1.3, 0, 0)
                        rospy.sleep(2)
                        self.wait_nav_status()

                        rospy.loginfo("开始倒车上坡...")
                        twist = Twist()
                        twist.linear.x = -0.25
                        self.mecanum_pub.publish(twist)
                        rospy.sleep(4.8)
                        self.mecanum_pub.publish(Twist())
                        rospy.loginfo("已到达上坡终点 (0, 0, 0)")
                        self.report_scene_card_results()
                        self.play('11')
                    else:
                        self.control(0.1, -0.3, 0, "back")
                        twist = Twist()
                        twist.linear.y = 0.1
                        self.mecanum_pub.publish(twist)
                        rospy.sleep(3)
                        self.mecanum_pub.publish(Twist())
                        self.report_scene_card_results()
                        self.play('mission_accomplished')

                    # ===============================================

                    rospy.loginfo("关闭摄像头检测...")
                    try:
                        rospy.ServiceProxy('/yolov5/stop', Trigger)()
                    except Exception as e:
                        rospy.logerr("停止YOLOv5服务失败: %s", e)

                    print('>>>>>>>>>>>>>>>>>>>> 结束任务<<<<<<<<<<<<<<<<<')
                elif self.words == '休眠(Sleep)':
                    rospy.sleep(0.01)
                self.words = None
            else:
                rospy.sleep(0.01)

        self.mecanum_pub.publish(Twist())


if __name__ == "__main__":
    VoiceControlNavNode('voice_control_nav')
