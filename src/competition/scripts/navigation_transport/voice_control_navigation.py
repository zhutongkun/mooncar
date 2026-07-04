#!/usr/bin/env python3
# encoding: utf-8
# 语音控制导航 - 具身智能任务版
# 保留原来的唤醒方式和语音指令：小迈小迈；开始执行任务 / 开始安全任务
# 新任务：起点 -> S 型绕障 -> 终点 -> 识别区标志点 -> 识别并播报球体/正方体/圆柱体

import os
import json
import rospy
import signal
import math

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from xf_mic_asr_offline import voice_play
from geometry_msgs.msg import Twist, PoseStamped, Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from ros_robot_controller.msg import BuzzerState
from servo_controllers import bus_servo_control
from servo_msgs.msg import MultiRawIdPosDur


def rpy2qua(roll, pitch, yaw):
    """将 rpy 转换为四元数。"""
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


def qua2yaw(q):
    """四元数转 yaw。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def norm_angle(rad):
    """角度归一化到 [-pi, pi]。"""
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class VoiceControlNavNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.words = None
        self.running = True
        self.move_base_status = 1
        self.current_pose = None       # 当前地图位姿: (x, y, yaw)
        self.start_pose = None         # 任务开始时的起点位姿: (x, y, yaw)
        self.slot_shape_cache = {}      # 识别区槽位 -> 形状缓存

        self.language = os.environ.get('ASR_LANGUAGE', 'Chinese')
        self.costmap = rospy.get_param('~costmap', '/move_base/local_costmap/costmap')
        self.map_frame = rospy.get_param('~map_frame', '/map')

        # 默认使用“相对起点”的比赛尺寸坐标。机器人放在起点区正中并朝向避障区即可。
        # 如果你已经在地图上量好了绝对坐标，可将 ~use_relative_waypoints 设为 false，
        # 再用 ~avoid_waypoints 和 ~marker_slots 传入绝对坐标。
        self.use_relative_waypoints = rospy.get_param('~use_relative_waypoints', True)
        self.s_route = rospy.get_param('~s_route', 'right')  # left/左 或 right/右，由裁判要求决定
        self.nav_timeout = float(rospy.get_param('~nav_timeout', 90.0))
        self.mission_timeout = float(rospy.get_param('~mission_timeout', 300.0))
        self.detect_timeout = float(rospy.get_param('~detect_timeout', 8.0))
        self.detect_retry = int(rospy.get_param('~detect_retry', 2))
        self.detect_settle_time = float(rospy.get_param('~detect_settle_time', 0.8))
        self.accept_commands = rospy.get_param('~accept_commands', ['开始执行任务', '开始安全任务'])

        # 麦轮运动控制节点
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        # 蜂鸣器
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        # 舵机控制，保留原车初始化姿态；如果不用机械臂，可 rosparam 设置 ~init_arm false
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        self._init_arm_if_needed()

        # 导航点发布
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_callback)

        # 订阅位姿，优先用 amcl_pose；没有 amcl 时用 odom 兜底。
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 服务：保留 test；新增绕障、识别、左右 S 路线切换测试。
        rospy.Service('~test', Trigger, self.test_callback)
        rospy.Service('~avoid', Trigger, self.start_avoid_callback)
        rospy.Service('~recognize', Trigger, self.start_recognize_callback)
        rospy.Service('~route_left', Trigger, self.set_route_left_callback)
        rospy.Service('~route_right', Trigger, self.set_route_right_callback)

        rospy.set_param('~status', 'start')

        # 等待语音识别节点启动；比赛时仍使用原来的“口令唤醒”。
        self._wait_voice_node()
        self.vc_sub = rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)

        rospy.loginfo('唤醒口令: 小迈小迈')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒')
        rospy.loginfo('控制指令: 开始执行任务 / 开始安全任务')

        # 等待导航 costmap，避免任务一开始就发点失败。
        try:
            rospy.wait_for_message(self.costmap, OccupancyGrid, timeout=30.0)
        except rospy.ROSException:
            rospy.logwarn('30秒内未收到 costmap: %s，仍继续启动节点；请确认 move_base 已启动。', self.costmap)

        self.play(rospy.get_param('~ready_voice', 'running'))
        signal.signal(signal.SIGINT, self.shutdown)
        self.run()

    # -------------------- 初始化和基础工具 --------------------

    def _init_arm_if_needed(self):
        if not rospy.get_param('~init_arm', True):
            return
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish'):
                    break
            except Exception:
                pass
            if (rospy.Time.now() - start).to_sec() > float(rospy.get_param('~arm_init_timeout', 10.0)):
                rospy.logwarn('舵机节点未完全就绪，跳过机械臂初始化；如需等待更久，调大 ~arm_init_timeout。')
                return
            rospy.sleep(0.1)

        # 原车初始姿态
        bus_servo_control.set_servos(
            self.joints_pub,
            2,
            ((1, 500), (2, 900), (3, 15), (4, 150), (5, 500), (10, 200))
        )
        rospy.sleep(2)

    def _wait_voice_node(self):
        if not rospy.get_param('~wait_voice_init', True):
            return
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/voice_control/init_finish'):
                    return
            except Exception:
                pass
            if (rospy.Time.now() - start).to_sec() > float(rospy.get_param('~voice_init_timeout', 30.0)):
                rospy.logwarn('30秒内未检测到 /voice_control/init_finish，继续启动；语音节点未启动时只能用服务 ~test。')
                return
            rospy.sleep(0.1)

    def play(self, name):
        try:
            voice_play.play(name, language=self.language)
        except Exception as e:
            rospy.logwarn('语音文件/播放失败: %s, %s', name, e)

    def shutdown(self, signum, frame):
        self.running = False
        self.mecanum_pub.publish(Twist())
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def call_trigger(self, service_name, timeout=1.5):
        """安全调用 Trigger 服务；服务不存在时不让主程序崩溃。"""
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            return rospy.ServiceProxy(service_name, Trigger)()
        except Exception as e:
            rospy.logwarn('服务调用跳过: %s, %s', service_name, e)
            return None

    # -------------------- ROS 回调 --------------------

    def test_callback(self, msg):
        self.words = '开始执行任务'
        return TriggerResponse(success=True)

    def start_avoid_callback(self, msg):
        self.capture_start_pose()
        ok = self.avoidance_task()
        return TriggerResponse(success=ok)

    def start_recognize_callback(self, msg):
        if self.start_pose is None:
            self.capture_start_pose()
        ok = self.recognition_task()
        return TriggerResponse(success=ok)

    def set_route_left_callback(self, msg):
        self.s_route = 'right'
        rospy.set_param('~s_route', 'right')
        rospy.loginfo('S 型绕障路线已设置为：right / 右侧开始')
        return TriggerResponse(success=True, message='route right')

    def set_route_right_callback(self, msg):
        self.s_route = 'right'
        rospy.set_param('~s_route', 'right')
        rospy.loginfo('S 型绕障路线已设置为：right / 右侧开始')
        return TriggerResponse(success=True, message='route right')

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            self.words = self.words.replace(' ', '')
        print('words:', self.words)

        if self.words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            buzzer_msg = BuzzerState()
            buzzer_msg.freq = 1900
            buzzer_msg.on_time = 0.05
            buzzer_msg.off_time = 0.01
            buzzer_msg.repeat = 1
            self.buzzer_pub.publish(buzzer_msg)

    def move_callback(self, msg):
        try:
            self.move_base_status = msg.status.status
            rospy.loginfo('move_base result status: %s', self.move_base_status)
        except Exception:
            self.move_base_status = 1

    def amcl_pose_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = (pose.position.x, pose.position.y, qua2yaw(pose.orientation))

    def odom_callback(self, msg):
        # 如果还没有 amcl_pose，就用 odom 兜底。
        if self.current_pose is None:
            pose = msg.pose.pose
            self.current_pose = (pose.position.x, pose.position.y, qua2yaw(pose.orientation))

    # -------------------- 导航相关 --------------------

    def capture_start_pose(self):
        """记录任务开始时位姿。相对坐标模式下，所有比赛尺寸都基于这个位姿换算。"""
        start = rospy.Time.now()
        while not rospy.is_shutdown() and self.current_pose is None:
            if (rospy.Time.now() - start).to_sec() > 5.0:
                break
            rospy.sleep(0.05)

        if self.current_pose is None:
            rospy.logwarn('没有收到 /amcl_pose 或 /odom，使用地图原点作为临时起点。')
            self.start_pose = (0.0, 0.0, 0.0)
        else:
            self.start_pose = self.current_pose
        rospy.loginfo('任务起点位姿: x=%.3f, y=%.3f, yaw=%.1fdeg',

                      self.start_pose[0], self.start_pose[1], math.degrees(self.start_pose[2]))
    def relative_to_map(self, forward, left, yaw_deg):
        """将“前进/左移/相对朝向”换算为 map 坐标。"""
        if self.start_pose is None:
            self.capture_start_pose()
        sx, sy, syaw = self.start_pose
        x = sx + forward * math.cos(syaw) - left * math.sin(syaw)
        y = sy + forward * math.sin(syaw) + left * math.cos(syaw)
        # 修改：目标朝向使用相对朝向（基于地图的绝对角度），不叠加初始朝向
        # 这样第一个点到第二个点时的方向为小车摆放的初始方向（yaw=0表示正对前方）
        yaw = norm_angle(math.radians(yaw_deg))
        return x, y, yaw

    def nav_position(self, x, y, w_deg):
        """兼容原代码：发布 map 绝对坐标，w_deg 为角度。"""
        self.nav_position_yaw(x, y, math.radians(w_deg))

    def nav_position_yaw(self, x, y, yaw_rad):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = rpy2qua(0.0, 0.0, yaw_rad)
        self.move_base_status = 1
        self.goal_pub.publish(pose)

    def go_to_relative(self, name, forward, left, yaw_deg=0.0, timeout=None):
        x, y, yaw = self.relative_to_map(float(forward), float(left), float(yaw_deg))
        rospy.loginfo('导航到 %-16s rel=(%.2f, %.2f, %.1fdeg) map=(%.2f, %.2f, %.1fdeg)',
                      name, forward, left, yaw_deg, x, y, math.degrees(yaw))
        self.nav_position_yaw(x, y, yaw)
        rospy.sleep(0.5)
        return self.wait_nav_status(timeout if timeout is not None else self.nav_timeout)

    def go_to_absolute(self, name, x, y, yaw_deg=0.0, timeout=None):
        rospy.loginfo('导航到 %-16s abs=(%.2f, %.2f, %.1fdeg)', name, x, y, yaw_deg)
        self.nav_position(float(x), float(y), float(yaw_deg))
        rospy.sleep(0.5)
        return self.wait_nav_status(timeout if timeout is not None else self.nav_timeout)

    def wait_nav_status(self, timeout=None):
        start = rospy.Time.now()
        failed_status = [4, 5, 8, 9]
        while not rospy.is_shutdown() and self.running:
            if self.move_base_status == 3:
                self.move_base_status = 1
                return True
            if self.move_base_status in failed_status:
                rospy.logwarn('导航失败，move_base status=%s', self.move_base_status)
                self.move_base_status = 1
                return False
            if timeout is not None and (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn('导航等待超时 %.1fs', timeout)
                return False
            rospy.sleep(0.2)
        return False

    def stop_chassis(self):
        self.mecanum_pub.publish(Twist())

    # -------------------- 比赛路径：S 型避障 --------------------

    def get_avoid_waypoints(self):
        """
        具身智能场地默认尺寸换算，单位 m。
        坐标含义：forward=从起点向前，left=机器人左侧。

        路径规划：初始向右 → 绕右边柱体外围半周 → 两柱体之间穿过 → 绕左边柱体外围半周 → 到达识别区
        柱体直径约0.2m，两柱体中心前后错开约0.4m，横向间距约0.8m
        """
        default_points = [
            {'name': 'first_passage',   'forward': 0.50, 'left': 0.00,  'yaw': 0},  # 进入通道
            {'name': 'obstacle_entry',  'forward': 1.40, 'left': 0.00,  'yaw': 0},  # 进入避障区
            {'name': 'right_approach',  'forward': 1.40, 'left': -0.77, 'yaw': -90},  # 向右靠近右边柱体
            {'name': 'right_around_1',  'forward': 1.90, 'left': -0.77, 'yaw': 90},  # 绕右边柱体外侧前半周
            {'name': 'right_around_2',  'forward': 1.90, 'left': 0.00, 'yaw': 90},  # 绕右边柱体外侧后半周
            {'name': 'between_pillars', 'forward': 1.30, 'left': 0.10, 'yaw': 90},  # 从两柱体之间穿过
            {'name': 'left_transition', 'forward': 1.30, 'left': 0.70, 'yaw': -55},  
            {'name': 'left_approach',   'forward': 1.90, 'left': 0.70, 'yaw': -90},  # 开始靠近左边柱体
            {'name': 'left_around_1',   'forward': 1.90, 'left': -0.07,  'yaw': -90},  # 绕左边柱体外侧前半周
            {'name': 'left_around_2',   'forward': 2.95, 'left': -0.07,  'yaw': 0},  # 绕左边柱体外侧后半周
        ]
        points = rospy.get_param('~avoid_waypoints', default_points)
        return self.normalize_waypoints(points, default_points)

    def avoidance_task(self):
        rospy.loginfo('开始 S 型避障任务，当前路线: %s', self.s_route)
        for p in self.get_avoid_waypoints():
            if not self._mission_time_ok():
                return False
            if self.use_relative_waypoints:
                ok = self.go_to_relative(p['name'], p['forward'], p['left'], p.get('yaw', 0.0))
            else:
                ok = self.go_to_absolute(p['name'], p['x'], p['y'], p.get('yaw', 0.0))
            if not ok:
                return False
        return True

    # -------------------- 比赛任务：识别 + 播报 --------------------

    def get_marker_slots(self):
        """
        识别区三个标志点，默认按规则图估算，单位 m。
        top/middle/bottom 是台子前方三个正方形标志点。
        如果你的建图坐标和默认值有偏差，优先在 launch/yaml 中改 ~marker_slots。
        """
        default_slots = [
            {'name': 'top_marker',    'forward': 3.50, 'left': 0.60,  'yaw': 0},  # 向右8厘米
            {'name': 'middle_marker', 'forward': 3.50, 'left': 0.00,  'yaw': 0},  # 向左10厘米
            {'name': 'bottom_marker', 'forward': 3.50, 'left': -0.45, 'yaw': 0},  # 向左15厘米
        ]
        slots = rospy.get_param('~marker_slots', default_slots)
        return self.normalize_waypoints(slots, default_slots)

    def normalize_waypoints(self, points, default_points):
        if not isinstance(points, list) or len(points) == 0:
            return default_points
        normalized = []
        for i, p in enumerate(points):
            if not isinstance(p, dict):
                continue
            item = {}
            item['name'] = str(p.get('name', 'point_%d' % i))
            item['yaw'] = float(p.get('yaw', p.get('w', 0.0)))
            if self.use_relative_waypoints:
                item['forward'] = float(p.get('forward', p.get('x', 0.0)))
                item['left'] = float(p.get('left', p.get('y', 0.0)))
            else:
                item['x'] = float(p.get('x', p.get('forward', 0.0)))
                item['y'] = float(p.get('y', p.get('left', 0.0)))
            normalized.append(item)
        return normalized if normalized else default_points

    def get_task_sequence(self):
        """
        识别顺序：
        - ~task_sequence 可设为 "sphere,cube,cylinder"、"球体,正方体,圆柱体" 或列表。
        - 默认 auto：先尝试在终点区识别裁判出示的汉字标签/形状标签；识别不到则按三类全识别。
        """
        param = rospy.get_param('~task_sequence', 'auto')
        all_shapes = ['sphere', 'cube', 'cylinder']

        if isinstance(param, list):
            seq = [self.normalize_shape(x) for x in param]
            seq = [x for x in seq if x in all_shapes]
            return seq if seq else all_shapes

        param_text = str(param).strip()
        if param_text.lower() != 'auto':
            raw_items = param_text.replace('，', ',').replace(';', ',').replace('；', ',').split(',')
            seq = [self.normalize_shape(x.strip()) for x in raw_items]
            seq = [x for x in seq if x in all_shapes]
            return seq if seq else all_shapes

        # auto：在终点区先试一次，适配已有 OCR/YOLO 把标签结果写入 /yolov5/shape 的情况。
        rospy.loginfo('尝试在终点区识别裁判标签；若识别不到，将自动识别三个标志点。')
        first = self.detect_shape(context='label')
        first = self.normalize_shape(first)
        if first in all_shapes:
            return [first] + [x for x in all_shapes if x != first]
        return all_shapes

    def recognition_task(self):
        rospy.loginfo('开始识别区任务')
        self.slot_shape_cache = {}
        used_slots = set()
        spoken_shapes = set()
        slots = self.get_marker_slots()
        targets = self.get_task_sequence()
        rospy.loginfo('目标识别顺序: %s', targets)

        # 按目标顺序寻找对应物体；物体摆放顺序随机时，会逐个标志点扫描。
        for target in targets:
            if not self._mission_time_ok():
                return False
            if target in spoken_shapes:
                continue
            ok = self.find_and_speak_target(target, slots, used_slots)
            if ok:
                spoken_shapes.add(target)

        # 保险：如果还有未播报的槽位，继续到剩余标志点识别并播报，保证三件物体都处理。
        for slot in slots:
            if not self._mission_time_ok():
                return False
            if slot['name'] in used_slots:
                continue
            shape = self.go_detect_slot(slot)
            if shape:
                self.speak_shape(shape)
                used_slots.add(slot['name'])
                spoken_shapes.add(shape)

        rospy.loginfo('识别区任务完成，已播报: %s', list(spoken_shapes))
        return True

    def find_and_speak_target(self, target, slots, used_slots):
        # 已经缓存过位置时，直接去对应标志点。
        for slot in slots:
            if slot['name'] in used_slots:
                continue
            if self.slot_shape_cache.get(slot['name']) == target:
                if self.go_to_slot(slot):
                    self.speak_shape(target)
                    used_slots.add(slot['name'])
                    return True

        # 未知位置时扫描剩余标志点。
        for slot in slots:
            if slot['name'] in used_slots:
                continue
            shape = self.go_detect_slot(slot)
            if shape:
                self.slot_shape_cache[slot['name']] = shape
                if shape == target:
                    self.speak_shape(target)
                    used_slots.add(slot['name'])
                    return True

        rospy.logwarn('没有找到目标物体: %s', target)
        return False

    def go_to_slot(self, slot):
        """
        导航到识别区标志点。采用横向平移模式：保持forward坐标不变，仅改变left坐标。
        三个识别点在同一纵向位置，横向间距约0.65m。
        """
        # 获取目标点坐标
        target_forward = slot['forward'] if self.use_relative_waypoints else slot['x']
        target_left = slot['left'] if self.use_relative_waypoints else slot['y']
        
        # 如果已经在识别区（有上次位置记录），采用横向平移
        if hasattr(self, '_last_slot_forward') and hasattr(self, '_last_slot_left'):
            # 保持forward不变，只改变left（横向平移）
            if self.use_relative_waypoints:
                result = self.go_to_relative(slot['name'], self._last_slot_forward, target_left, slot.get('yaw', 0.0))
            else:
                result = self.go_to_absolute(slot['name'], self._last_slot_forward, target_left, slot.get('yaw', 0.0))
        else:
            # 第一次进入识别区，正常导航到目标点
            if self.use_relative_waypoints:
                result = self.go_to_relative(slot['name'], target_forward, target_left, slot.get('yaw', 0.0))
            else:
                result = self.go_to_absolute(slot['name'], target_forward, target_left, slot.get('yaw', 0.0))
        
        # 记录当前位置
        if self.use_relative_waypoints:
            self._last_slot_forward = target_forward
            self._last_slot_left = target_left
        else:
            self._last_slot_forward = slot['x']
        
        return result

    def go_detect_slot(self, slot):
        if not self.go_to_slot(slot):
            return None
        rospy.sleep(self.detect_settle_time)
        shape = self.detect_shape(context=slot['name'])
        shape = self.normalize_shape(shape)
        if shape:
            rospy.loginfo('%s 识别结果: %s', slot['name'], shape)
        else:
            rospy.logwarn('%s 未识别到有效形状', slot['name'])
        return shape

    def detect_shape(self, context='object'):
        """
        使用已有 yolov5 节点识别。要求识别节点把结果写入 /yolov5/shape。
        支持返回：sphere/ball/球体、cube/box/正方体、cylinder/圆柱体 等。
        """
        for attempt in range(max(1, self.detect_retry)):
            rospy.set_param('/yolov5/shape', 'None')
            self.call_trigger('/yolov5/start', timeout=1.0)
            start = rospy.Time.now()
            while not rospy.is_shutdown() and self.running:
                raw_shape = rospy.get_param('/yolov5/shape', 'None')
                shape = self.normalize_shape(raw_shape)
                if shape:
                    self.call_trigger('/yolov5/stop', timeout=0.5)
                    return shape
                if (rospy.Time.now() - start).to_sec() > self.detect_timeout:
                    break
                rospy.sleep(0.2)
            self.call_trigger('/yolov5/stop', timeout=0.5)
            rospy.logwarn('%s 第 %d 次识别超时', context, attempt + 1)
        return None

    def normalize_shape(self, raw_shape):
        if raw_shape is None:
            return None
        s = str(raw_shape).strip().lower()
        if s in ['', 'none', 'null', 'unknown', 'no', '未识别']:
            return None

        aliases = {
            'sphere': ['sphere', 'ball', 'round', 'orb', 'qiu', '球体', '球', '圆球'],
            'cube': ['cube', 'box', 'square', 'fang', 'zhengfangti', '正方体', '方体', '方块', '立方体'],
            'cylinder': ['cylinder', 'cylindrical', 'yuanzhuti', '圆柱体', '圆柱'],
        }
        for key, values in aliases.items():
            for value in values:
                if value in s:
                    return key
        return None

    def speak_shape(self, shape):
        """按规则连续播报 3 次：这是……。"""
        voice_map = {
            'sphere': rospy.get_param('~voice_sphere', 'this_is_sphere'),
            'cube': rospy.get_param('~voice_cube', 'this_is_cube'),
            'cylinder': rospy.get_param('~voice_cylinder', 'this_is_cylinder'),
        }
        name = voice_map.get(shape)
        if name is None:
            rospy.logwarn('未知形状，无法播报: %s', shape)
            return
        for _ in range(3):
            self.play(name)
            rospy.sleep(0.25)

    # -------------------- 总任务流程 --------------------

    def embodied_ai_mission(self):
        self.mission_start_time = rospy.Time.now()
        rospy.set_param('~status', 'running')
        self.capture_start_pose()
        self.play(rospy.get_param('~start_voice', '1'))

        # 1. 从起点出发，通过第一个通道，按 S 形绕开两个圆柱障碍，到达终点区。
        if not self.avoidance_task():
            rospy.set_param('~status', 'failed')
            self.stop_chassis()
            return False

        # 2. 终点/识别区任务：识别球体、正方体、圆柱体，并在标志点播报三次。
        if not self.recognition_task():
            rospy.set_param('~status', 'failed')
            self.stop_chassis()
            return False

        self.stop_chassis()
        rospy.set_param('~status', 'stop')
        self.play(rospy.get_param('~finish_voice', 'mission_accomplished'))
        return True

    def _mission_time_ok(self):
        if not hasattr(self, 'mission_start_time'):
            return True
        used = (rospy.Time.now() - self.mission_start_time).to_sec()
        if used > self.mission_timeout:
            rospy.logwarn('任务超时: %.1fs > %.1fs', used, self.mission_timeout)
            return False
        return True

    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.words is not None:
                if self.words in self.accept_commands:
                    print('>>>>>>>>>>>>>>>>>>>> 开始具身智能任务 <<<<<<<<<<<<<<<<<')
                    try:
                        self.vc_sub.unregister()
                    except Exception:
                        pass
                    ok = self.embodied_ai_mission()
                    print('>>>>>>>>>>>>>>>>>>>> 具身智能任务结束: %s <<<<<<<<<<<<<<<<<' % ('成功' if ok else '失败'))
                elif self.words == '休眠(Sleep)':
                    rospy.sleep(0.01)
                self.words = None
            else:
                rospy.sleep(0.01)

        self.stop_chassis()


if __name__ == '__main__':
    VoiceControlNavNode('voice_control_nav')
