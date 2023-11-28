# data:2023/09/19 by aiden
# 本文档只包含启动需要的指令，部分指令还需要配合其他设置才能生效
# 请结合教程文档使用. 特别说明: 每行指令需要单独开一个终端运行，
# 且有先后之分

# hiwonder_bringup hiwonder_app 
#1 关闭app自启功能 
sudo systemctl disable start_app_node.service 

#2 停止app功能 
sudo systemctl stop start_app_node.service

#3 开启app自启功能 
sudo systemctl enable start_app_node.service

#4 开启app功能 
sudo systemctl start start_app_node.service

#5 重启app功能
sudo systemctl restart start_app_node.service

#6 查看app后台自启状态 
sudo systemctl status start_app_node.service

# hiwonder_calibration
#7 深度摄像头红外标定 
roslaunch hiwonder_calibration depth_cam_ir_calibration.launch

#8 深度摄像头RGB标定 
roslaunch hiwonder_calibration depth_cam_rgb_calibration.launch

#9 角速度校准 
roslaunch hiwonder_calibration calibrate_angular.launch angular:=true

#10 线速度校准 
roslaunch hiwonder_calibration calibrate_linear.launch linear:=true

#11 imu校准 
roslaunch hiwonder_calibration calibrate_imu.launch

# hiwonder_example
#13 apriltag检测 
roslaunch hiwonder_example apriltag_detect_demo.launch
 
#14 ar检测 
roslaunch hiwonder_example ar_detect_demo.launch

#15 深度摄像头红外可视化 
roslaunch hiwonder_example depth_cam_ir_view.launch

#16 深度摄像头点云可视化 
roslaunch hiwonder_example depth_cam_point_cloud_view.launch

#17 深度摄像头RGB图像可视化 
roslaunch hiwonder_example depth_cam_rgb_view.launch

#18 单目orb_slam2 
roslaunch hiwonder_example orb_slam2_mono.launch

#19 深度orb_slam2 
roslaunch hiwonder_example orb_slam2_rgbd.launch

#20 单目orb_slam3 
roslaunch hiwonder_example orb_slam3_mono.launch

#21 深度orb_slam3 
roslaunch hiwonder_example orb_slam3_rgbd.launch

#22 肢体姿态融合RGB控制 
roslaunch hiwonder_example body_and_rgb_control.launch

#23 肢体姿态控制 
roslaunch hiwonder_example body_control.launch

#24 人体跟踪 
roslaunch hiwonder_example body_track.launch

#25 跌倒检测 
roslaunch hiwonder_example fall_down_detect.launch

#26 颜色识别 
roscd hiwonder_example/scripts/color_detect && python3 color_detect_demo.py

#27 颜色追踪 
roslaunch hiwonder_example color_track_node.launch

#28 指尖轨迹 
roslaunch hiwonder_example hand_trajectory_node.launch

#29 手部跟随 
roslaunch hiwonder_example hand_track_node.launch

#30 二维码生成 
roscd hiwonder_example/scripts/qrcode && python3 qrcode_creater.py

#31 二维码检测 
roscd hiwonder_example/scripts/qrcode && python3 qrcode_detecter.py

#33 物体跟踪 
roslaunch hiwonder_example object_tracking.launch

#34 颜色分拣
roslaunch hiwonder_example color_sorting_node.launch
#roslaunch hiwonder_example color_sorting_node.launch debug:=true

#35 垃圾分类
roslaunch hiwonder_example garbage_classification.launch
#roslaunch hiwonder_example garbage_classification.launch debug:=true

#36 循线清障
roslaunch hiwonder_example line_follow_clean_node.launch
#roslaunch hiwonder_example line_follow_clean_node.launch debug:=true

#37 颜色夹取 
roslaunch hiwonder_example automatic_pick.launch
#roslaunch hiwonder_example automatic_pick.launch debug:=true
# 开启夹取
rosservice call /automatic_pick/pick "{}"
# 开启放置
rosservice call /automatic_pick/place "{}"

#38 导航搬运 
roslaunch hiwonder_example navigation_transport.launch map:=xxx

#39 人脸检测 
roscd hiwonder_example/scripts/mediapipe_example && python3 face_detect.py

#40 人脸网格 
roscd hiwonder_example/scripts/mediapipe_example && python3 face_mesh.py

#41 手关键点检测 
roscd hiwonder_example/scripts/mediapipe_example && python3 hand.py

#42 肢体关键点检测 
roscd hiwonder_example/scripts/mediapipe_example && python3 pose.py

#43 背景分割 
roscd hiwonder_example/scripts/mediapipe_example && python3 self_segmentation.py

#39 整体检测 
roscd hiwonder_example/scripts/mediapipe_example && workon mediapipe && python3 holistic.py
#使用完毕后需要退出虚拟环境，输入指令deactivate

#40 3D物体检测 
roscd hiwonder_example/scripts/mediapipe_example && workon mediapipe && python3 objectron.py
#使用完毕后需要退出虚拟环境，输入指令deactivate

# hiwonder_slam
#41 建图 
# 建图
roslaunch hiwonder_slam slam.launch

# rviz查看建图效果
roslaunch hiwonder_slam rviz_slam.launch

# 键盘控制(可选)
roslaunch hiwonder_peripherals teleop_key_control.launch

#42 保存地图 
roscd hiwonder_slam/maps && rosrun map_server map_saver map:=/robot_1/map -f 保存名称

# app建图
roslaunch hiwonder_slam slam.launch app:=true
roscd hiwonder_slam/maps && rosrun map_server map_saver map:=/map -f 保存名称

# hiwonder navigation
#43 导航 
# 导航
roslaunch hiwonder_navigation navigation.launch map:=地图名称

# rviz发布导航目标
roslaunch hiwonder_navigation rviz_navigation.launch

# 多点导航
roslaunch hiwonder_navigation publish_point.launch

# app导航
roslaunch hiwonder_navigation navigation.launch map:=地图名称 app:=true

#44 3D建图
# 3D建图
roslaunch hiwonder_slam slam.launch slam_methods:=rtabmap
# rviz查看建图效果
roslaunch hiwonder_slam rviz_slam.launch slam_methods:=rtabmap

#45 3D导航 
# 3D导航
roslaunch hiwonder_navigation rtabmap_navigation.launch
# rviz发布导航目标
roslaunch hiwonder_navigation rviz_rtabmap_navigation.launch

# hiwonder_multi
#46 多机群控 
roslaunch hiwonder_multi multi_control.launch

#47 多机建图 
#master 
roscd hiwonder_multi/launch/multi_slam && roslaunch master_node.launch
roslaunch hiwonder_multi multi_slam_rviz.launch

#host   
roscd hiwonder_multi/launch/multi_slam && roslaunch slave_node.launch

#48 多机导航 
#master 
roscd hiwonder_multi/launch/multi_navigation && roslaunch master_node.launch map:=地图名称
roslaunch hiwonder_multi multi_navigation_rviz.launch

#host   
roscd hiwonder_multi/launch/multi_navigation && roslaunch slave_node.launch

#49 多机编队 
#master 
roscd hiwonder_multi/launch/multi_formation && roslaunch master_node.launch map:=地图名称
roslaunch hiwonder_multi multi_formation_rviz.launch

#host   
roscd hiwonder_multi/launch/multi_formation && roslaunch slave_node.launch

#50 多机环绕 
#master 
roscd hiwonder_multi/launch/multi_surround && roslaunch master_node.launch map:=地图名称
roslaunch hiwonder_multi multi_surround_rviz.launch

#host   
roscd hiwonder_multi/launch/multi_surround && roslaunch slave_node.launch

# hiwonder_simulations
#51 urdf可视化 
roslaunch hiwonder_description display.launch

#52 gazebo可视化 
roslaunch hiwonder_gazebo worlds.launch

#53 moveit
# 仅仿真 
roslaunch hiwonder_moveit_config demo.launch

# 和真实联动
roslaunch hiwonder_moveit_config demo.launch fake_execution:=false

# 和gazebo联动
roslaunch hiwonder_moveit_config demo_gazebo.launch

#54 仿真建图 
# gazebo仿真
roslaunch hiwonder_gazebo room_worlds.launch
roslaunch hiwonder_slam slam.launch sim:=true
roslaunch hiwonder_slam rviz_slam.launch sim:=true

#55 仿真导航 
roslaunch hiwonder_gazebo room_worlds.launch
roslaunch hiwonder_navigation navigation.launch sim:=true map:=地图名称
roslaunch hiwonder_navigation rviz_navigation.launch sim:=true

# xf_mic_asr_offline
#56 语音控制移动 
roslaunch xf_mic_asr_offline voice_control_move.launch

#57 语音控制导航 
roslaunch xf_mic_asr_offline voice_control_navigation.launch map:=地图名称

#58 语音控制颜色检测 
roslaunch xf_mic_asr_offline voice_control_color_detect.launch

#59 语音控制颜色跟踪 
roslaunch xf_mic_asr_offline voice_control_color_track.launch

#60 语音控制颜色分拣
roslaunch xf_mic_asr_offline voice_control_color_sorting.launch

#61 语音控制垃圾分类
roslaunch xf_mic_asr_offline voice_control_garbage_classification.launch

#62 图像采集软件
roslaunch hiwonder_peripherals depth_cam.launch
python3 ~/software/collect_picture/main.py

#63 lab软件
roslaunch hiwonder_peripherals depth_cam.launch
python3 ~/software/lab_tool/main.py

#64 舵机调试软件
python3 ~/software/servo_tool/main.py

#65 图像标注软件
python3 ~/software/labelImg/labelImg.py

#66 手势控制
roslaunch hiwonder_example hand_gesture_control_node.launch
