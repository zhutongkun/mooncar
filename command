# data:2023/09/19 by aiden
# 本文档只包含启动需要的指令，部分指令还需要配合其他设置才能生效
# 请结合教程文档使用. 特别说明: 每行指令需要单独开一个终端运行，
# 且有先后之分

# hiwonder_bringup hiwonder_app 
#关闭app自启功能 
sudo systemctl disable start_app_node.service 

#停止app功能 
sudo systemctl stop start_app_node.service

#开启app自启功能 
sudo systemctl enable start_app_node.service

#开启app功能 
sudo systemctl start start_app_node.service

#重启app功能
sudo systemctl restart start_app_node.service

#查看app后台自启状态 
sudo systemctl status start_app_node.service

#hiwonder_calibration
#深度摄像头红外标定 
roslaunch hiwonder_calibration depth_cam_ir_calibration.launch

#深度摄像头RGB标定 
roslaunch hiwonder_calibration depth_cam_rgb_calibration.launch

#角速度校准 
roslaunch hiwonder_calibration calibrate_angular.launch angular:=true

#线速度校准 
roslaunch hiwonder_calibration calibrate_linear.launch linear:=true

#imu校准 
roslaunch hiwonder_calibration calibrate_imu.launch

#hiwonder_example
#深度摄像头红外可视化 
roslaunch hiwonder_example depth_cam_ir_view.launch

#深度摄像头点云可视化 
roslaunch hiwonder_example depth_cam_point_cloud_view.launch

#深度摄像头RGB图像可视化 
roslaunch hiwonder_example depth_cam_rgb_view.launch

#肢体姿态融合RGB控制 
roslaunch hiwonder_example body_and_rgb_control.launch

#肢体姿态控制 
roslaunch hiwonder_example body_control.launch

#人体跟踪 
roslaunch hiwonder_example body_track.launch

#跌倒检测 
roslaunch hiwonder_example fall_down_detect.launch

#颜色识别 
roscd hiwonder_example/scripts/color_detect && python3 color_detect_demo.py

#颜色追踪 
roslaunch hiwonder_example color_track_node.launch

#颜色分拣
roslaunch hiwonder_example color_sorting_node.launch
#roslaunch hiwonder_example color_sorting_node.launch debug:=true

#循线清障
roslaunch hiwonder_example line_follow_clean_node.launch
#roslaunch hiwonder_example line_follow_clean_node.launch debug:=true

#颜色夹取 
roslaunch hiwonder_example automatic_pick.launch
#roslaunch hiwonder_example automatic_pick.launch debug:=true
# 开启夹取
rosservice call /automatic_pick/pick "{}"
#开启放置
rosservice call /automatic_pick/place "{}"

#导航搬运 
roslaunch hiwonder_example navigation_transport.launch map:=xxx

#无人驾驶
roslaunch hiwonder_example self_driving.launch

#人脸检测 
roscd hiwonder_example/scripts/mediapipe_example && python3 face_detect.py

#人脸网格 
roscd hiwonder_example/scripts/mediapipe_example && python3 face_mesh.py

#手关键点检测 
roscd hiwonder_example/scripts/mediapipe_example && python3 hand.py

#肢体关键点检测 
roscd hiwonder_example/scripts/mediapipe_example && python3 pose.py

#背景分割 
roscd hiwonder_example/scripts/mediapipe_example && python3 self_segmentation.py

#整体检测 
roscd hiwonder_example/scripts/mediapipe_example && python3 holistic.py

#3D物体检测 
roscd hiwonder_example/scripts/mediapipe_example && python3 objectron.py

#hiwonder_slam
#建图
roslaunch hiwonder_slam slam.launch

#rviz查看建图效果
roslaunch hiwonder_slam rviz_slam.launch

#键盘控制(可选)
roslaunch hiwonder_peripherals teleop_key_control.launch

#保存地图 
roscd hiwonder_slam/maps && rosrun map_server map_saver map:=/robot_1/map -f 保存名称

#app建图
roslaunch hiwonder_slam slam.launch app:=true
roscd hiwonder_slam/maps && rosrun map_server map_saver map:=/map -f 保存名称

# hiwonder navigation
#导航
roslaunch hiwonder_navigation navigation.launch map:=地图名称

#rviz发布导航目标
roslaunch hiwonder_navigation rviz_navigation.launch

#多点导航
roslaunch hiwonder_navigation publish_point.launch

#app导航
roslaunch hiwonder_navigation navigation.launch map:=地图名称 app:=true

#3D建图
roslaunch hiwonder_slam slam.launch slam_methods:=rtabmap
#rviz查看建图效果
roslaunch hiwonder_slam rviz_slam.launch slam_methods:=rtabmap

#3D导航 
roslaunch hiwonder_navigation rtabmap_navigation.launch
#rviz发布导航目标
roslaunch hiwonder_navigation rviz_rtabmap_navigation.launch

#hiwonder_simulations
#urdf可视化 
roslaunch hiwonder_description display.launch

#gazebo可视化 
roslaunch hiwonder_gazebo worlds.launch

#moveit
#仅仿真 
roslaunch hiwonder_moveit_config demo.launch

#和真实联动
roslaunch hiwonder_moveit_config demo.launch fake_execution:=false

#和gazebo联动
roslaunch hiwonder_moveit_config demo_gazebo.launch

#仿真建图 
#gazebo仿真
roslaunch hiwonder_gazebo room_worlds.launch
roslaunch hiwonder_slam slam.launch sim:=true
roslaunch hiwonder_slam rviz_slam.launch sim:=true

#仿真导航 
roslaunch hiwonder_gazebo room_worlds.launch
roslaunch hiwonder_navigation navigation.launch sim:=true map:=地图名称
roslaunch hiwonder_navigation rviz_navigation.launch sim:=true

#xf_mic_asr_offline
#语音控制移动 
roslaunch xf_mic_asr_offline voice_control_move.launch

#语音控制导航 
roslaunch xf_mic_asr_offline voice_control_navigation.launch map:=地图名称

#图像采集软件
roslaunch hiwonder_peripherals depth_cam.launch
python3 ~/software/collect_picture/main.py

#lab软件
roslaunch hiwonder_peripherals depth_cam.launch
python3 ~/software/lab_tool/main.py

#舵机调试软件
python3 ~/software/servo_tool/main.py

#图像标注软件
python3 ~/software/labelImg/labelImg.py
