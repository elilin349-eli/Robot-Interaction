import lerobot
import lerobot.find_port
import json
import os

import rclpy
import rclpy.duration
from rclpy.node import Node
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
import serial
import serial.tools.list_ports
from pathlib import Path
from control_pkg.robot_system.utils.path_manager import PathManager
from geometry_msgs.msg import TransformStamped,PoseStamped
from visualization_msgs.msg import MarkerArray,Marker
from lerobot.model.kinematics import RobotKinematics
from tf2_ros import TransformBroadcaster,Buffer,TransformListener
from scipy.spatial.transform import Rotation as R
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf_transformations as tf_trans
import scservo_sdk as scs
import numpy as np
import threading
import time

from control_pkg.gui import GUI
from control_pkg.robot_system.control.motion_planner import clamp_step, marker_is_fresh

class TFNode(Node):
    def __init__(self):
        super().__init__("tf_node")
        self.declare_parameter("root_path", "")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("calibration_dir", "")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("enable_gui", False)
        self.declare_parameter("marker_cache_ttl", 2.0)
        self.declare_parameter("max_step", 15.0)
        self.dt = float(self.get_parameter("dt").value or 0.1)
        self.speed = 0.05
        self.running = True
        self.is_connected = False
        self.pid = 0  # 协议版本，SO-100 用 0
        self.ports = lerobot.find_port.find_available_ports()
        self.get_logger().info(f"可用端口:{self.ports}")

        self.port = self.get_parameter("port").value or "/dev/ttyACM0"
        baudrate = int(self.get_parameter("baudrate").value or 1000000)

        self.motor_ids = [1, 2, 3, 4, 5, 6]
        root_path = self.get_parameter("root_path").value
        root_obj = Path(root_path).expanduser() if root_path else None
        try:
            if root_obj:
                default_calibration_dir = root_obj / "calibration"
                default_urdf_path = root_obj / "urdf" / "so101.urdf"
            else:
                default_calibration_dir = PathManager.calibration_dir()
                default_urdf_path = PathManager.urdf_file()
        except Exception:
            default_calibration_dir = self._default_share_path("calibration")
            default_urdf_path = self._default_share_path("Simulation", "SO101", "so101_new_calib.urdf")
        calibration_dir = self.get_parameter("calibration_dir").value
        if calibration_dir:
            self.calibration_dir_path = Path(calibration_dir)
        else:
            self.calibration_dir_path = default_calibration_dir

        urdf_path = self.get_parameter("urdf_path").value
        if urdf_path:
            self.urdf_path = Path(urdf_path)
        else:
            self.urdf_path = default_urdf_path
        self.config = SO101FollowerConfig(
            port=self.port,
            id="so101",
            use_degrees=True,
            calibration_dir=self.calibration_dir_path)
        self.robot = SO101Follower(self.config)
        self.get_logger().info(f"舵机id:{self.robot.bus.ids}")
        self.get_logger().info("start connect")

        try:
            self.robot.connect(calibrate=False)
        except Exception as e:
            self.get_logger().error(f"Robot connection failed: {e}")
            self.is_connected = False
            self.running = False
            return
        self.is_connected = True
        self.get_logger().info("已连接")
        self.obs_keys = list(self.robot.get_observation().keys())
        self.get_logger().info(f"观测结构:{self.obs_keys}")
        self.motors = list(self.robot.bus.motors.keys())
        self.get_logger().info(f"舵机名字:{self.motors}")
        self.action_features = self.robot.action_features
        self.get_logger().info(f"动作特征:{list(self.action_features)}")
        self.kinematics = RobotKinematics(
            urdf_path=str(self.urdf_path), 
            joint_names=self.motors
        )
        self.robot.bus.disable_torque(self.motors)
        self.is_torque_on = False

        self.obs = self.robot.get_observation()
        self.current_joints = [self.obs.get(obs_key) for obs_key in self.obs_keys]

        self.ee_pos = None
        self.ee_quat = None
        self.target_xyz:np.ndarray = np.array([])   # 目标位置 (米)
        self.target_quat:np.ndarray = np.array([])   # 目标姿态 (横滚, 俯仰, 偏航 rad)
        self.target_pose = None
        self.init_joints = self.current_joints
        self.gripper_degree = None
        self.marker_pos_camera = None
        self.marker_pos = None
        self.marker_cache = None
        self.marker_cache_time = 0.0
        self.marker_cache_ttl = float(self.get_parameter("marker_cache_ttl").value or 2.0)
        self.init_pos = None
        self.init_quat = None
        self._voice_grasp_lock = threading.Lock()
        self._voice_grasp_active = False
        
        
        # IK 控制参数
        self.max_step = float(self.get_parameter("max_step").value or 15.0)
        self.pos_weight = 1.0
        self.ori_weight = 0.01

        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer,node=self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_publisher = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )

        self.markerarray_subscriber = self.create_subscription(
            msg_type=MarkerArray,
            topic="/gpd/grasp_markers",
            callback=self.marker_callback,
            qos_profile=10
        )

        self.voice_grasp_sub = self.create_subscription(
            msg_type=String,
            topic="/voice_grasp_cmd",
            callback=self.voice_grasp_callback,
            qos_profile=10
        )

        self.enable_torque_srv = self.create_service(Trigger, "/enable_torque", self._srv_enable_torque)
        self.disable_torque_srv = self.create_service(Trigger, "/disable_torque", self._srv_disable_torque)
        self.emergency_stop_srv = self.create_service(Trigger, "/emergency_stop", self._srv_emergency_stop)

        self.init()
        
        #self.robot.bus.sync_write("Torque_Enable",1)
        self.thread = threading.Thread(
            target=self.read_loop,
            daemon=True,
        )
        self.thread.start()
        
        #self.timer = self.create_timer(0.03, self.broadcast_timer) # 10Hz 发布
        #port_name = self.find_arm_port()
        #self.ser = serial.Serial(port=port_name,baudrate=115200,timeout=0.1)

    def disable_torque(self):
        self.is_torque_on = False
        self.robot.bus.disable_torque()

    def enable_torque(self):
        self.is_torque_on = True
        self.robot.bus.enable_torque()

    def emergency_stop(self):
        self.running = False
        self._voice_grasp_active = False
        self.is_torque_on = False
        self.robot.bus.disable_torque(self.motors)
        self.get_logger().warn("紧急停止已触发")

    def _srv_enable_torque(self, request, response):
        self.enable_torque()
        response.success = True
        response.message = "torque enabled"
        return response

    def _srv_disable_torque(self, request, response):
        self.disable_torque()
        response.success = True
        response.message = "torque disabled"
        return response

    def _srv_emergency_stop(self, request, response):
        self.emergency_stop()
        response.success = True
        response.message = "emergency stop executed"
        return response

    def _default_share_path(self, *parts):
        try:
            from ament_index_python.packages import get_package_share_directory
            base = Path(get_package_share_directory("control_pkg"))
        except Exception:
            base = Path(__file__).resolve().parents[1]
        return base.joinpath(*parts)


    def transform_pose_to_target_frame(self,pose:PoseStamped,target_frame:str):
        can_tf = self.tf_buffer.can_transform(
            target_frame=target_frame,
            source_frame=pose.header.frame_id,
            time=self.get_clock().now(),
            timeout=rclpy.duration.Duration(seconds=1))
        if can_tf:
            pose_transformed = self.tf_buffer.transform(
                object_stamped=pose,
                target_frame=target_frame,
                timeout=rclpy.duration.Duration(seconds=1)
            )
            return pose_transformed
        else:
            self.get_logger().info("没有转换链")
            return None

    def marker_callback(self,msg:MarkerArray):
        if not msg.markers:
            return
        marker:Marker = msg.markers[0]
        self.marker_pos_camera = np.array([marker.pose.position.x,marker.pose.position.y,marker.pose.position.z])
        self.get_logger().info(f"相机坐标系下的抓取位姿:{np.round(self.marker_pos_camera,3)}")

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg() 
        pose.header.frame_id = "camera_link"
        pose.pose.position.x = self.marker_pos_camera[0]
        pose.pose.position.y = self.marker_pos_camera[1]
        pose.pose.position.z = self.marker_pos_camera[2]
        pose_base = self.transform_pose_to_target_frame(pose=pose,target_frame="base_link")
        if pose_base is None:
            self.marker_pos = None
        else:
            pos = np.array([pose_base.pose.position.x,pose_base.pose.position.y,pose_base.pose.position.z])# type:ignore
            self.marker_pos = pos
            self.marker_cache = pos.copy()
            self.marker_cache_time = time.time()
            self.get_logger().info(f"基座坐标系下的抓取位姿{np.round(self.marker_pos,3)}")
        '''
        pose_end = self.transform_pose_to_target_frame(pose=pose,target_frame="Link6")
        pose_end.header.stamp = self.get_clock().now().to_msg() 
        if pose_end is not None:
            pos = np.array([pose_end.pose.position.x,pose_end.pose.position.y,pose_end.pose.position.z]) # type:ignore
            self.get_logger().info(f"基座坐标系下的抓取位姿{np.round(pos,3)}")
        '''


    def init(self):
        self.ee_pos,self.ee_quat = self.get_ee_pos_and_quat()
        self.target_xyz = self.ee_pos
        self.target_quat = self.ee_quat
        self.init_pos = self.ee_pos
        self.init_quat = self.ee_quat
        self.gripper_degree = self.obs.get('wrist_roll.pos')


        '''
        # 2. 发布 "修正变换" (关键步骤)
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = 'world'          # 新的根坐标系：标准世界坐标系
        t_base.child_frame_id = 'base_link'       # 子坐标系：你的机械臂底座

        # 这里的 rpy 是为了抵消 URDF 里的旋转
        # URDF 里是 rpy="1.57 -1.67685e-15 1.57" (歪的)
        # 这里就填 rpy="-1.57 1.67685e-15 -1.57" (把它扭回来)
        correct_rpy = [-1.5708, 1.67685e-15, -1.5708] # 根据你的 URDF 调整
        correct_quat = tf_trans.quaternion_from_euler(correct_rpy[0], correct_rpy[1], correct_rpy[2])

        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.x = correct_quat[0]
        t_base.transform.rotation.y = correct_quat[1]
        t_base.transform.rotation.z = correct_quat[2]
        t_base.transform.rotation.w = correct_quat[3]

        self.tf_broadcaster.sendTransform(t_base) # 先发修正
        '''


    def reset(self):
        if self.init_pos is not None:
            self.target_xyz = self.init_pos
        if self.init_quat is not None:
            self.target_quat = self.init_quat


    def read_loop(self):
        while self.running:
            if not self.robot.is_connected:
                time.sleep(1)
                continue
            obs = self.robot.get_observation()
            current_joints = np.array([
                obs.get('shoulder_pan.pos'),
                obs.get('shoulder_lift.pos'),
                obs.get('elbow_flex.pos'),
                obs.get('wrist_flex.pos'),
                obs.get('wrist_roll.pos'),
                obs.get('gripper.pos'),
                ])
            self.current_joints = current_joints.astype(float)
            
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.motors
            joint_state_msg.position = np.deg2rad(self.current_joints).tolist()
            self.joint_publisher.publish(joint_state_msg)
            self.get_logger().info(f"关节角度:{np.round(np.array(self.current_joints),3)}")
            # 3. 计算正向运动学
            # 返回的是一个 4x4 的变换矩阵 (SE(3))
            self.ee_pos,self.ee_quat = self.get_ee_pos_and_quat()

            if self.is_torque_on:
                pass
                self.ik_control(self.target_xyz,self.target_quat)
                
                #self.target_xyz[0] += self.speed * self.dt
                #self.target_xyz[2] += self.speed * self.dt

                #self.gripper_degree = self.current_joints[-1]
                self.gripper_control(self.gripper_degree)
            self.get_logger().info(f"目标末端位置:{np.round(self.target_xyz,3)}")

            if self.marker_pos_camera is None:
                self.get_logger().info("相机坐标系下的抓取位姿:None")
            else:
                self.get_logger().info(f"相机坐标系下的抓取位姿:{np.round(self.marker_pos_camera,3)}")

            if self.marker_pos is None:
                self.get_logger().info("基座坐标系下的抓取位姿:None")
            else:
                self.get_logger().info(f"基座坐标系下的抓取位姿:{np.round(self.marker_pos,3)}")
            
            #action = {self.obs_keys[-1]:np.round(self.current_joints[-1]) + 2} # type:ignore
            #self.robot.send_action(action=action)
            '''
            action = {f"{self.motors[0]}.pos":0.1,
                      f"{self.motors[1]}.pos":0.1,
                      f"{self.motors[2]}.pos":0.1,
                      f"{self.motors[3]}.pos":0.1,
                      f"{self.motors[4]}.pos":0.1,
                      f"{self.motors[5]}.pos":0.1}
            self.robot.send_action(action=action)
            '''
            time.sleep(self.dt)

    def broadcast_timer(self,matrix):
        translation = matrix[0:3,3]
        rotation_matrix = matrix[0:3,0:3]
        rot = R.from_matrix(rotation_matrix)
        quat = rot.as_quat()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() # 关键：使用当前时间！
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'Link6'

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.get_logger().info(f"末端位置:{np.round(translation,3)}")
    
        self.tf_broadcaster.sendTransform(t)

    def build_target_matrix(self, xyz, quat):
        """把 XYZ + RPY 转成 4x4 变换矩阵（给 IK 用）"""
        mat = np.eye(4)
        mat[:3, 3] = xyz
        mat[:3, :3] = R.from_quat(quat).as_matrix()
        return mat
    
    def get_ee_pos_and_quat(self):
        ee_pose = self.kinematics.forward_kinematics(joint_pos_deg=self.current_joints)
        translation = ee_pose[0:3,3]
        rotation_matrix = ee_pose[0:3,0:3]
        rot = R.from_matrix(rotation_matrix)
        quat = rot.as_quat()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() # 关键：使用当前时间！
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'Link6'

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.ee_pos = translation
        self.ee_quat = quat

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"末端位置:{np.round(translation,3)}")
        
        return translation,quat
    
    def gripper_control(self,target_degree):
        self.get_logger().info(f"目标夹爪角:{target_degree}")
        obs = self.robot.get_observation()
        gripper_degree = obs.get('gripper.pos')
        if np.abs(target_degree - gripper_degree) < 1:
            return
        safe_degree = gripper_degree + np.clip(
            target_degree - gripper_degree,
            -self.max_step,self.max_step
        )
        safe_degree = np.round(safe_degree)
        action = {self.obs_keys[-1]:safe_degree}
        self.robot.send_action(action=action)

    def open_gripper(self):
        self.gripper_degree = 90

    def close_gripper(self):
        self.gripper_degree = 0

    def start_grip(self):
        if self.marker_pos is not None:
            self.target_xyz = self.marker_pos

    def _decode_voice_grasp_plan(self, raw: str):
        text = (raw or "").strip()
        if not text:
            return {"action": "VISION_GRASP", "target": "", "steps": []}
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                target = str(data.get("target") or data.get("text") or text).strip()
                steps = data.get("steps") if isinstance(data.get("steps"), list) else []
                if not steps:
                    steps = self._build_default_voice_plan(target)["steps"]
                data["action"] = "VISION_GRASP"
                data["target"] = target
                data["steps"] = steps
                return data
        except Exception:
            pass
        return self._build_default_voice_plan(text)

    def _build_default_voice_plan(self, target: str):
        return {
            "action": "VISION_GRASP",
            "target": target,
            "steps": [
                {"stage": "detect_target", "action": "VISION_GRASP", "target": target, "settle_seconds": 0.2},
                {"stage": "move_to_front", "action": "VISION_GRASP", "target": target, "approach_offset": 0.08, "settle_seconds": 0.9},
                {"stage": "grasp", "action": "VISION_GRASP", "target": target, "gripper_degree": 0, "settle_seconds": 0.8},
            ],
        }

    def _get_active_marker_pos(self):
        if self.marker_pos is not None:
            return self.marker_pos
        if self.marker_cache is not None and marker_is_fresh(self.marker_cache_time, self.marker_cache_ttl, time.time()):
            return self.marker_cache
        return None

    def _build_approach_target(self, step: dict):
        base = self._get_active_marker_pos()
        if base is None:
            return None
        approach_offset = step.get("approach_offset", 0.08)
        if isinstance(approach_offset, (list, tuple, np.ndarray)) and len(approach_offset) == 3:
            offset = np.array(approach_offset, dtype=float)
        else:
            offset = np.array([0.0, 0.0, float(approach_offset)], dtype=float)
        return np.array(base, dtype=float) + offset

    def _run_voice_grasp_plan(self, plan: dict):
        target = str(plan.get("target") or "").strip()
        steps = plan.get("steps") if isinstance(plan.get("steps"), list) else []
        if not target:
            self.get_logger().warn("语音抓取缺少目标名称")
            return

        with self._voice_grasp_lock:
            if self._voice_grasp_active:
                self.get_logger().warn("已有语音抓取任务正在执行")
                return
            self._voice_grasp_active = True

        try:
            marker = self._get_active_marker_pos()
            if marker is None:
                self.get_logger().warn("未检测到可抓取目标，请先由视觉系统输出 /gpd/grasp_markers")
                return

            if not self.is_torque_on:
                self.enable_torque()

            steps = steps or self._build_default_voice_plan(target)["steps"]
            for index, step in enumerate(steps, start=1):
                stage = str(step.get("stage") or step.get("action") or "").lower()
                self.get_logger().info(f"语音抓取步骤 {index}/{len(steps)}: {stage or 'unknown'} | {target}")

                if stage in ("detect_target", "recognize_target", "observe_target"):
                    marker = self._get_active_marker_pos()
                    if marker is None:
                        self.get_logger().warn("目标已丢失，停止语音抓取")
                        return
                    time.sleep(float(step.get("settle_seconds", 0.2)))
                    continue

                if stage in ("move_to_front", "approach", "pre_grasp"):
                    approach = self._build_approach_target(step)
                    if approach is None:
                        self.get_logger().warn("无法生成预抓取位")
                        return
                    self.target_xyz = approach
                    time.sleep(float(step.get("settle_seconds", 0.9)))
                    continue

                if stage in ("grasp", "grab", "close_gripper"):
                    marker = self._get_active_marker_pos()
                    if marker is None:
                        self.get_logger().warn("抓取阶段目标已丢失")
                        return
                    self.target_xyz = np.array(marker, dtype=float)
                    self.gripper_degree = float(step.get("gripper_degree", 0))
                    time.sleep(float(step.get("settle_seconds", 0.8)))
                    continue

                if stage in ("wait", "pause"):
                    time.sleep(float(step.get("settle_seconds", 0.5)))
                    continue

            self.get_logger().info(f"语音抓取完成: {target}")
        finally:
            with self._voice_grasp_lock:
                self._voice_grasp_active = False

    def voice_grasp_callback(self, msg: String):
        plan = self._decode_voice_grasp_plan(msg.data)
        self.get_logger().info(f"收到语音抓取指令: {plan.get('target', '')}")
        threading.Thread(target=self._run_voice_grasp_plan, args=(plan,), daemon=True).start()

    def reset_location(self):
        if self.init_pos is not None:
            self.target_xyz = self.init_pos
        if self.init_quat is not None:
            self.target_quat = self.init_quat

    
    def ik_control(self,target_xyz,target_quat):
        # --------------------- 1. 获取当前关节角度 ---------------------
        obs = self.robot.get_observation()
        current_joints = np.array([
            obs.get('shoulder_pan.pos'),
            obs.get('shoulder_lift.pos'),
            obs.get('elbow_flex.pos'),
            obs.get('wrist_flex.pos'),
            obs.get('wrist_roll.pos'),
            obs.get('gripper.pos'),
        ])
        # --------------------- 2. 构建目标 4x4 矩阵 ---------------------
        target_mat = self.build_target_matrix(target_xyz, target_quat)

        # --------------------- 3. 调用 LeRobot 官方 IK ---------------------
        target_joints = self.kinematics.inverse_kinematics(
            current_joint_pos=current_joints,
            desired_ee_pose=target_mat,
            position_weight=self.pos_weight,
            orientation_weight=self.ori_weight
        )

        # --------------------- 4. 平滑限幅（防抖动） ---------------------
        safe_joints = np.array(clamp_step(current_joints, target_joints, self.max_step))
        safe_joints = np.round(safe_joints)
        self.get_logger().info(f"目标关节角:{target_joints}")

        # --------------------- 5. 发送到机械臂 ---------------------
        action = {
            f"{self.motors[0]}.pos": safe_joints[0],
            f"{self.motors[1]}.pos": safe_joints[1],
            f"{self.motors[2]}.pos": safe_joints[2],
            f"{self.motors[3]}.pos": safe_joints[3],
            f"{self.motors[4]}.pos": safe_joints[4],
            f"{self.motors[5]}.pos": safe_joints[5],
        }
        self.robot.send_action(action=action)

    def ik_control_loop(self):
        """官方API标准 IK 控制循环"""
        while self.running:
            if not self.is_connected:
                time.sleep(0.1)
                continue

            try:
                # --------------------- 1. 获取当前关节角度 ---------------------
                obs = self.robot.get_observation()
                current_joints = np.array([
                    obs.get('shoulder_pan.pos'),
                    obs.get('shoulder_lift.pos'),
                    obs.get('elbow_flex.pos'),
                    obs.get('wrist_flex.pos'),
                    obs.get('wrist_roll.pos'),
                    obs.get('gripper.pos'),
                ])

                # --------------------- 2. 构建目标 4x4 矩阵 ---------------------
                target_mat = self.build_target_matrix(self.target_xyz, self.target_quat)

                # --------------------- 3. 调用 LeRobot 官方 IK ---------------------
                target_joints = self.kinematics.inverse_kinematics(
                    current_joint_pos=current_joints,
                    desired_ee_pose=target_mat,
                    position_weight=self.pos_weight,
                    orientation_weight=self.ori_weight
                )

                # --------------------- 4. 平滑限幅（防抖动） ---------------------
                safe_joints = np.array(clamp_step(current_joints, target_joints, self.max_step))

                # --------------------- 5. 发送到机械臂 ---------------------
                action = {
                    f"{self.motors[0]}.pos": safe_joints[0],
                    f"{self.motors[1]}.pos": safe_joints[1],
                    f"{self.motors[2]}.pos": safe_joints[2],
                    f"{self.motors[3]}.pos": safe_joints[3],
                    f"{self.motors[4]}.pos": safe_joints[4],
                    f"{self.motors[5]}.pos": safe_joints[5],
                }
                self.robot.send_action(action=action)

                # --------------------- 6. 正运动学 + TF 广播 ---------------------
                ee_pose = self.kinematics.forward_kinematics(joint_pos_deg=current_joints)
                self.broadcast_timer(ee_pose)

                # 日志
                self.get_logger().info(f"""
                    当前: {np.round(current_joints,1)} deg
                    目标: {np.round(safe_joints,1)} deg
                    目标XYZ: {np.round(self.target_xyz,3)} m
                    """)

            except Exception as e:
                self.get_logger().error(f"IK循环异常: {str(e)}")

            time.sleep(0.02)

        self.stop()


    def find_arm_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.get_logger().info(f"{port.device}:{port.description}")

    def stop(self):
        self.running = False
        self.robot.bus.disable_torque(self.motors)
        self.robot.disconnect()
        self.get_logger().info("断开连接")

def main(args=None):
    rclpy.init(args=args)
    node = TFNode()
    executor = MultiThreadedExecutor()

    def ros_spin():
        rclpy.spin(node, executor=executor)
    
    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()
    try:
        if bool(node.get_parameter("enable_gui").value):
            gui = GUI(node=node)
            gui.run()
        else:
            while rclpy.ok() and node.running:
                time.sleep(0.2)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
