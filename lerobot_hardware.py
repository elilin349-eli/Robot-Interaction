#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot SO-101 硬件通信模块 (v2.1 - 增加过载恢复 + 平滑起始帧)
修复说明：
  - 新增 clear_overload_error() 方法：重置硬件过载状态
  - 修改 safe_playback()：回放前缓慢移动到第一帧，防止瞬间跳跃引发过载
  - 修改 set_torque()：延长释放等待时间，增加重试机制
"""

import json
import time
import os
import threading
import re
from datetime import datetime
from typing import Optional, List, Dict, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    import threading
from pathlib import Path

# 【兼容性补丁】舵机底层 SDK 别名映射
try:
    import scservo_sdk
except ImportError:
    try:
        import feetech_servo_sdk as scservo_sdk
    except ImportError:
        pass


def check_serial_port(port):
    import serial.tools.list_ports

    ports = [p.device for p in serial.tools.list_ports.comports()]
    if port not in ports:
        raise RuntimeError(f"Serial port {port} not found")


class MotorWrapper:
    def __init__(self, motor_id: int, model: str = "sts3215"):
        self.id = motor_id
        self.model = model


class SO101LeaderArm:
    """SO-101 主臂/从臂硬件接口 - FeetechMotorsBus 直接集成"""
    
    REGISTER_CURRENT_POS = 56
    REGISTER_TARGET_POS = 42
    REGISTER_TORQUE_ENABLE = 40
    REGISTER_LENGTH = 2
    TORQUE_ENABLE_LENGTH = 1
    POS_MIN = 0
    POS_MAX = 4095
    JOINT_OFFSETS = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
    
    def __init__(self, port="COM12", baudrate=115200, timeout=1.0, role="leader"):
        self.port = port
        self.role = role
        self.baudrate = baudrate
        self.timeout = timeout
        self.bus = None
        self.connected = False
        self._io_lock = threading.Lock()
        self.available_motors = [1, 2, 3, 4, 5, 6]
        self.calib_path = self._get_calib_path()
        self.JOINT_CALIB: Dict[int, Tuple[float, float]] = {}
        self.range_limits: Dict[int, Tuple[int, int]] = {i: (self.POS_MIN, self.POS_MAX) for i in range(1, 7)}
        self.JOINT_OFFSETS = dict(self.JOINT_OFFSETS)
        self._load_range_limits()
        self._init_bus()
        if self.role == "follower":
            try:
                offsets_file = self._get_joint_offsets_path()
                if os.path.exists(offsets_file):
                    with open(offsets_file, 'r', encoding='utf-8') as fh:
                        data = json.load(fh)
                    loaded = {int(k): int(v) for k, v in (data.items() if isinstance(data, dict) else [])}
                    if loaded:
                        self.JOINT_OFFSETS = loaded
                        print(f"✅ 已从 {offsets_file} 加载 JOINT_OFFSETS: {self.JOINT_OFFSETS}")
            except Exception as e:
                print(f"⚠️ 自动加载 joint_offsets 失败: {e}")
        else:
            self.JOINT_OFFSETS = {i: 0 for i in range(1, 7)}
        
    def _get_calib_path(self) -> str:
        try:
            from control_pkg.robot_system.utils.path_manager import PathManager

            candidate = (
                PathManager.follower_calibration_file()
                if self.role == "follower"
                else PathManager.leader_calibration_file()
            )
            if candidate.exists():
                return str(candidate)
        except Exception:
            pass
        return ""

    def _get_joint_offsets_path(self) -> str:
        try:
            from control_pkg.robot_system.utils.path_manager import PathManager

            return str(PathManager.joint_offsets_file())
        except Exception:
            return os.path.join(os.path.dirname(__file__), 'joint_offsets.json')

    def _load_range_limits(self):
        self.range_limits = {i: (self.POS_MIN, self.POS_MAX) for i in range(1, 7)}
        try:
            if not self.calib_path:
                print(f"⚠️ [{self.role}] 未提供标定文件，使用全范围")
                return
            if not os.path.exists(self.calib_path):
                raise FileNotFoundError(self.calib_path)
            with open(self.calib_path, 'r', encoding='utf-8') as f:
                calib = json.load(f)
            if not isinstance(calib, dict):
                raise ValueError("invalid calibration format")

            loaded = {}
            for data in calib.values():
                if not isinstance(data, dict):
                    continue
                jid = data.get('id')
                lo = data.get('range_min')
                hi = data.get('range_max')
                if jid is None or lo is None or hi is None:
                    continue
                jid = int(jid)
                lo = int(lo)
                hi = int(hi)
                if lo > hi:
                    lo, hi = hi, lo
                loaded[jid] = (lo, hi)

            if loaded:
                self.range_limits.update(loaded)
            print(f"✅ [{self.role}] 已加载关节限位: {self.range_limits}")
        except Exception as e:
            print(f"⚠️ [{self.role}] 加载限位失败: {e}，使用全范围")
    
    def _init_bus(self):
        FeetechMotorsBus = None
        try:
            from lerobot.motors.feetech import FeetechMotorsBus
            print("✅ 导入模式 1: lerobot.motors.feetech (0.4.4+)")
        except ImportError:
            try:
                from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
                print("✅ 导入模式 2: lerobot.common.robot_devices.motors.feetech (0.3.x)")
            except ImportError:
                try:
                    from lerobot.robot_devices.motors.feetech import FeetechMotorsBus
                    print("✅ 导入模式 3: lerobot.robot_devices.motors.feetech (early dev)")
                except ImportError:
                    print("⚠️  LeRobot 环境缺失或版本不兼容，使用降级模式")
                    self.bus = None
                    return
        
        try:
            motor_objects = {i: MotorWrapper(i) for i in range(1, 7)}
            self.bus = FeetechMotorsBus(port=self.port, motors=motor_objects)
        except Exception as e:
            print(f"⚠️  FeetechMotorsBus 初始化失败: {e}")
            self.bus = None

    def _low_level_write(self, addr: int, length: int, motor_id: int, value, retries: int = 3, delay: float = 0.02) -> bool:
        """Central low-level write wrapper with retries.

        Uses the underlying bus._write(addr, length, motor_id, value) signature.
        Returns True on success, False on persistent failure.
        """
        if not self.bus:
            return False
        for attempt in range(retries):
            try:
                with self._io_lock:
                    self.bus._write(addr, length, motor_id, int(value))
                return True
            except Exception:
                time.sleep(delay)
                continue
        return False
    
    def connect(self) -> bool:
        try:
            try:
                check_serial_port(self.port)
            except Exception as e:
                print(f"⚠️  串口检查失败: {e}")
                return False

            if self.bus is None:
                print("⚠️  FeetechMotorsBus 未初始化，尝试重新初始化")
                self._init_bus()
            
            if self.bus:
                try:
                    self.bus.connect()
                except Exception as motor_check_error:
                    error_msg = str(motor_check_error)
                    if "motor check failed" in error_msg or "Missing motor IDs" in error_msg:
                        print(f"⚠️  舵机部分故障: {error_msg}")
                        missing_match = re.search(r"Missing motor IDs:\s*\[([^\]]*)\]", error_msg)
                        if missing_match:
                            missing_text = missing_match.group(1).strip()
                            missing_ids = [int(x.strip()) for x in missing_text.split(',') if x.strip().isdigit()]
                            if missing_ids:
                                self.available_motors = [m for m in range(1, 7) if m not in missing_ids]
                                print(f"⚠️  降级可用舵机: {self.available_motors}")
                        self.connected = True
                        return True
                    else:
                        raise
                
                self.connected = True
                self.available_motors = [1, 2, 3, 4, 5, 6]
                print(f"✅ 已连接 ({self.port})")
                return True
            else:
                print("❌ 无法初始化 FeetechMotorsBus")
                return False
        except Exception as e:
            print(f"❌ 无法连接: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> bool:
        try:
            if self.bus and self.connected:
                self.bus.disconnect()
            self.connected = False
            print("👋 连接已关闭")
            return True
        except Exception as e:
            print(f"⚠️  关闭连接时出错: {e}")
            return False

    # ===================================================================
    # 【修复 v2.1】clear_overload_error - 重置硬件过载保护状态
    # ===================================================================
    def clear_overload_error(self):
        """
        清除所有电机的硬件过载错误。
        
        STS3215 过载保护原理：一旦触发过载，即使接收到 torque_enable=0 的写入，
        电机也可能拒绝响应。本方法通过：
        1. 向每个电机连续发送多次 torque_disable（忽略错误）
        2. 等待足够时间让硬件状态复位
        3. 重新发送 torque_enable=1 并验证响应
        
        调用时机：每次执行 safe_playback 之前调用。
        """
        if not self.bus:
            return
        
        print("🔄 正在清除电机过载状态...")
        
        # 第一步：连续多次尝试 torque disable，忽略单次失败
        for attempt in range(3):
            for motor_id in list(self.available_motors):
                try:
                    # ignore single failures during overload clear
                    self._low_level_write(self.REGISTER_TORQUE_ENABLE, self.TORQUE_ENABLE_LENGTH, motor_id, 0)
                except Exception:
                    pass
            time.sleep(0.1)
        
        # 第二步：等待硬件状态稳定（必须足够长）
        time.sleep(0.8)
        
        # 第三步：重新使能力矩，验证每个电机响应
        ok_motors = []
        fail_motors = []
        for motor_id in list(self.available_motors):
            try:
                if self._low_level_write(self.REGISTER_TORQUE_ENABLE, self.TORQUE_ENABLE_LENGTH, motor_id, 1):
                    ok_motors.append(motor_id)
                else:
                    fail_motors.append(motor_id)
                    print(f"⚠️  电机 {motor_id} 力矩恢复失败（可能仍在过载）")
            except Exception as e:
                fail_motors.append(motor_id)
                print(f"⚠️  电机 {motor_id} 力矩恢复失败（可能仍在过载）: {e}")
        
        if ok_motors:
            print(f"✅ 电机力矩恢复: {ok_motors}")
        if fail_motors:
            print(f"⚠️  以下电机过载未解除，回放中将跳过: {fail_motors}")
            self.available_motors = [m for m in self.available_motors if m not in fail_motors]
            
        # 新增下面这一行：只要还有可用的电机，就告诉上层恢复成功了
        return len(self.available_motors) > 0
    
    def set_torque(self, enable: bool):
        """
        统一控制所有电机的力矩使能状态
        【v2.1 修复】过载时不再静默跳过，而是重试一次
        """
        val = 1 if enable else 0
        if not self.bus:
            return
        with self._io_lock:
            for motor_id in list(self.available_motors):
                for attempt in range(2):  # 最多重试 1 次
                    ok = self._low_level_write(self.REGISTER_TORQUE_ENABLE, self.TORQUE_ENABLE_LENGTH, motor_id, val)
                    if ok:
                        break
                    if attempt == 1:
                        print(f"⚠️  设置电机 {motor_id} 力矩失败")
                    time.sleep(0.05)
    
    def read_angles(self) -> Optional[List[int]]:
        """读取 6 个关节的当前位置（编码值 0-4095）"""
        if not self.connected or not self.bus:
            return None
        
        try:
            angles = []
            with self._io_lock:
                for motor_id in range(1, 7):
                    if motor_id not in self.available_motors:
                        angles.append(2048)  # 默认中位，不用 0 避免安全范围问题
                        continue
                    result = self.bus._read(
                        self.REGISTER_CURRENT_POS,
                        self.REGISTER_LENGTH,
                        motor_id
                    )
                    if isinstance(result, (tuple, list)) and len(result) >= 1:
                        val = result[0]
                    else:
                        val = result
                    if val is not None:
                        angles.append(int(val))
                    else:
                        return None
            return angles
        except Exception as e:
            print(f"⚠️  读取关节位置时出错: {e}")
            return None

    def set_joint_calib(self, calib: Dict[int, Tuple[float, float]]):
        """设置线性标定系数（slope, intercept）并进行简单校验。"""
        safe = {}
        for j in range(1, 7):
            v = calib.get(j)
            if not v:
                continue
            try:
                slope = float(v[0])
                intercept = float(v[1])
            except Exception:
                continue
            # 基本合理性检测，拒绝极端拟合值
            if abs(slope) > 5 or abs(intercept) > 10000:
                print(f"⚠️ 忽略异常标定 J{j}: slope={slope}, intercept={intercept}")
                continue
            safe[j] = (slope, intercept)
        self.JOINT_CALIB = safe
        if safe:
            print(f"✅ 已加载 JOINT_CALIB: {safe}")

    def load_joint_calib(self, path: str) -> bool:
        try:
            if not os.path.exists(path):
                return False
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # expect {"1": [slope, intercept], ...} or {1: [..]}
            calib = {int(k): tuple(v) for k, v in (data.items() if isinstance(data, dict) else [])}
            self.set_joint_calib(calib)
            return True
        except Exception as e:
            print(f"⚠️ 加载标定文件失败: {e}")
            return False

    def compute_offsets_from_pairs(self, pairs: List[Tuple[List[int], List[int]]]):
        """从多组 (leader_angles, follower_angles) 对计算稳健偏移（中位数），并设置为 JOINT_OFFSETS.
        每个 pair 是两个长度为6的列表，表示在相同物理姿态下的读数。
        """
        if not pairs:
            return {}
        per_joint_diffs = {i: [] for i in range(1, 7)}
        for leader, follower in pairs:
            if not leader or not follower or len(leader) < 6 or len(follower) < 6:
                continue
            for i in range(6):
                per_joint_diffs[i + 1].append(int(follower[i]) - int(leader[i]))

        offsets = {}
        for j in range(1, 7):
            vals = per_joint_diffs.get(j, [])
            if not vals:
                continue
            vals_sorted = sorted(vals)
            mid = vals_sorted[len(vals_sorted) // 2]
            offsets[j] = int(mid)
        self.JOINT_OFFSETS = offsets
        print(f"✅ 已计算并设置 JOINT_OFFSETS: {offsets}")
        return offsets

    def load_joint_offsets_file(self, path: Optional[str] = None) -> bool:
        """从文件加载 joint_offsets.json（支持字符串或数字键），并更新 self.JOINT_OFFSETS"""
        try:
            if path is None:
                path = self._get_joint_offsets_path()
            if not os.path.exists(path):
                return False
            with open(path, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
            loaded = {}
            for k, v in (data.items() if isinstance(data, dict) else []):
                try:
                    ik = int(k)
                    iv = int(v)
                except Exception:
                    continue
                # range protection
                if abs(iv) > 2000:
                    print(f"⚠️ joint_offsets 值异常，已修正: J{ik}={iv}")
                    iv = max(-2000, min(2000, iv))
                loaded[ik] = iv
            if loaded:
                self.JOINT_OFFSETS = loaded
                print(f"✅ 已从 {path} 加载 JOINT_OFFSETS: {self.JOINT_OFFSETS}")
                return True
            return False
        except Exception as e:
            print(f"⚠️ 加载 joint_offsets 失败: {e}")
            return False

    def save_joint_offsets_file(self, path: Optional[str] = None) -> bool:
        """将当前 self.JOINT_OFFSETS 保存到 joint_offsets.json（备份原文件）"""
        try:
            if path is None:
                path = self._get_joint_offsets_path()
            # backup
            if os.path.exists(path):
                try:
                    os.replace(path, path + '.bak')
                except Exception:
                    pass
            # enforce sane ranges before saving
            out_data = {}
            for k, v in (self.JOINT_OFFSETS.items() if isinstance(self.JOINT_OFFSETS, dict) else []):
                try:
                    ik = int(k)
                    iv = int(v)
                except Exception:
                    continue
                if abs(iv) > 2000:
                    iv = max(-2000, min(2000, iv))
                out_data[str(ik)] = int(iv)
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(out_data, fh, ensure_ascii=False, indent=2)
            print(f"✅ 已保存 JOINT_OFFSETS 到: {path}")
            return True
        except Exception as e:
            print(f"⚠️ 保存 JOINT_OFFSETS 失败: {e}")
            return False

    def _apply_joint_limit(self, motor_id: int, target_pos: int) -> int:
        lo, hi = self.range_limits.get(motor_id, (self.POS_MIN, self.POS_MAX))
        return max(int(lo), min(int(hi), int(target_pos)))

    def map_leader_to_follower(self, leader_angles: List[int]) -> List[int]:
        """将主臂编码值映射为从臂目标。"""
        targets = []
        for i in range(6):
            a = int(leader_angles[i]) if i < len(leader_angles) and leader_angles[i] is not None else 0
            j = i + 1
            if j in self.JOINT_CALIB:
                slope, intercept = self.JOINT_CALIB[j]
                try:
                    val = int(round(slope * a + intercept))
                except Exception:
                    val = a
            elif j in self.JOINT_OFFSETS:
                val = a + int(self.JOINT_OFFSETS[j])
            else:
                val = a
            targets.append(self._apply_joint_limit(j, val))
        return targets
    
    def safe_write(self, motor_id: int, target_pos: int) -> bool:
        """单舵机安全写入"""
        if not self.connected or not self.bus:
            return False
        if motor_id not in self.available_motors:
            print(f"⚠️  电机 {motor_id} 不可用，跳过写入")
            return False
        
        try:
            target_pos = self._apply_joint_limit(motor_id, target_pos)
            # ensure torque on, then write target
            with self._io_lock:
                self._low_level_write(self.REGISTER_TORQUE_ENABLE, self.TORQUE_ENABLE_LENGTH, motor_id, 1)
                ok = self._low_level_write(self.REGISTER_TARGET_POS, self.REGISTER_LENGTH, motor_id, int(target_pos))
            return bool(ok)
        except Exception as e:
            print(f"⚠️  舵机 {motor_id} 写入失败: {e}")
            return False
    
    # ===================================================================
    # 【修复 v2.1】safe_playback - 增加平滑起始帧 + 更健壮的过载处理
    # ===================================================================
    def safe_playback(
        self,
        sequence: List[Dict],
        stop_signal: Optional['threading.Event'] = None,
        use_interpolation: bool = False
    ) -> bool:
        """
        安全回放动作序列。
        
        v2.1 关键修复：
        1. 【平滑起始】回放前，先用 1 秒时间从当前位置缓慢移动到第一帧，
           防止瞬间跳跃导致过载（这是"只有6号电机动"的根本原因之一）
        2. 【过载跳过】某个电机出现过载时跳过该帧，不中断整体回放
        3. 【帧间延时】从 0.08s 增加到 0.1s，降低整体负载
        """
        if not self.connected or not self.bus:
            print("⚠️  未连接，无法回放")
            return False
        
        if not sequence:
            print("⚠️  动作序列为空")
            return False
        
        # --- 阶段一：平滑移动到第一帧 ---
        first_angles = sequence[0].get('angles', [])
        if len(first_angles) == 6:
            current_angles = self.read_angles()
            if current_angles and len(current_angles) == 6:
                print(f"🎯 平滑移动到起始帧（共 20 步，约 1s）...")
                APPROACH_STEPS = 20
                for step in range(1, APPROACH_STEPS + 1):
                    if stop_signal and stop_signal.is_set():
                        print("🛑 起始帧移动被中断")
                        return False

                    ratio = step / APPROACH_STEPS
                    for i, (cur, tgt) in enumerate(zip(current_angles, first_angles)):
                        motor_id = i + 1
                        if motor_id not in self.available_motors:
                            continue
                        interp_pos = int(cur + (tgt - cur) * ratio)
                        safe_pos = self._apply_joint_limit(motor_id, interp_pos)
                        try:
                            self._low_level_write(self.REGISTER_TARGET_POS, self.REGISTER_LENGTH, motor_id, int(safe_pos))
                        except Exception:
                            continue
                    time.sleep(0.05)  # 20步 × 50ms = 1秒
                
                print("✅ 已到达起始帧，开始正式回放")
        
        # --- 阶段二：正式回放 ---
        try:
            prev_angles = None
            for frame_idx, frame in enumerate(sequence):
                if stop_signal and stop_signal.is_set():
                    print("🛑 回放被中断")
                    return False
                
                angles = frame.get('angles', [])
                if len(angles) != 6:
                    continue
                
                if use_interpolation and prev_angles:
                    for step in range(1, 3):
                        interp = [int(p + (t - p) * step / 2) for p, t in zip(prev_angles, angles)]
                        for i, pos in enumerate(interp):
                            motor_id = i + 1
                            if motor_id not in self.available_motors:
                                continue
                            safe_pos = self._apply_joint_limit(motor_id, pos)
                            try:
                                self._low_level_write(self.REGISTER_TARGET_POS, self.REGISTER_LENGTH, motor_id, int(safe_pos))
                            except Exception:
                                continue
                        time.sleep(0.02)

                for i, pos in enumerate(angles):
                    motor_id = i + 1
                    if motor_id not in self.available_motors:
                        continue
                    safe_pos = self._apply_joint_limit(motor_id, pos)
                    try:
                        self._low_level_write(self.REGISTER_TARGET_POS, self.REGISTER_LENGTH, motor_id, int(safe_pos))
                    except Exception as e:
                        if "overload" in str(e).lower():
                            continue
                        print(f"⚠️  电机 {motor_id} 回放失败: {e}")
                        continue
                
                prev_angles = angles
                time.sleep(0.1)  # 【v2.1】从 0.08 增加到 0.1，减轻总线负荷
            
            print("✅ 动作回放完成")
            return True
        except Exception as e:
            print(f"❌ 回放出错: {e}")
            return False
    
    def write_angles(self, angles: List[int]) -> bool:
        if not self.connected or not self.bus:
            return False
        if len(angles) != 6:
            return False
        try:
            for motor_id, target_pos in enumerate(angles, start=1):
                if motor_id not in self.available_motors:
                    continue
                target_pos = self._apply_joint_limit(motor_id, target_pos)
                # ensure torque enabled then write via central low-level writer
                ok = self._low_level_write(self.REGISTER_TORQUE_ENABLE, self.TORQUE_ENABLE_LENGTH, motor_id, 1)
                if not ok:
                    print(f"⚠️  电机 {motor_id} 力矩启用失败")
                ok = self._low_level_write(self.REGISTER_TARGET_POS, self.REGISTER_LENGTH, motor_id, int(target_pos))
                if not ok:
                    print(f"❌ 电机 {motor_id} 位置写入失败")
                    return False
            return True
        except Exception as e:
            print(f"❌ 写入关节位置时出错: {e}")
            return False
    
    def get_status(self) -> Dict:
        return {
            "port": self.port,
            "connected": self.connected,
            "timestamp": datetime.now().isoformat(),
            "available_motors": self.available_motors,
        }

    def normalize_action(self, raw_angles: List[int]) -> List[float]:
        scale = self.POS_MAX - self.POS_MIN
        return [round(((a - self.POS_MIN) / scale) * 2.0 - 1.0, 4) for a in raw_angles]

    def denormalize_action(self, model_actions: List[float]) -> List[int]:
        scale = self.POS_MAX - self.POS_MIN
        return [int((max(-1.0, min(1.0, a)) + 1.0) / 2.0 * scale + self.POS_MIN) for a in model_actions]


class GestureRecorder:
    """手势/姿态记录器"""
    
    def __init__(self, filename: str = "gestures_library.json"):
        self.filename = filename
        self.library = self._load_library()
        self.current_recording = []
        self.lock = threading.Lock()
    
    def _load_library(self) -> Dict:
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    return data if isinstance(data, dict) else {}
            except Exception:
                return {}
        return {}
    
    def record_frame(self, angles: List[int]):
        if len(angles) != 6:
            return False
        with self.lock:
            self.current_recording.append({
                'timestamp': time.time(),
                'angles': list(angles)
            })
        return True
    
    def save_named_action(self, name: str) -> bool:
        with self.lock:
            if not self.current_recording:
                print(f"⚠️  当前无录制数据")
                return False
            recording_copy = list(self.current_recording)
            self.current_recording = []
        
        try:
            self.library[name] = recording_copy
            with open(self.filename, 'w', encoding='utf-8') as f:
                json.dump(self.library, f, ensure_ascii=False, indent=4)
            print(f"✅ 已存储动作库中: {name} (共 {len(recording_copy)} 帧)")
            return True
        except Exception as e:
            print(f"❌ 存储失败: {e}")
            return False
    
    def record_gesture(self, angles: List[int], label: str = "", metadata: Optional[Dict] = None):
        if len(angles) != 6:
            return False
        record = {
            "timestamp": datetime.now().isoformat(),
            "angles": list(angles),
            "label": label,
            "metadata": metadata or {}
        }
        with self.lock:
            self.current_recording.append(record)
        print(f"✅ 已记录姿态: {label}")
        return True
    
    def get_frame_count(self) -> int:
        return len(self.current_recording)

    def get_action_count(self) -> int:
        return len(self.library)

    def get_count(self) -> int:
        return self.get_action_count()
    
    def export_csv(self, csv_file="gestures.csv"):
        try:
            import csv
            with open(csv_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["action_name", "frame_index", "timestamp",
                                 "angle_1", "angle_2", "angle_3", "angle_4", "angle_5", "angle_6"])
                for action_name, frames in self.library.items():
                    for frame_idx, frame in enumerate(frames):
                        angles = frame.get('angles', [0]*6)
                        writer.writerow([action_name, frame_idx, frame.get('timestamp', ''), *angles])
            print(f"✅ 已导出 CSV: {csv_file}")
            return True
        except Exception as e:
            print(f"❌ 导出 CSV 失败: {e}")
            return False

    def export_vla_dataset(self, output_dir="dataset", enable_vision=False) -> bool:
        if not self.library:
            return False
        os.makedirs(output_dir, exist_ok=True)
        vla_dataset = []
        for name, frames in self.library.items():
            image_path = ""
            if enable_vision:
                try:
                    import cv2
                    cap = cv2.VideoCapture(0)
                    ret, frame_img = cap.read()
                    if ret:
                        img_dir = os.path.join(output_dir, "images")
                        os.makedirs(img_dir, exist_ok=True)
                        image_path = os.path.join(img_dir, f"{name}_{int(time.time())}.jpg")
                        cv2.imwrite(image_path, frame_img)
                    cap.release()
                except Exception as e:
                    print(f"⚠️  摄像头抓拍失败: {e}")
            vla_dataset.append({"instruction": name, "image_path": image_path, "sequence": frames})
        
        export_path = os.path.join(output_dir, "vla_dataset.json")
        try:
            with open(export_path, 'w', encoding='utf-8') as f:
                json.dump(vla_dataset, f, ensure_ascii=False, indent=2)
            print(f"✅ VLA 数据集已导出: {export_path} ({len(vla_dataset)} 条)")
            return True
        except Exception as e:
            print(f"❌ 导出失败: {e}")
            return False


if __name__ == "__main__":
    import sys
    arm = SO101LeaderArm(port="COM12")
    recorder = GestureRecorder("test_gestures.json")
    
    if not arm.connect():
        print("❌ 无法连接主臂")
        sys.exit(1)
    
    for i in range(3):
        angles = arm.read_angles()
        if angles:
            print(f"  第 {i+1} 次: {angles}")
            recorder.record_gesture(angles, label=f"sample_{i+1}")
        time.sleep(0.5)
    
    arm.disconnect()
    recorder.export_csv("test_gestures.csv")
    print(f"✅ 动作库共 {recorder.get_action_count()} 个动作")
    print(f"✅ 已保存到 test_gestures.json 和 test_gestures.csv")


