#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot SO-101 硬件通信模块 (v2.2)
直接使用 scservo_sdk.sms_sts，绕开 FeetechMotorsBus 兼容性问题
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
    import os

    # Windows serial names (COMx) are valid even when os.path.exists is False.
    if isinstance(port, str) and re.match(r"^COM\d+$", port, re.IGNORECASE):
        return
    if isinstance(port, str) and port.upper().startswith("\\\\.\\COM"):
        return

    if not os.path.exists(port):
        raise RuntimeError(f"Serial port {port} not found")
    real = os.path.realpath(port)
    if not os.path.exists(real):
        raise RuntimeError(f"Symlink {port} -> {real} broken")


class SO101LeaderArm:
    """SO-101 主臂/从臂硬件接口 - 直接使用 scservo_sdk.sms_sts"""

    REGISTER_CURRENT_POS = 56
    REGISTER_TARGET_POS = 42
    REGISTER_TORQUE_ENABLE = 40
    REGISTER_LENGTH = 2
    TORQUE_ENABLE_LENGTH = 1
    POS_MIN = 0
    POS_MAX = 4095
    DEFAULT_SPEED = 1500
    DEFAULT_ACC = 50
    JOINT_OFFSETS = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}

    def __init__(self, port="COM12", baudrate=1000000, timeout=1.0, role="leader"):
        self.port = port
        self.role = role
        self.baudrate = baudrate
        self.timeout = timeout
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
        """获取限位/标定文件路径"""
        # 优先使用你提供的绝对路径
        if self.role == "follower":
            path = r"E:\OneDrive\桌面\Voice-Robot\calibration\follower.json"
            if os.path.exists(path):
                return path

        # 如果绝对路径不存在，尝试原始的逻辑
        try:
            from control_pkg.robot_system.utils.path_manager import PathManager
            candidate = PathManager.follower_calibration_file() if self.role == "follower" else PathManager.leader_calibration_file()
            if candidate.exists():
                return str(candidate)
        except Exception:
            pass
        return ""

    def _get_joint_offsets_path(self) -> str:
        """获取主从臂偏移量文件路径"""
        # 强制指向你生成的标定文件
        path = r"E:\OneDrive\桌面\Voice-Robot\calibration\joint_offsets.json"
        if os.path.exists(path):
            return path

        # 备选逻辑
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
        """直接使用 scservo_sdk.sms_sts，绕开 FeetechMotorsBus 兼容性问题"""
        try:
            import scservo_sdk as scs
            from scservo_sdk import sms_sts
            self._scs = scs
            self._sms_sts_cls = sms_sts
            self.port_handler = scs.PortHandler(self.port)
            self.sms = None
            print(f"✅ scservo_sdk sms_sts 模式已就绪 (端口: {self.port})")
        except Exception as e:
            print(f"❌ scservo_sdk 初始化失败: {e}")
            self.port_handler = None
            self._scs = None
            self._sms_sts_cls = None

    def _low_level_write(self, addr: int, length: int, motor_id: int, value,
                         retries: int = 3, delay: float = 0.02) -> bool:
        """低层写入（用于 clear_overload_error 等内部调用）"""
        if not self.sms or not self.port_handler:
            return False
        for _ in range(retries):
            try:
                with self._io_lock:
                    if length == 1:
                        ret, err = self.sms.write1ByteTxRx(motor_id, addr, int(value))
                        if ret == self._scs.COMM_SUCCESS and err == 0:
                            return True
                    else:
                        if addr == self.REGISTER_TARGET_POS:
                            ret = self.sms.WritePosEx(motor_id, int(value), self.DEFAULT_SPEED, self.DEFAULT_ACC)
                            if ret == self._scs.COMM_SUCCESS:
                                return True
                        else:
                            ret, err = self.sms.write2ByteTxRx(motor_id, addr, int(value))
                            if ret == self._scs.COMM_SUCCESS and err == 0:
                                return True
            except Exception:
                pass
            time.sleep(delay)
        return False

    def _read_pos(self, motor_id: int):
        if not self.sms:
            return None, None, None
        result = self.sms.ReadPos(motor_id)
        if isinstance(result, (list, tuple)):
            if len(result) == 3:
                pos, comm_result, err = result
                return pos, comm_result, err
            if len(result) == 2:
                pos, err = result
                comm_result = self._scs.COMM_SUCCESS if self._scs else 0
                return pos, comm_result, err
        return None, None, None

    def connect(self) -> bool:
        if not self.port_handler:
            return False
        try:
            check_serial_port(self.port)
        except Exception as e:
            print(f"⚠️  串口检查失败: {e}")
            return False

        try:
            if not self.port_handler.openPort():
                print(f"❌ 无法打开串口 {self.port}")
                return False
        except Exception as e:
            print(f"❌ 打开串口异常 {self.port}: {e}")
            self.connected = False
            self.sms = None
            return False

        try:
            if not self.port_handler.setBaudRate(self.baudrate):
                print(f"❌ 波特率设置失败 {self.baudrate}")
                self.port_handler.closePort()
                return False
        except Exception as e:
            print(f"❌ 设置波特率异常 {self.baudrate}: {e}")
            try:
                self.port_handler.closePort()
            except Exception:
                pass
            self.connected = False
            self.sms = None
            return False

        self.sms = self._sms_sts_cls(self.port_handler)

        ok_motors = []
        fail_motors = []
        for mid in range(1, 7):
            success = False
            last_error = None
            for attempt in range(3):
                try:
                    time.sleep(0.05)
                    pos, comm_result, err = self._read_pos(mid)
                    if comm_result == self._scs.COMM_SUCCESS and err == 0 and pos is not None:
                        ok_motors.append(mid)
                        success = True
                        break
                    last_error = f"comm={comm_result}, err={err}"
                except Exception as exc:
                    last_error = str(exc)
                time.sleep(0.1 * (attempt + 1))
            if not success:
                fail_motors.append(mid)
                if last_error:
                    print(f"⚠️  电机 {mid} 读取失败: {last_error}")

        if not ok_motors:
            print(f"❌ 没有检测到任何舵机，请检查电源和线缆")
            self.port_handler.closePort()
            return False

        if fail_motors:
            print(f"⚠️  部分舵机无响应: {fail_motors}，降级运行: {ok_motors}")

        self.available_motors = ok_motors
        self.connected = True
        print(f"✅ 已连接 {self.port}，在线舵机: {ok_motors}")
        return True

    def disconnect(self) -> bool:
        try:
            if self.port_handler:
                self.port_handler.closePort()
        except Exception:
            pass
        self.connected = False
        self.sms = None
        print("👋 连接已关闭")
        return True

    def clear_overload_error(self):
        """清除所有电机的硬件过载错误"""
        if not self.sms:
            return

        print("🔄 正在清除电机过载状态...")

        for attempt in range(3):
            for motor_id in list(self.available_motors):
                try:
                    self._low_level_write(self.REGISTER_TORQUE_ENABLE, self.TORQUE_ENABLE_LENGTH, motor_id, 0)
                except Exception:
                    pass
            time.sleep(0.1)

        time.sleep(0.8)

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

        return len(self.available_motors) > 0

    def set_torque(self, enable: bool) -> bool:
        """使能/释放所有在线舵机力矩"""
        if not self.connected or not self.sms:
            return False
        val = 1 if enable else 0
        success_count = 0
        with self._io_lock:
            for mid in list(self.available_motors):
                try:
                    ret, err = self.sms.write1ByteTxRx(mid, self.REGISTER_TORQUE_ENABLE, val)
                    if ret == self._scs.COMM_SUCCESS and err == 0:
                        success_count += 1
                    time.sleep(0.02)
                except Exception as e:
                    print(f"⚠️  力矩设置失败 motor={mid}: {e}")
        print(f"✅ 力矩{'使能' if enable else '释放'} ({success_count}/{len(self.available_motors)}台)")
        return success_count > 0

    def read_angles(self) -> Optional[List[int]]:
        """读取 6 个关节的当前位置（编码值 0-4095）"""
        if not self.connected or not self.sms:
            return None
        results = []
        with self._io_lock:
            for mid in range(1, 7):
                if mid not in self.available_motors:
                    results.append(2048)
                    continue
                try:
                    pos, comm_result, err = self._read_pos(mid)
                    if comm_result == self._scs.COMM_SUCCESS and err == 0 and pos is not None:
                        results.append(pos)
                    else:
                        results.append(2048)
                except Exception:
                    results.append(2048)
        return results if any(v != 2048 for v in results) else None

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
        """写目标位置（带范围限制，保护硬件）"""
        if not self.connected or not self.sms:
            return False

        # 1. 检查电机是否在可用列表中
        if motor_id not in self.available_motors:
            return False

        # 2. 范围夹紧：确保位置不会超过物理限位
        lo, hi = self.range_limits.get(motor_id, (self.POS_MIN, self.POS_MAX))
        position = max(lo, min(hi, int(target_pos)))

        # 3. 线程安全写入
        with self._io_lock:
            try:
                # 使用你之前配置的 DEFAULT_SPEED 和 DEFAULT_ACC
                ret = self.sms.WritePosEx(motor_id, position, self.DEFAULT_SPEED, self.DEFAULT_ACC)
                return ret == self._scs.COMM_SUCCESS
            except Exception as e:
                print(f"⚠️  safe_write 失败 motor={motor_id}: {e}")
                return False

    def safe_playback(self, sequence: List[Dict], stop_signal: Optional[threading.Event] = None,
                      use_interpolation: bool = False) -> bool:
        """安全回放动作序列，全兼容时间格式版"""
        if not self.connected or not self.sms:
            print("⚠️  未连接，无法回放")
            return False

        if not sequence:
            print("⚠️  动作序列为空")
            return False

        # --- 预处理：强制转换所有数据类型，支持 ISO 时间戳 ---
        try:
            for frame in sequence:
                if 'angles' in frame:
                    frame['angles'] = [int(float(a)) for a in frame['angles']]

                if 'timestamp' in frame and frame['timestamp'] is not None:
                    ts_val = frame['timestamp']
                    if isinstance(ts_val, str) and 'T' in ts_val:
                        # 处理像 '2026-05-13T09:36:59.545158' 这样的 ISO 格式
                        from datetime import datetime
                        frame['timestamp'] = datetime.fromisoformat(ts_val).timestamp()
                    else:
                        # 处理纯数字格式
                        frame['timestamp'] = float(ts_val)
        except Exception as e:
            print(f"❌ 数据格式预处理失败: {e}")
            return False

        # 1. 平滑移动到起始帧逻辑
        first_angles = sequence[0].get('angles', [])
        if len(first_angles) == 6:
            current_angles = self.read_angles()
            if current_angles and len(current_angles) == 6:
                current_angles = [int(float(a)) for a in current_angles]

                print(f"🎯 平滑移动到起始帧（共 20 步，约 1s）...")
                APPROACH_STEPS = 20
                for step in range(1, APPROACH_STEPS + 1):
                    if stop_signal and stop_signal.is_set():
                        return False

                    ratio = step / APPROACH_STEPS
                    for i, (cur, tgt) in enumerate(zip(current_angles, first_angles)):
                        motor_id = i + 1
                        if motor_id not in self.available_motors:
                            continue
                        interp_pos = int(cur + (tgt - cur) * ratio)
                        self.safe_write(motor_id, interp_pos)
                    time.sleep(0.05)

                print("✅ 已到达起始帧，开始正式回放")

        # 2. 正式回放序列
        try:
            prev_ts = None
            for frame_idx, frame in enumerate(sequence):
                if stop_signal and stop_signal.is_set():
                    return False

                angles = frame.get('angles', [])
                if len(angles) == 6:
                    for motor_id, angle in enumerate(angles, start=1):
                        self.safe_write(motor_id, angle)

                # 时间同步
                curr_ts = frame.get('timestamp', 0)
                if prev_ts is not None and curr_ts > prev_ts:
                    wait_time = curr_ts - prev_ts
                    time.sleep(min(wait_time, 0.1))

                prev_ts = curr_ts

            print("✅ 动作回放完成")
            return True

        except Exception as e:
            print(f"❌ 回放执行出错: {e}")
            return False

            # --- 利用录制时间戳计算真实间隔 ---
            cur_ts = frame.get('timestamp')
            if prev_ts is not None and cur_ts is not None:
                try:
                    if isinstance(cur_ts, str):
                        from datetime import datetime
                        dt_cur = datetime.fromisoformat(cur_ts).timestamp()
                        dt_pre = datetime.fromisoformat(prev_ts).timestamp()
                        frame_dt = max(0.02, min(dt_cur - dt_pre, 1.0))
                    else:
                        frame_dt = max(0.02, min(float(cur_ts) - float(prev_ts), 1.0))
                except Exception:
                    frame_dt = 0.1
            else:
                frame_dt = 0.1
            prev_ts = cur_ts

            # --- 多步平滑插值 ---
            if use_interpolation and prev_angles:
                INTERP_STEPS = 5  # 将 1 步增加到 5 步
                for step in range(1, INTERP_STEPS):
                    if stop_signal and stop_signal.is_set():
                        return False
                    ratio = step / INTERP_STEPS
                    interp = [int(p + (t - p) * ratio) for p, t in zip(prev_angles, angles)]
                    for i, pos in enumerate(interp):
                        motor_id = i + 1
                        if motor_id not in self.available_motors:
                            continue
                        safe_pos = self._apply_joint_limit(motor_id, pos)
                        try:
                            self._low_level_write(self.REGISTER_TARGET_POS, self.REGISTER_LENGTH, motor_id,
                                                  int(safe_pos))
                        except Exception:
                            continue
                    time.sleep(frame_dt / INTERP_STEPS)

            # --- 写入目标帧 ---
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
            time.sleep(0.02)  # 给电机一点稳定时间

            print("✅ 动作回放完成")
            return True

        except Exception as e:
            print(f"❌ 回放出错: {e}")
            return False


def write_angles(self, angles: List[int]) -> bool:
    if not self.connected or not self.sms:
        return False
    if len(angles) != 6:
        return False
    try:
        for motor_id, target_pos in enumerate(angles, start=1):
            if motor_id not in self.available_motors:
                continue
            target_pos = self._apply_joint_limit(motor_id, target_pos)
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
            safe_angles = [int(a) for a in angles]
            self.current_recording.append({
                'timestamp': time.time(),
                'angles': safe_angles
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
                        angles = frame.get('angles', [0] * 6)
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

    arm = SO101LeaderArm(port="/dev/leader_arm")
    recorder = GestureRecorder("test_gestures.json")

    if not arm.connect():
        print("❌ 无法连接主臂")
        sys.exit(1)

    for i in range(3):
        angles = arm.read_angles()
        if angles:
            print(f"  第 {i + 1} 次: {angles}")
            recorder.record_gesture(angles, label=f"sample_{i + 1}")
        time.sleep(0.5)

    arm.disconnect()
    recorder.export_csv("test_gestures.csv")
    print(f"✅ 动作库共 {recorder.get_action_count()} 个动作")
    print(f"✅ 已保存到 test_gestures.json 和 test_gestures.csv")
