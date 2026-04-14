#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot SO-101 硬件通信模块 (v2.0 - FeetechMotorsBus 直接集成)
用途：与主臂（Leader Arm）通过 COM12 串口进行通信
核心特性：
  - 使用 LeRobot FeetechMotorsBus 底层 _read/_write 接口（绕过 calibration 检查）
  - 读取 6 个关节实时位置（地址 56, 长度 2）
  - 写入目标位置用于回放演示（地址 42, 长度 2）
  - 完整的手势记录和回放支持
"""

import json
import time
import os
from datetime import datetime
from typing import Optional, List, Dict, Tuple
from pathlib import Path

# 【兼容性补丁】舵机底层 SDK 别名映射
# 不同的 LeRobot 版本可能使用不同的 SDK 包名
try:
    import scservo_sdk
except ImportError:
    try:
        import feetech_servo_sdk as scservo_sdk
    except ImportError:
        pass  # 如果两个都找不到，会在 _init_bus 中捕获


class MotorWrapper:
    """
    兼容 LeRobot FeetechMotorsBus 的电机包装类
    用于避免 calibration 检查导致的 KeyError
    """
    def __init__(self, motor_id: int, model: str = "sts3215"):
        self.id = motor_id
        self.model = model


class SO101LeaderArm:
    """SO-101 主臂（Leader Arm）硬件接口 - FeetechMotorsBus 直接集成"""
    
    # 寄存器地址（基于 Feetech STS3215 协议）
    REGISTER_CURRENT_POS = 56      # 当前位置地址
    REGISTER_TARGET_POS = 42       # 目标位置地址
    REGISTER_TORQUE_ENABLE = 40    # 力矩使能地址
    REGISTER_LENGTH = 2            # 2 字节
    TORQUE_ENABLE_LENGTH = 1       # 力矩使能是 1 字节
    POS_MIN = 0                    # 最小位置（0 度）
    POS_MAX = 4095                 # 最大位置（240 度）
    
    def __init__(self, port="COM12", baudrate=115200, timeout=1.0):
        """
        初始化主臂控制器
        
        Args:
            port: 串口号 (默认 COM12)
            baudrate: 波特率 (默认 115200)
            timeout: 串口超时 (秒)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.bus = None
        self.connected = False
        self.calib_path = self._get_calib_path()
        self._init_bus()
        
    def _get_calib_path(self) -> str:
        """获取校准文件路径"""
        home = Path.home()
        calib_dir = home / ".cache" / "huggingface" / "lerobot" / "calibration" / "teleoperators" / "so101_leader"
        calib_file = calib_dir / "None.json"
        return str(calib_file)
    
    def _init_bus(self):
        """初始化 FeetechMotorsBus 实例（支持多版本兼容）"""
        FeetechMotorsBus = None
        
        # 【兼容性补丁】多层级导入机制，自动适配不同 lerobot 版本
        try:
            # 路径 1: 适配 0.4.4 及之后的扁平化版本
            from lerobot.motors.feetech import FeetechMotorsBus
            print("✅ 导入模式 1: lerobot.motors.feetech (0.4.4+)")
        except ImportError:
            try:
                # 路径 2: 适配 0.3.x 版本
                from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
                print("✅ 导入模式 2: lerobot.common.robot_devices.motors.feetech (0.3.x)")
            except ImportError:
                try:
                    # 路径 3: 适配早期开发版
                    from lerobot.robot_devices.motors.feetech import FeetechMotorsBus
                    print("✅ 导入模式 3: lerobot.robot_devices.motors.feetech (early dev)")
                except ImportError:
                    print("⚠️  LeRobot 环境缺失或版本不兼容，使用降级模式")
                    self.bus = None
                    return
        
        try:
            # 创建电机包装器（ID 1-6 对应 6 个关节）
            motor_objects = {i: MotorWrapper(i) for i in range(1, 7)}
            
            # 初始化总线（不传 timeout 参数，以适配最新版库）
            self.bus = FeetechMotorsBus(
                port=self.port,
                motors=motor_objects
            )
        except Exception as e:
            print(f"⚠️  FeetechMotorsBus 初始化失败: {e}")
            self.bus = None
    
    def connect(self) -> bool:
        """连接到主臂（容忍单个舵机故障）"""
        try:
            if self.bus is None:
                print("⚠️  FeetechMotorsBus 未初始化，尝试重新初始化")
                self._init_bus()
            
            if self.bus:
                try:
                    self.bus.connect()
                except Exception as motor_check_error:
                    # 【容错处理】如果是舵机检测失败（常见于 6 号过载）
                    error_msg = str(motor_check_error)
                    if "motor check failed" in error_msg or "Missing motor IDs" in error_msg:
                        print(f"⚠️  舵机部分故障: {error_msg}")
                        print("⚠️  尝试继续连接（忽略故障舵机）...")
                        
                        # 强制设置连接状态，允许部分功能继续运行
                        self.connected = True
                        print(f"✅ 主臂已连接（降级模式，部分舵机可能不可用）")
                        return True
                    else:
                        raise  # 其他错误继续抛出
                
                self.connected = True
                print(f"✅ 主臂已连接 ({self.port})")
                return True
            else:
                print("❌ 无法初始化 FeetechMotorsBus")
                return False
        except Exception as e:
            print(f"❌ 无法连接主臂: {e}")
            print("💡 建议: 请检查 USB 连接、12V 电源，或重启程序")
            self.connected = False
            return False
    
    def disconnect(self) -> bool:
        """断开连接"""
        try:
            if self.bus and self.connected:
                self.bus.disconnect()
            self.connected = False
            print("👋 主臂连接已关闭")
            return True
        except Exception as e:
            print(f"⚠️  关闭连接时出错: {e}")
            return False
    
    def read_angles(self) -> Optional[List[int]]:
        """
        读取主臂的 6 个关节位置（原始编码值 0-4095）
        使用底层 _read 接口绕过 calibration 检查
        
        Returns:
            6 维数组 [pos1, pos2, ..., pos6] (单位: 编码值) 或 None (失败时)
        """
        if not self.connected or not self.bus:
            print("⚠️  主臂未连接")
            return None
        
        try:
            angles = []
            for motor_id in range(1, 7):
                # 使用底层 _read 接口：(address, length, motor_id)
                result = self.bus._read(
                    self.REGISTER_CURRENT_POS,
                    self.REGISTER_LENGTH,
                    motor_id
                )
                
                # 【关键修复】_read 返回 (value, error_code) 元组，必须严格解包
                # 直接 int(result) 会导致 "int() argument must be a string"崩溃
                if isinstance(result, (tuple, list)) and len(result) >= 1:
                    val = result[0]
                else:
                    val = result
                
                # 再次确保 val 是有效的数值
                if val is not None:
                    angles.append(int(val))
                else:
                    print(f"⚠️  电机 {motor_id} 读取失败")
                    return None
            
            return angles
        except Exception as e:
            print(f"⚠️  读取关节位置时出错: {e}")
            return None
    
    def safe_write(self, motor_id: int, target_pos: int) -> bool:
        """
        【新增】安全写入：带软限位保护的单舵机写入
        
        特别针对 6 号舵机（夹持器）的过载保护。基于实测数据，
        6 号舵机在 2954 时容易卡顿，故设置安全区间为 [1000, 2800]。
        
        Args:
            motor_id: 电机 ID (1-6)
            target_pos: 目标位置 (0-4095)
        
        Returns:
            成功返回 True
        """
        if not self.connected or not self.bus:
            return False
        
        try:
            # 【硬件保护】6 号舵机的专属限位，防止过载
            if motor_id == 6:
                target_pos = max(1000, min(target_pos, 2800))
                print(f"🛡️  6号舵机软限位: {target_pos} (安全区间: [1000, 2800])")
            else:
                target_pos = max(self.POS_MIN, min(target_pos, self.POS_MAX))
            
            # 启用力矩
            try:
                self.bus._write(self.REGISTER_TORQUE_ENABLE, self.TORQUE_ENABLE_LENGTH, motor_id, 1)
            except Exception:
                pass
            
            # 写入位置
            self.bus._write(self.REGISTER_TARGET_POS, self.REGISTER_LENGTH, motor_id, int(target_pos))
            return True
        except Exception as e:
            print(f"⚠️  舵机 {motor_id} 写入失败: {e}")
            return False
    
    def write_angles(self, angles: List[int]) -> bool:
        """
        ...existing code...
        """
        if not self.connected or not self.bus:
            print("⚠️  主臂未连接")
            return False
        
        if len(angles) != 6:
            print(f"❌ 角度数组长度不正确: 需要 6，收到 {len(angles)}")
            return False
        
        try:
            for motor_id, target_pos in enumerate(angles, start=1):
                # ...existing code...
                if not (self.POS_MIN <= target_pos <= self.POS_MAX):
                    print(f"⚠️  电机 {motor_id} 的目标位置 {target_pos} 超出范围 [{self.POS_MIN}, {self.POS_MAX}]")
                    target_pos = max(self.POS_MIN, min(self.POS_MAX, target_pos))
                
                # 【修复2】先启用力矩（Address 40, Length 1, Value 1）
                try:
                    self.bus._write(
                        self.REGISTER_TORQUE_ENABLE,
                        self.TORQUE_ENABLE_LENGTH,
                        motor_id,
                        1  # 力矩使能 = 1
                    )
                except Exception as e:
                    print(f"⚠️  电机 {motor_id} 力矩启用失败: {e}")
                
                # 再写入目标位置（Address 42, Length 2）
                try:
                    self.bus._write(
                        self.REGISTER_TARGET_POS,
                        self.REGISTER_LENGTH,
                        motor_id,
                        int(target_pos)
                    )
                except Exception as e:
                    print(f"❌ 电机 {motor_id} 位置写入失败: {e}")
                    return False
            
            return True
        except Exception as e:
            print(f"❌ 写入关节位置时出错: {e}")
            return False
    
    def get_status(self) -> Dict:
        """获取主臂状态（连接性检查）"""
        return {
            "port": self.port,
            "connected": self.connected,
            "timestamp": datetime.now().isoformat(),
            "calib_file": self.calib_path,
            "register_current_pos": self.REGISTER_CURRENT_POS,
            "register_target_pos": self.REGISTER_TARGET_POS
        }


class GestureRecorder:
    """手势/姿态记录器 - 支持原始编码值和回放"""
    
    def __init__(self, output_file="gestures.json"):
        """
        初始化记录器
        
        Args:
            output_file: 输出 JSON 文件路径
        """
        self.output_file = output_file
        self.gestures = []
        self._load_existing()
    
    def _load_existing(self):
        """加载现有的记录"""
        try:
            if os.path.exists(self.output_file):
                with open(self.output_file, 'r', encoding='utf-8') as f:
                    self.gestures = json.load(f)
        except Exception as e:
            print(f"⚠️  加载现有记录失败: {e}")
            self.gestures = []
    
    def record_gesture(self, angles: List[int], label: str = "", metadata: Optional[Dict] = None):
        """
        记录一个姿态
        
        Args:
            angles: 6 维关节位置数组 (编码值 0-4095)
            label: 标签（比如 "拿起球"、"放下球"）
            metadata: 可选的额外元数据
        """
        if len(angles) != 6:
            print(f"❌ 角度数组长度不正确: 需要 6，收到 {len(angles)}")
            return False
        
        record = {
            "timestamp": datetime.now().isoformat(),
            "angles": list(angles),  # 确保是列表
            "label": label,
            "metadata": metadata or {}
        }
        self.gestures.append(record)
        self._save()
        print(f"✅ 已记录姿态 (当前 {len(self.gestures)} 条): {label}")
        return True
    
    def _save(self):
        """保存到文件"""
        try:
            with open(self.output_file, 'w', encoding='utf-8') as f:
                json.dump(self.gestures, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"❌ 保存姿态失败: {e}")
    
    def get_count(self) -> int:
        """获取已记录的姿态数量"""
        return len(self.gestures)
    
    def get_gesture(self, index: int) -> Optional[Dict]:
        """获取指定索引的姿态"""
        if 0 <= index < len(self.gestures):
            return self.gestures[index]
        return None
    
    def export_csv(self, csv_file="gestures.csv"):
        """导出为 CSV 格式"""
        try:
            import csv
            with open(csv_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # 表头
                writer.writerow([
                    "timestamp", "angle_1", "angle_2", "angle_3", 
                    "angle_4", "angle_5", "angle_6", "label"
                ])
                # 数据
                for g in self.gestures:
                    angles = g.get('angles', [0]*6)
                    writer.writerow([
                        g['timestamp'],
                        *angles,
                        g.get('label', '')
                    ])
            print(f"✅ 已导出 CSV: {csv_file}")
            return True
        except Exception as e:
            print(f"❌ 导出 CSV 失败: {e}")
            return False


if __name__ == "__main__":
    """测试和演示脚本"""
    import sys
    
    print("""
╔════════════════════════════════════════════════════════════╗
║  LeRobot SO-101 硬件通信模块测试                            ║
╚════════════════════════════════════════════════════════════╝
    """)
    
    # 1. 初始化主臂和记录器
    arm = SO101LeaderArm(port="COM12")
    recorder = GestureRecorder("test_gestures.json")
    
    # 2. 尝试连接
    print("\n【连接测试】")
    if not arm.connect():
        print("❌ 无法连接主臂，请检查：")
        print("   1. USB 数据线是否已接入 COM12")
        print("   2. 12V 电源是否已打开")
        print("   3. LeRobot 环境是否已正确安装")
        sys.exit(1)
    
    # 3. 读取关节位置
    print("\n【关节位置读取】")
    for i in range(3):
        angles = arm.read_angles()
        if angles:
            print(f"  第 {i+1} 次: {angles}")
            recorder.record_gesture(angles, label=f"sample_{i+1}")
        else:
            print(f"  第 {i+1} 次: 读取失败")
        time.sleep(0.5)
    
    # 4. 关闭连接
    print("\n【关闭连接】")
    arm.disconnect()
    
    # 5. 导出数据
    print("\n【导出数据】")
    recorder.export_csv("test_gestures.csv")
    print(f"✅ 共记录 {recorder.get_count()} 个姿态")
    print(f"✅ 已保存到 test_gestures.json 和 test_gestures.csv")


