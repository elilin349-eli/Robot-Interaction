#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Voice-Robot: 语音控制机械臂系统 (LeRobot SO-101)
核心特性：
  - 配置驱动架构（commands.yaml）
  - iFlytek 多方言实时识别 + 动态修正
  - 模糊匹配（difflib）+ 鲁棒性增强
  - TTS 双向反馈 (pyttsx3)
  - 进程安全守护（atexit）
  - WebSocket 稳定重连 + PCM 音频处理
"""

import os, sys, time, ssl, hmac, base64, json, queue, threading, signal, hashlib, atexit, difflib, subprocess, re
import numpy as np
from datetime import datetime
from time import mktime
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from typing import Dict, List, Optional, Tuple

# ==========================================
# 依赖导入
# ==========================================
try:
    import sounddevice as sd
    import websocket
except ImportError as e:
    print(f"❌ 缺失依赖: {e}\n请运行: pip install sounddevice websocket-client")
    sys.exit(1)

# Whisper 本地语音识别（备用方案）
try:
    import whisper

    WHISPER_AVAILABLE = True
    print("✅ Whisper 库已加载（可用于本地语音识别）")
except ImportError:
    WHISPER_AVAILABLE = False
    print("⚠️  Whisper 库缺失。运行: pip install openai-whisper")

# 导入 LeRobot 硬件模块
try:
    from lerobot_hardware import SO101LeaderArm, GestureRecorder

    LEROBOT_AVAILABLE = True
except ImportError:
    print("⚠️  LeRobot 硬件模块缺失，主臂控制不可用")
    LEROBOT_AVAILABLE = False

try:
    import yaml
except ImportError:
    print("⚠️  yaml 库缺失，使用备用 JSON 加载器")
    yaml = None

try:
    import pyttsx3

    TTS_AVAILABLE = True
except ImportError:
    print("⚠️  pyttsx3 库缺失，TTS 反馈不可用。运行: pip install pyttsx3")
    TTS_AVAILABLE = False


# ==========================================
# 配置加载器（多层级：yaml > .env > 默认值）
# ==========================================
class ConfigLoader:
    """从 yaml/json/.env 加载配置的通用加载器"""

    def __init__(self, config_file='commands.yaml', env_file='.env'):
        self.config = {}
        self.load_config_file(config_file)
        self.load_env_file(env_file)

    def load_config_file(self, filename):
        """加载 YAML 配置文件"""
        fpath = os.path.join(os.path.dirname(__file__), filename)
        if not os.path.exists(fpath):
            print(f"⚠️  配置文件不存在: {fpath}，使用默认值")
            return

        try:
            if yaml:
                with open(fpath, 'r', encoding='utf-8') as f:
                    self.config = yaml.safe_load(f) or {}
            else:
                # 备用：简单的 JSON 解析（如果 YAML 不可用）
                with open(fpath, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if content.startswith('{'):
                        self.config = json.loads(content)
        except Exception as e:
            print(f"❌ 加载配置文件失败: {e}")

    def load_env_file(self, filename):
        """从 .env 覆盖配置"""
        fpath = os.path.join(os.path.dirname(__file__), filename)
        if not os.path.exists(fpath):
            return

        try:
            with open(fpath, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#') or '=' not in line:
                        continue
                    k, v = line.split('=', 1)
                    k, v = k.strip(), v.strip().strip('"').strip("'")
                    os.environ[k] = v
        except Exception:
            pass

    def get(self, key, default=None):
        """获取配置值（支持嵌套 key: "commands.PICK.keywords"）"""
        keys = key.split('.')
        val = self.config
        for k in keys:
            if isinstance(val, dict):
                val = val.get(k)
            else:
                return default
        return val if val is not None else default


# 加载配置
CONFIG = ConfigLoader()

APP_ID = os.environ.get('IFLY_APP_ID', CONFIG.get('defaults.app_id', 'REMOVED'))
API_KEY = os.environ.get('IFLY_API_KEY', CONFIG.get('defaults.api_key', 'REMOVED'))
API_SECRET = os.environ.get('IFLY_API_SECRET', CONFIG.get('defaults.api_secret', 'MWNlODE0YzRhODEyYTFjNmMwNTVkZmFh'))

ACCENT = os.environ.get('IFLY_ACCENT', CONFIG.get('speech.accent', 'mandarin'))
RATE = CONFIG.get('speech.sample_rate', 16000)
CHANNELS = CONFIG.get('speech.channels', 2)
CHUNK_MS = CONFIG.get('speech.chunk_ms', 80)
CHUNK = int(RATE * CHUNK_MS / 1000)
FUZZY_THRESHOLD = CONFIG.get('speech.fuzzy_match_threshold', 0.75)

# 全局控制
EXIT_EVENT = threading.Event()
AUDIO_QUEUE = queue.Queue(maxsize=150)
ROBOT_QUEUE = queue.Queue()
LATEST_RMS = 0.0
RMS_LOCK = threading.Lock()
LAST_TEXT = ""
# 【P0修复 问题2】全局停止信号，用于中断回放
STOP_SIGNAL = threading.Event()

# VAD 阈值
START_THRESHOLD = 6.0
END_THRESHOLD = 3.0
CALIBRATE_SECONDS = 1.5

# 日志输出
LOG_FILE = os.path.join(os.path.dirname(__file__), CONFIG.get('logging.log_file', 'commands.log'))
CMD_FILE = os.path.join(os.path.dirname(__file__), CONFIG.get('logging.cmd_json', 'last_command.json'))


# ==========================================
# TTS 语音反馈引擎
# ==========================================
class TTSEngine:
    """文字转语音（中文）"""

    def __init__(self):
        self.enabled = TTS_AVAILABLE
        self._queue = queue.Queue()
        if TTS_AVAILABLE:
            try:
                self.engine = pyttsx3.init()
                self.engine.setProperty('rate', 160)
                self.engine.setProperty('volume', 0.8)
                threading.Thread(target=self._worker, daemon=True).start()
                atexit.register(self.stop)
            except Exception as e:
                print(f"⚠️  TTS 初始化失败: {e}")
                self.enabled = False

    def _worker(self):
        while True:
            text = self._queue.get()
            if text is None:
                break
            try:
                self.engine.say(text)
                self.engine.runAndWait()
            except Exception as e:
                print(f"⚠️ TTS 播放异常: {e}")

    def stop(self):
        """显式停止引擎"""
        if self.enabled and self.engine:
            try:
                self._queue.put(None)
                self.engine.stop()
            except Exception:
                pass

    def speak(self, text):
        """异步播报"""
        if not self.enabled or not text:
            return
        self._queue.put(text)


tts = TTSEngine()


# ==========================================
# 命令识别引擎（配置驱动 + 模糊匹配）
# ==========================================
class CommandMatcher:
    """配置驱动的指令匹配器，支持模糊匹配"""

    def __init__(self, config: ConfigLoader):
        self.config = config
        self.commands = config.get('commands', {})
        self.fuzzy_threshold = config.get('speech.fuzzy_match_threshold', 0.75)

    def match(self, text: str) -> Tuple[Optional[str], float]:
        if not text:
            return None, 0.0

        # 精确匹配时优先选择最长关键词，避免短词截胡。
        best_exact_action = None
        best_exact_len = 0
        for action_name, action_config in self.commands.items():
            if not isinstance(action_config, dict):
                continue
            for kw in action_config.get('keywords', []):
                if kw in text and len(kw) > best_exact_len:
                    best_exact_len = len(kw)
                    best_exact_action = action_name

        if best_exact_action:
            return best_exact_action, 1.0

        best_action = None
        best_score = 0.0

        for action_name, action_config in self.commands.items():
            if not isinstance(action_config, dict):
                continue

            for kw in action_config.get('keywords', []):
                score = difflib.SequenceMatcher(None, text, kw).ratio()
                if score > best_score:
                    best_score = score
                    best_action = action_name
        if best_score >= self.fuzzy_threshold:
            return best_action, best_score
        return None, best_score

    def get_tts_feedback(self, action_name: str) -> str:
        """获取指令对应的 TTS 反馈文本"""
        action = self.commands.get(action_name, {})
        return action.get('tts_feedback', '')


# 全局指令匹配器
matcher = CommandMatcher(CONFIG)


# ==========================================
# Whisper 本地语音识别（单例）
# ==========================================
class WhisperEngine:
    """Whisper 本地语音识别引擎"""
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        self.model = None
        self.enabled = False
        if WHISPER_AVAILABLE:
            self._load_model()

    def _load_model(self):
        """加载 Whisper 模型（首次加载会下载 ~140MB）"""
        try:
            print("\n🔄 加载 Whisper 本地语音识别模型（首次约需 30s）...")
            self.model = whisper.load_model("base")
            self.enabled = True
            print("✅ Whisper 模型已加载")
        except Exception as e:
            print(f"⚠️  Whisper 模型加载失败: {e}")
            self.enabled = False

    def transcribe(self, audio_file: str) -> Optional[str]:
        """
        转写音频文件（使用 Whisper）
        
        Args:
            audio_file: WAV 文件路径
        
        Returns:
            转写的中文文本，失败时返回 None
        """
        if not self.enabled or not self.model:
            return None

        try:
            result = self.model.transcribe(audio_file, language="zh")
            return result.get("text", "").strip()
        except Exception as e:
            print(f"⚠️  Whisper 转写失败: {e}")
            return None


whisper_engine = WhisperEngine()


# ==========================================
# 日志工具
# ==========================================
def log_cmd(text, cmd_label, score=None):
    """写入命令日志"""
    try:
        with open(LOG_FILE, 'a', encoding='utf-8') as f:
            json.dump({
                'ts': datetime.now().isoformat(),
                'text': text,
                'cmd': cmd_label,
                'score': score
            }, f, ensure_ascii=False)
            f.write('\n')
    except Exception:
        pass


def save_cmd_json(action, text, score=None):
    """保存当前指令到 JSON（外部程序可读）"""
    try:
        with open(CMD_FILE, 'w', encoding='utf-8') as f:
            json.dump({
                'ts': datetime.now().isoformat(),
                'action': action,
                'text': text,
                'score': score
            }, f, ensure_ascii=False)
    except Exception:
        pass


# ==========================================
# 进程安全管理（LeRobot 子进程保护）
# ==========================================
class ProcessManager:
    """管理 LeRobot 子进程的生命周期以及主臂/从臂硬件"""

    def __init__(self):
        self.process = None
        self.lock = threading.Lock()
        self.waiting_for_naming = False
        self.pending_record_path = None
        self._record_snapshot = set()

        # 主臂硬件接口
        self.leader_arm = None
        self.gesture_recorder = None
        self.recording_gestures = False
        
        # 从臂硬件接口（用于协同演示）
        self.follower_arm = None

        # --- Whisper 补位方案：预加载模型 ---
        self.whisper_model = None
        if WHISPER_AVAILABLE:
            print("🚀 正在预加载 Whisper base 模型...")
            try:
                self.whisper_model = whisper.load_model("base")
                print("✅ Whisper 模型加载完成")
            except Exception as e:
                print(f"⚠️  Whisper 预加载失败: {e}")

        self.is_active = False
        self.active_timer = None
        self.WAKE_WORDS = ["机器人", "开始指令", "小臂小臂", "小臂", "激活"]
        self.ACTIVE_DURATION = 20.0
        self.operation_mode = None

        # 注册清理钩子，程序退出时强制杀死子进程
        atexit.register(self.cleanup)

        # 初始化主臂和从臂（如果可用）
        if LEROBOT_AVAILABLE:
            self._init_leader_arm()
            self._init_follower_arm()

    def start_recording(self, env_name='lerobot', robot_path='lerobot/configs/robot/so_101'):
        """启动 LeRobot 数据采集"""
        with self.lock:
            if self.process is not None:
                print("⚠️  LeRobot 进程已在运行")
                return False

            self.waiting_for_naming = False
            self.pending_record_path = None
            self._record_snapshot = self._scan_record_artifacts()

            script_path = CONFIG.get('lerobot.script_path', 'lerobot/scripts/control_robot.py')
            cmd = f"conda run -n {env_name} python {script_path} teleoperate --robot-path {robot_path} --record"

            try:
                # 日志重定向到文件，防止管道缓冲区溢出导致进程卡死
                log_file = open(CONFIG.get('logging.lerobot_log', 'lerobot_runtime.log'), 'a', encoding='utf-8')
                self.process = subprocess.Popen(
                    cmd,
                    shell=True,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    text=True,
                    creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == 'nt' else 0
                )
                print(f"📊 LeRobot 采集已启动 (PID: {self.process.pid})")
                tts.speak("开始记录遥操作数据")
                return True
            except Exception as e:
                print(f"❌ 启动 LeRobot 失败: {e}")
                return False

    def stop_recording(self):
        """停止 LeRobot 数据采集"""
        with self.lock:
            if self.process is None:
                return True

            try:
                if os.name == 'nt':  # Windows
                    subprocess.run(f"taskkill /F /T /PID {self.process.pid}", shell=True, timeout=5)
                else:  # Linux/Mac
                    self.process.terminate()
                    self.process.wait(timeout=5)
                print("✅ LeRobot 采集已停止")
                tts.speak("停止记录，数据已保存")
                self.process = None
                self.pending_record_path = self._find_new_record_artifact(self._record_snapshot)
                self.waiting_for_naming = True
                return True
            except Exception as e:
                print(f"⚠️  停止 LeRobot 时出错: {e}")
                self.process = None
                return False

    def _scan_record_artifacts(self):
        roots = ["data", "datasets", "outputs", "recordings"]
        found = set()
        for root in roots:
            abs_root = os.path.join(os.getcwd(), root)
            if not os.path.exists(abs_root):
                continue
            try:
                for name in os.listdir(abs_root):
                    found.add(os.path.join(abs_root, name))
            except Exception:
                continue
        return found

    def _find_new_record_artifact(self, before_set):
        candidates = list(self._scan_record_artifacts() - (before_set or set()))
        if not candidates:
            return None
        candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return candidates[0]

    def finalize_record_naming(self, new_name: str) -> bool:
        if not new_name:
            return False
        if not self.pending_record_path or not os.path.exists(self.pending_record_path):
            return False

        parent = os.path.dirname(self.pending_record_path)
        ext = os.path.splitext(self.pending_record_path)[1]
        target = os.path.join(parent, f"{new_name}{ext}")
        if target == self.pending_record_path:
            self.waiting_for_naming = False
            return True

        os.rename(self.pending_record_path, target)
        self.pending_record_path = target
        self.waiting_for_naming = False
        return True

    def _init_leader_arm(self):
        """初始化主臂硬件接口"""
        try:
            port = CONFIG.get('hardware.leader_port', 'COM12')
            self.leader_arm = SO101LeaderArm(port=port)
            self.gesture_recorder = GestureRecorder("gestures_library.json")

            # 【关键】立即尝试连接硬件
            # 这样可以在程序启动时就发现问题，而不是等到指令时才发现
            print(f"📦 主臂模块已初始化 (端口: {port})")
            print("🔗 正在建立硬件连接...")

            if not self.leader_arm.connect():
                print("⚠️  硬件连接失败，请检查：")
                print("   1. SO-101 机械臂是否接入 COM12")
                print("   2. 12V 电源是否已打开")
                print("   3. USB 数据线连接是否正常")
                print("   进程将继续运行，但硬件指令会被忽略")
            else:
                print("✅ 硬件连接已建立")

        except Exception as e:
            print(f"⚠️  主臂初始化失败: {e}")
    
    def _init_follower_arm(self):
        """初始化从臂硬件接口（用于协同演示）"""
        try:
            port = CONFIG.get('hardware.follower_port', 'COM11')
            self.follower_arm = SO101LeaderArm(port=port)
            print(f"📦 从臂模块已初始化 (端口: {port})")
            
            if not self.follower_arm.connect():
                # 【P2修复 问题6】使用动态变量替换硬编码端口
                print(f"⚠️  从臂连接失败，请检查 {port} 端口")
                self.follower_arm = None
            else:
                print("✅ 从臂连接已建立")
        except Exception as e:
            print(f"⚠️  从臂初始化失败: {e}")
            self.follower_arm = None
    
    def activate(self, reason="语音唤醒"):
        """激活指令监听窗口"""
        self.is_active = True

        if self.operation_mode == 'teaching':
            if self.active_timer:
                self.active_timer.cancel()
                self.active_timer = None
            print(f"🟢 [系统激活] 原因: {reason} | 状态：示教模式锁定")
            return

        if self.active_timer:
            self.active_timer.cancel()

        self.active_timer = threading.Timer(self.ACTIVE_DURATION, self.deactivate)
        self.active_timer.start()

        print(f"🟢 [系统激活] 原因: {reason} | 监听窗口开启 {self.ACTIVE_DURATION}s")
        if reason == "语音唤醒":
            tts.speak("我在，请吩咐")

    def deactivate(self):
        """进入待机状态"""
        if self.operation_mode == 'teaching':
            return
        self.is_active = False
        print("🔴 [系统待机] 监听窗口已关闭，等待唤醒...")
    
    def execute_action(self, action_name: str) -> bool:
        """
        线程安全地在从臂执行命名动作
        【深水区优化】锁粒度精简：仅锁住内存读取，将耗时的 safe_playback 移出锁块
        
        Args:
            action_name: 动作名称（必须在 gesture_recorder.library 中存在）
        
        Returns:
            成功返回 True
        """
        # 【优化】第一阶段：在锁保护下仅进行快速检查和数据读取
        with self.lock:
            if not self.gesture_recorder or action_name not in self.gesture_recorder.library:
                print(f"⚠️  动作库中不存在: {action_name}")
                tts.speak(f"动作库里没有{action_name}这个名字")
                return False
            
            if not self.follower_arm or not self.follower_arm.connected:
                print(f"⚠️  从臂未连接，无法回放")
                tts.speak("从臂未连接")
                return False
            
            # 仅在锁内读取一份数据副本，然后立即释放锁
            data = self.gesture_recorder.library[action_name]
        
        # 【优化】第二阶段：在锁外进行长耗时的硬件回放
        # 这样 STOP 指令可以不被阻塞地进入
        try:
            if self.follower_arm.safe_playback(data, stop_signal=STOP_SIGNAL):
                tts.speak(f"已在从臂上执行{action_name}")
                return True
            else:
                tts.speak(f"回放{action_name}失败")
                return False
        except Exception as e:
            print(f"❌ 执行动作失败: {e}")
            tts.speak("执行动作出错")
            return False

    def record_gesture_from_leader(self, label: str = ""):
        """从主臂读取当前姿态并记录"""
        if not LEROBOT_AVAILABLE or not self.leader_arm:
            print("⚠️  主臂硬件不可用")
            return False

        try:
            if not self.leader_arm.connected:
                if not self.leader_arm.connect():
                    return False

            angles = self.leader_arm.read_angles()
            if angles is None:
                print("⚠️  无法读取关节角度")
                return False

            # 记录姿态
            self.gesture_recorder.record_gesture(angles, label)
            tts.speak(f"已记录姿态：{label}")
            return True
        except Exception as e:
            print(f"❌ 记录姿态失败: {e}")
            return False

    def start_gesture_recording(self):
        """开始连续记录主臂姿态"""
        if self.recording_gestures:
            print("⚠️ 已在录制中，跳过重复启动")
            return False

        if not LEROBOT_AVAILABLE or not self.leader_arm:
            print("⚠️  主臂硬件不可用")
            return False

        self.recording_gestures = True
        print("🎬 开始连续记录主臂姿态（后台线程）")
        tts.speak("开始记录主臂姿态")

        def record_loop():
            interval = CONFIG.get('hardware.record_interval_ms', 100) / 1000.0
            frame_count = 0
            MOVE_THRESHOLD = 12
            while self.recording_gestures and not EXIT_EVENT.is_set():
                try:
                    if not self.leader_arm.connected:
                        if not self.leader_arm.connect():
                            time.sleep(1)
                            continue

                    angles = self.leader_arm.read_angles()
                    if angles:
                        if self.gesture_recorder.current_recording:
                            last_angles = self.gesture_recorder.current_recording[-1]['angles']
                            total_diff = sum(abs(a - b) for a, b in zip(angles, last_angles))
                            if total_diff < MOVE_THRESHOLD:
                                time.sleep(interval)
                                continue
                        
                        self.gesture_recorder.record_gesture(angles, f"frame_{frame_count}")
                        frame_count += 1

                    time.sleep(interval)
                except Exception as e:
                    print(f"⚠️  记录循环异常: {e}")
                    time.sleep(interval)

        thread = threading.Thread(target=record_loop, daemon=True)
        thread.start()
        return True

    def stop_gesture_recording(self):
        """停止连续记录主臂姿态"""
        self.recording_gestures = False
        print("⏹️  已停止记录主臂姿态")
        tts.speak(f"停止记录，共记录 {self.gesture_recorder.get_count()} 个姿态")
        return True

    def cleanup(self):
        """清理钩子：程序退出时调用
        【P0修复 问题4】增强退出逻辑，确保回放被中断，减少串口僵死风险"""
        # 【关键】立即发送全局停止信号，中断任何正在进行的回放
        STOP_SIGNAL.set()
        
        self.stop_recording()
        if self.gesture_recorder:
            print(f"📊 共记录 {self.gesture_recorder.get_count()} 个姿态")

        # 【舵机过载保护】尝试释放所有舵机的力矩，即使某个舵机已过载也不中断清理流程
        if self.leader_arm and self.leader_arm.connected:
            try:
                # 使用统一的 set_torque 方法释放所有舵机
                self.leader_arm.set_torque(False)
                print("✅ 主臂所有舵机已释放")
            except Exception as e:
                print(f"⚠️  清理舵机力矩时出错: {e}")

            # 最后断开连接
            try:
                self.leader_arm.disconnect()
            except Exception as e:
                print(f"⚠️  断开连接时出错: {e}")


pm = ProcessManager()


# ==========================================
# 机器人指令处理器
# ==========================================
def keyboard_listener():
    """手动按键激活（汇报演示防尴尬神器）"""
    try:
        import keyboard
        print("💡 提示：演示中若噪音太大，可按 [Space] 强制激活监听")
        while not EXIT_EVENT.is_set():
            if keyboard.is_pressed('space'):
                pm.activate(reason="手动按键")
                time.sleep(1)  # 防抖
            time.sleep(0.1)
    except ImportError:
        pass


def robot_worker():
    """
    【关键修复】处理机器人指令队列
    
    核心特性：
    - 独立线程运行，不受 WebSocket 超时影响
    - 即使语音连接断开也持续工作
    - 包含异常捕获，防止线程意外退出
    """
    print("🤖 robot_worker 已启动（后台持续监听指令）")

    error_count = 0
    while not EXIT_EVENT.is_set():
        try:
            # 非阻塞读队列，避免卡死
            cmd = ROBOT_QUEUE.get(timeout=0.5)
        except queue.Empty:
            # 定期检查 EXIT_EVENT，但不打印日志避免噪音
            continue
        except Exception as e:
            error_count += 1
            if error_count > 10:
                print(f"⚠️  robot_worker 异常多次: {e}")
                error_count = 0
            continue

        try:
            action = cmd.get('action')
            text = cmd.get('text', '')
            score = cmd.get('score', 0.0)

            print(f"▶️  执行: {action} | '{text}' (置信度: {score:.2f})")
            save_cmd_json(action, text, score)
            log_cmd(text, action, score)

            # 【关键】根据动作类型分发处理
            if action == 'RECORD_START':
                pm.start_recording()
            elif action == 'RECORD_STOP':
                pm.stop_recording()
                pm.operation_mode = None
                pm.activate(reason="采集结束")
                pm.waiting_for_naming = True
                tts.speak("录制已停止，请说命名为加上名字来保存动作")
            elif action == 'GESTURE_RECORD':
                pm.record_gesture_from_leader(label=text)
            elif action == 'GESTURE_START':
                pm.start_gesture_recording()
            elif action == 'GESTURE_STOP':
                pm.stop_gesture_recording()
                pm.operation_mode = None
                pm.activate(reason="录制结束")
                pm.waiting_for_naming = True
                tts.speak("录制已停止并保存。现在请说命名为加上名字，例如命名为挥手，来给动作起名。")

            elif action == '__PLAYBACK__':
                target = (cmd.get('target') or '').strip()
                if not target:
                    tts.speak("没有听清要执行的动作名字")
                    continue

                if not pm.gesture_recorder or not pm.gesture_recorder.library:
                    tts.speak("动作库为空，请先录制并命名")
                    continue

                names = list(pm.gesture_recorder.library.keys())
                if target in pm.gesture_recorder.library:
                    selected = target
                else:
                    matched = difflib.get_close_matches(target, names, n=1, cutoff=0.6)
                    if not matched:
                        tts.speak("动作库里没有这个名字")
                        continue
                    selected = matched[0]

                tts.speak(f"正在执行{selected}")
                threading.Thread(target=pm.execute_action, args=(selected,), daemon=True).start()

            # 【新增】机械臂实时控制逻辑（使用 safe_write 保护）
            elif action == 'PICK':
                try:
                    print("🦾 执行实时抓取动作 (6 号舵机)...")
                    threading.Thread(target=lambda: tts.speak("收到抓取指令"), daemon=True).start()
                    pick_pos = int(CONFIG.get('hardware.gripper_pick_pos', 2600))

                    if pm.leader_arm and pm.leader_arm.connected:
                        current_angles = pm.leader_arm.read_angles()
                        if current_angles:
                            print(f"🔍 调试: 6号舵机当前位置 {current_angles[5]} (范围: 1000-2800)")

                        pm.leader_arm.safe_write(6, pick_pos)
                        print("✅ 抓取动作已执行")
                    else:
                        print("⚠️  主臂未连接，无法执行抓取")
                        threading.Thread(target=lambda: tts.speak("主臂未连接"), daemon=True).start()

                except Exception as e:
                    print(f"⚠️  抓取动作执行失败: {e}")
                    threading.Thread(target=lambda: tts.speak("操作执行遇到异常"), daemon=True).start()

            elif action == 'PLACE':
                try:
                    print("🦾 执行实时放下动作 (6 号舵机)...")
                    threading.Thread(target=lambda: tts.speak("收到放下指令"), daemon=True).start()
                    place_pos = int(CONFIG.get('hardware.gripper_place_pos', 1600))

                    if pm.leader_arm and pm.leader_arm.connected:
                        current_angles = pm.leader_arm.read_angles()
                        if current_angles:
                            print(f"🔍 调试: 6号舵机当前位置 {current_angles[5]} (范围: 1000-2800)")

                        pm.leader_arm.safe_write(6, place_pos)
                        print("✅ 放下动作已执行")
                    else:
                        print("⚠️  主臂未连接，无法执行放下")
                        threading.Thread(target=lambda: tts.speak("主臂未连接"), daemon=True).start()

                except Exception as e:
                    print(f"⚠️  放下动作执行失败: {e}")
                    threading.Thread(target=lambda: tts.speak("操作执行遇到异常"), daemon=True).start()

            elif action == 'FOLLOWER_PICK':
                try:
                    pick_pos = int(CONFIG.get('hardware.follower_gripper_pick_pos', CONFIG.get('hardware.gripper_pick_pos', 2600)))
                    if pm.follower_arm is None:
                        print("❌ 从臂对象未初始化")
                        tts.speak("从臂硬件未初始化")
                    elif 6 not in getattr(pm.follower_arm, 'available_motors', [1, 2, 3, 4, 5, 6]):
                        print("⚠️ 从臂 6 号舵机缺失，无法抓取")
                        tts.speak("检测到从臂夹爪掉线，无法执行")
                    elif not pm.follower_arm.connected:
                        print(f"⚠️ 从臂未连接 ({pm.follower_arm.port})，尝试重连...")
                        if pm.follower_arm.connect():
                            print("✅ 从臂重连成功")
                            pm.follower_arm.safe_write(6, pick_pos)
                            print("✅ 从臂执行：抓取")
                        else:
                            tts.speak("从臂连接失败")
                    else:
                        pm.follower_arm.safe_write(6, pick_pos)
                        print("✅ 从臂执行：抓取")
                except Exception as e:
                    print(f"⚠️ 从臂抓取异常: {e}")
                    tts.speak("从臂操作异常")

            elif action == 'FOLLOWER_PLACE':
                try:
                    place_pos = int(CONFIG.get('hardware.follower_gripper_place_pos', CONFIG.get('hardware.gripper_place_pos', 1600)))
                    if pm.follower_arm is None:
                        print("❌ 从臂对象未初始化")
                        tts.speak("从臂硬件未初始化")
                    elif 6 not in getattr(pm.follower_arm, 'available_motors', [1, 2, 3, 4, 5, 6]):
                        print("⚠️ 从臂 6 号舵机缺失，无法放下")
                        tts.speak("检测到从臂夹爪掉线，无法执行")
                    elif not pm.follower_arm.connected:
                        print(f"⚠️ 从臂未连接 ({pm.follower_arm.port})，尝试重连...")
                        if pm.follower_arm.connect():
                            print("✅ 从臂重连成功")
                            pm.follower_arm.safe_write(6, place_pos)
                            print("✅ 从臂执行：放下")
                        else:
                            tts.speak("从臂连接失败")
                    else:
                        pm.follower_arm.safe_write(6, place_pos)
                        print("✅ 从臂执行：放下")
                except Exception as e:
                    print(f"⚠️ 从臂放下异常: {e}")
                    tts.speak("从臂操作异常")

            elif action == 'JOINT_CONTROL':
                """
                【多关节独立语音控制】
                支持以下语义：
                - "一号关节复位" -> 1号舵机回到中位(2048)
                - "六号舵机随机" -> 6号舵机移动到随机位置(1200-2800)
                - "三号关节转一点" -> 3号舵机增加 400 编码值
                - "四号电机反向" -> 4号舵机减少 400 编码值
                """
                try:
                    # 1. 提取舵机ID（正则表达式）
                    id_match = re.search(r'(\d+)\s*号', text)
                    joint_id = int(id_match.group(1)) if id_match else None

                    if not joint_id or not (1 <= joint_id <= 6):
                        tts.speak("没听清是几号关节，请重新说")
                        continue

                    print(f"🎯 提取到关节ID: {joint_id}")
                    threading.Thread(target=lambda: tts.speak(f"正在调整{joint_id}号关节"), daemon=True).start()

                    if not pm.leader_arm or not pm.leader_arm.connected:
                        print("⚠️  主臂未连接")
                        continue

                    # 2. 读取当前位置（用于增量动作）
                    current_poses = pm.leader_arm.read_angles()
                    if not current_poses:
                        print("⚠️  无法读取关节位置")
                        continue

                    current_pos = current_poses[joint_id - 1]
                    print(f"📍 当前位置: {current_pos}")

                    # 3. 语义判断，决定目标位置
                    if "随机" in text or "乱动" in text:
                        target = np.random.randint(1200, 2800)
                        print(f"🎲 随机移动至: {target}")
                    elif "复位" in text or "回到" in text or "中间" in text:
                        target = 2048
                        print(f"🔄 复位至中位: {target}")
                    elif "转一点" in text or "增加" in text or "往前" in text:
                        target = current_pos + 400
                        print(f"➡️  增加值至: {target}")
                    elif "反向" in text or "减少" in text or "往后" in text:
                        target = current_pos - 400
                        print(f"⬅️  减少值至: {target}")
                    elif "打开" in text and joint_id == 6:
                        target = 1000
                        print(f"📂 夹持器打开至: {target}")
                    elif "闭合" in text and joint_id == 6:
                        target = 2800
                        print(f"📁 夹持器闭合至: {target}")
                    else:
                        # 默认中位
                        target = 2048
                        print(f"❓ 指令不明确，默认回到中位: {target}")

                    # 4. STS3215 安全限位过滤（关键！防止过载）
                    final_target = max(1000, min(target, 3000))
                    if final_target != target:
                        print(f"⚠️  目标值 {target} 超出安全范围，已裁剪至 {final_target}")

                    # 5. 执行动作
                    pm.leader_arm.safe_write(joint_id, final_target)
                    print(f"✅ {joint_id}号舵机已移动至 {final_target}")

                except Exception as e:
                    print(f"⚠️  多关节控制失败: {e}")
                    threading.Thread(target=lambda: tts.speak("关节控制异常"), daemon=True).start()

            elif action == 'ARM_RELAX':
                try:
                    pm.operation_mode = 'teaching'
                    pm.activate(reason="进入示教模式")

                    if pm.process is not None:
                        pm.stop_recording()
                        time.sleep(0.5)

                    if not (pm.leader_arm and pm.leader_arm.connected):
                        pm.operation_mode = None
                        tts.speak("主臂未连接")
                        continue

                    tts.speak("已进入示教模式，主臂已变软。请用手拖动主臂完成你想要的动作，完成后说停止录制，系统会提示你命名。")

                    def safe_torque_off():
                        try:
                            pm.leader_arm.set_torque(False)
                            pm.start_gesture_recording()
                        except Exception as hw_error:
                            print(f"⚠️ 硬件告警: {hw_error}")

                    threading.Thread(target=safe_torque_off, daemon=True).start()

                except Exception as e:
                    pm.operation_mode = None
                    print(f"⚠️ 示教启动异常: {e}")

            elif action == 'ARM_LOCK':
                """退出示教模式：锁定所有舵机力矩"""
                try:
                    pm.operation_mode = None
                    pm.activate(reason="退出示教模式")
                    print("🔒 锁定所有舵机...")
                    threading.Thread(target=lambda: tts.speak("已锁定机械臂"), daemon=True).start()

                    if pm.leader_arm and pm.leader_arm.connected:
                        pm.leader_arm.set_torque(True)
                        print("✅ 所有舵机已锁定")
                    else:
                        print("⚠️  主臂未连接")

                except Exception as e:
                    print(f"⚠️  锁定舵机失败: {e}")

            elif action == 'STOP':
                """【深水区优化】增强型紧急停止逻辑
                1. 全局广播: 立即触发 STOP_SIGNAL
                2. 双臂释放: leader_arm + follower_arm 所有舵机
                3. 时序同步: 缓冲时间确保回放线程捕捉到中断信号
                4. 复位: STOP_SIGNAL.clear() 为下一轮准备"""
                try:
                    print("🛑 紧急停止：全局广播中断信号...")
                    # 【第一步】立即发送全局停止信号
                    STOP_SIGNAL.set()
                    threading.Thread(target=lambda: tts.speak("已紧急停止"), daemon=True).start()

                    # 【第二步】释放主臂所有舵机 (1-6)
                    if pm.leader_arm and pm.leader_arm.connected:
                        print("📍 释放主臂舵机力矩...")
                        pm.leader_arm.set_torque(False)
                        print("✅ 主臂舵机已释放")
                    
                    # 【第三步】释放从臂所有舵机 (1-6) - 新增支持
                    if pm.follower_arm and pm.follower_arm.connected:
                        print("📍 释放从臂舵机力矩...")
                        pm.follower_arm.set_torque(False)
                        print("✅ 从臂舵机已释放")
                    
                    # 【第四步】时序同步：给回放线程足够窗口期捕捉中断信号
                    # 原理：safe_playback 每帧 40ms，0.15s 缓冲可覆盖 3-4 帧采样
                    print("⏱️  同步等待回放线程退出...")
                    time.sleep(0.15)
                    
                    # 【第五步】清除停止信号，为下一轮做准备
                    STOP_SIGNAL.clear()
                    print("✅ 系统已完全停止并复位")

                except Exception as e:
                    print(f"⚠️  释放舵机失败: {e}")
                    threading.Thread(target=lambda: tts.speak("释放操作失败"), daemon=True).start()
                    # 即使异常也尝试清除信号
                    try:
                        STOP_SIGNAL.clear()
                    except:
                        pass
            
            elif "命名为" in text:
                try:
                    name = text.split("命名为")[-1].strip().replace("。", "").replace("！", "")
                    if not name:
                        tts.speak("没有听清动作名字")
                    elif pm.gesture_recorder and pm.gesture_recorder.save_named_action(name):
                        pm.waiting_for_naming = False
                        pm.operation_mode = None
                        pm.stop_gesture_recording()
                        if pm.leader_arm and pm.leader_arm.connected:
                            pm.leader_arm.set_torque(True)
                        pm.activate(reason="命名完成")
                        tts.speak(f"已存入动作库，名字是{name}")
                        tts.speak(f"你现在可以说执行{name}让二号臂复现了。")
                        print(f"✅ 动作已保存: {name}")
                    else:
                        tts.speak("保存动作失败")
                        
                except Exception as e:
                    print(f"⚠️  保存动作失败: {e}")
                    tts.speak("保存出错")
            
            elif "执行" in text:
                """【深水区优化】根据语音指令异步执行从臂上的命名动作
                改进：直接用正则匹配提取目标名，不受 action 值影响
                架构优化：使用独立线程执行，防止主循环被硬件 IO 阻塞"""
                try:
                    target = text.split("执行")[-1].strip().replace("。", "").replace("！", "")
                    if not target:
                        tts.speak("没有听清要执行的动作名字")
                    elif pm.gesture_recorder:
                        # 先尝试精确匹配
                        action_to_execute = None
                        if target in pm.gesture_recorder.library:
                            action_to_execute = target
                        else:
                            # 模糊匹配（仅当相似度足够高时）
                            matches = difflib.get_close_matches(
                                target, 
                                pm.gesture_recorder.library.keys(), 
                                n=1, 
                                cutoff=0.75
                            )
                            if matches:
                                action_to_execute = matches[0]
                        
                        if action_to_execute:
                            # 【核心优化】异步执行：启动独立线程，主循环立即继续监听
                            def async_playback(name):
                                pm.execute_action(name)
                            
                            playback_thread = threading.Thread(
                                target=async_playback, 
                                args=(action_to_execute,), 
                                daemon=True
                            )
                            playback_thread.start()
                            print(f"🎬 已启动异步回放线程: {action_to_execute}")
                        else:
                            tts.speak(f"动作库里没有{target}这个名字")
                    else:
                        tts.speak("动作库不可用")
                        
                except Exception as e:
                    print(f"⚠️  执行动作失败: {e}")
                    tts.speak("执行动作出错")

            # 短暂延迟，避免快速连续指令冲突
            time.sleep(0.2)

        except Exception as e:
            print(f"❌ robot_worker 处理指令失败: {e}")
            # 不 sys.exit()，继续等待下一个指令
            continue

    print("🤖 robot_worker 已退出")


# ==========================================
# 麦克风校准
# ==========================================
def calibrate_mic(device_id=None):
    """校准麦克风阈值"""
    try:
        if device_id is not None:
            sd.default.device = device_id

        print(f"\n🔧 校准麦克风 ({CALIBRATE_SECONDS}s)，请保持安静...")

        frames = []

        def _cb(indata, frames_count, time_info, status):
            frames.append(indata.copy())

        with sd.InputStream(samplerate=RATE, channels=CHANNELS, dtype='float32', callback=_cb):
            sd.sleep(int(CALIBRATE_SECONDS * 1000))

        if not frames:
            print("⚠️ 无法采集校准数据")
            return 0.0, 6.0, 3.0

        # 合并帧，只取左声道，应用增益
        arr = np.concatenate(frames, axis=0)
        if arr.ndim > 1 and arr.shape[1] > 1:
            arr = arr[:, 0]
            arr = np.clip(arr * 1.5, -1.0, 1.0)
        else:
            arr = arr.reshape(-1)

        rms = float(np.sqrt(np.mean(np.square(arr)))) * 1000
        start_th = max(rms * 1.5, 5.0)
        end_th = max(rms * 0.8, 2.0)

        print(f"✅ 校准完成: 环境={rms:.1f}, start_th={start_th:.1f}, end_th={end_th:.1f}")
        return rms, start_th, end_th
    except Exception as e:
        print(f"⚠️ 校准异常: {e}")
        return 0.0, 6.0, 3.0


# ==========================================
# 音频采集（生产者）
# ==========================================
def audio_callback(indata, frames, time_info, status):
    """麦克风回调：双声道→单声道→队列"""
    if EXIT_EVENT.is_set():
        return

    try:
        # 【修复1】只取左声道，避免相位抵消
        mono = indata[:, 0:1].copy() if indata.shape[1] > 1 else indata.copy()

        # 【修复2】应用增益：1.5 倍放大
        mono_amplified = np.clip(mono * 1.5, -1.0, 1.0)
        audio_bytes = (mono_amplified.reshape(-1) * 32768).astype(np.int16).tobytes()

        # 非阻塞写队列
        try:
            AUDIO_QUEUE.put_nowait(audio_bytes)
        except queue.Full:
            pass

        # 【优化】只在必要时更新显示
        with RMS_LOCK:
            global LATEST_RMS
            rms_raw = float(np.sqrt(np.mean(np.square(mono)))) * 1000
            LATEST_RMS = 0.7 * LATEST_RMS + 0.3 * rms_raw

        # 实时音量显示（简化计算）
        vol_bar_len = min(int(LATEST_RMS / 2), 20)
        bar = '█' * vol_bar_len + '-' * (20 - vol_bar_len)
        print(f"\r🎙️  [{bar}] {LATEST_RMS:5.1f}", end='', flush=True)
    except Exception:
        pass


# ==========================================
# WebSocket 处理
# ==========================================
class IFlyURL:
    """生成讯飞 WebSocket URL"""

    def __init__(self, appid, key, secret):
        self.APPID, self.APIKey, self.APISecret = appid, key, secret
        self.host = "iat-api.xfyun.cn"
        self.url = "wss://iat-api.xfyun.cn/v2/iat"

    def create_url(self):
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        sig_origin = f"host: {self.host}\ndate: {date}\nGET /v2/iat HTTP/1.1"
        sig_sha = hmac.new(self.APISecret.encode(), sig_origin.encode(), hashlib.sha256).digest()
        sig_b64 = base64.b64encode(sig_sha).decode()
        auth_origin = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{sig_b64}"'
        auth_b64 = base64.b64encode(auth_origin.encode()).decode()
        params = {"authorization": auth_b64, "date": date, "host": self.host}
        return self.url + '?' + urlencode(params)


def on_message(ws, msg):
    """处理识别结果（支持动态修正与断尾兜底）"""
    global LAST_TEXT
    try:
        data = json.loads(msg)
    except Exception:
        return

    if data.get('code') != 0:
        return

    # 提取识别文本
    result = data.get('data', {}).get('result', {})
    text = ""
    for seg in result.get('ws', []):
        cw_list = seg.get('cw', [])
        if cw_list:
            text += cw_list[0]['w']

    text = text.strip()
    if not text or text in ['。', '？', '！']:
        return

    # 更新全局缓存
    LAST_TEXT = text

    # 获取状态标志
    status = data.get('data', {}).get('status', 0)
    is_ls = result.get('ls', False)

    # 屏幕反馈
    if result.get('pgs') == 'rpl':
        print(f"✨ 修正: {text}", end='\r')
    else:
        print(f"🎙️ 识别中: {text}", end='\r')

    # 双重判定：触发指令解析
    if (status == 2 or is_ls) and text:
        print(f"\n🏁 最终识别: {text}")
        process_final_text(text)
        LAST_TEXT = ""

    # 安全关闭连接
    if is_ls:
        try:
            ws.close()
        except Exception:
            pass


def on_error(ws, error):
    # 仅当 error 是真正的异常对象时打印，过滤掉整数状态码和字符串消息
    if isinstance(error, Exception):
        print(f"\n❗ WS error: {error}")


def on_close(ws, close_status_code, close_msg):
    """断尾兜底逻辑：连接关闭时处理剩余文本"""
    global LAST_TEXT

    # 如果连接断开时还有未处理的文本，立刻处理
    if LAST_TEXT and len(LAST_TEXT.strip()) > 0:
        print(f"\n🏁 捕获到断开前的最后文本: {LAST_TEXT}")
        process_final_text(LAST_TEXT)
        LAST_TEXT = ""


# ==========================================
# 统一的指令解析与入队逻辑（三层过滤架构）
# ==========================================
def process_final_text(text):
    global pm
    
    clean_text = text.strip().replace('。', '').replace('？', '').replace('！', '')

    if pm.waiting_for_naming:
        if "命名为" not in clean_text:
            tts.speak("请说命名为加上名字")
            return
        name = clean_text.split("命名为", 1)[-1].strip()
        if not name or len(name) < 1:
            tts.speak("没有听清名字，请再说一次命名为加上名字")
            return
        try:
            if pm.gesture_recorder and pm.gesture_recorder.save_named_action(name):
                pm.waiting_for_naming = False
                tts.speak(f"动作已保存，名字是{name}。你可以说执行{name}让从臂复现这个动作。")
                print(f"✅ 动作已命名并保存: {name}")
            else:
                tts.speak("没有找到录制数据，请先录制再命名")
                pm.waiting_for_naming = False
        except Exception as e:
            print(f"⚠️ 命名失败: {e}")
            tts.speak("保存失败，请重试")
            pm.waiting_for_naming = False
        return
    
    # 口音硬纠正（方言/误识别补丁）
    ACCENT_FIX = {
        "腹壁": "从臂", "腹b": "从臂", "从b": "从臂", "2b": "从臂",
        "铜币": "从臂", "务必": "从臂", "虫币": "从臂",
        "试驾": "示教", "支教": "示教", "日教": "示教", "自觉": "示教",
        "睡觉": "示教", "是叫": "示教",
        "开启指令": "开始指令", "开启录制": "开始录制",
        "开始试教": "示教模式",
        "卟哔": "", "卟哔卟哔": ""
    }
    for wrong, right in ACCENT_FIX.items():
        if wrong in clean_text:
            clean_text = clean_text.replace(wrong, right)
            print(f"🔧 口音纠正: '{wrong}' → '{right}'")

    if any(word in clean_text for word in ["不", "别", "取消"]):
        print(f"🛑 否定语义拦截: {clean_text}")
        return
    
    # Whisper 补位：讯飞异常时纠正
    if WHISPER_AVAILABLE and pm.whisper_model and (len(clean_text) > 16 or "腹壁" in clean_text or "从b" in clean_text):
        temp_wav = "temp_recording.wav"
        if os.path.exists(temp_wav):
            print(f"🔍 讯飞结果 [{clean_text}] 异常，Whisper 介入...")
            try:
                result = pm.whisper_model.transcribe(temp_wav, language='zh')
                whisper_text = result['text'].strip()
                if len(whisper_text) > 1 and whisper_text != clean_text:
                    print(f"✅ Whisper 纠正为: {whisper_text}")
                    clean_text = whisper_text
            except Exception as e:
                print(f"⚠️  Whisper 纠正失败: {e}")
    
    if len(clean_text) < 2 or len(clean_text) > 20:
        return

    # 过滤乱码（汉字占比必须超过60%）
    chinese_chars = len(re.findall(r'[\u4e00-\u9fa5]', clean_text))
    if chinese_chars / max(len(clean_text), 1) < 0.6:
        print(f"🤫 [字符过滤] 疑似背景噪音: {clean_text}")
        return

    # --- 第二层：唤醒词检测 ---
    # 【P0修复 问题5】不应该 return，而是清洗唤醒词后继续处理
    if any(word in clean_text for word in pm.WAKE_WORDS):
        pm.activate(reason="语音唤醒")
        # 擦除唤醒词部分，继续处理后续指令
        for wake_word in pm.WAKE_WORDS:
            if wake_word in clean_text:
                clean_text = clean_text.replace(wake_word, "").strip()
                break
        # 如果清洗后没有内容，则返回
        if not clean_text or len(clean_text) < 2:
            return
    
    # --- 第三层：状态检查与指令匹配 ---
    if not pm.is_active:
        print(f"😴 [待机中] 忽略: {clean_text}")
        return

    if "执行" in clean_text:
        target = clean_text.split("执行", 1)[-1].strip()
        if not target:
            tts.speak("没有听清要执行的动作名字")
            return
        pm.activate(reason="动态执行")
        try:
            ROBOT_QUEUE.put_nowait({
                'action': '__PLAYBACK__',
                'text': clean_text,
                'target': target,
                'score': 1.0
            })
        except Exception:
            pass
        return

    # 尝试匹配 commands.yaml 中的指令
    action, score = matcher.match(clean_text)
    
    if action:
        # 成功匹配后重置计时器
        pm.activate(reason="指令重置")
        
        print(f"🎯 匹配指令: {action} (得分: {score:.2f})")
        tts_feedback = matcher.get_tts_feedback(action)
        if tts_feedback:
            tts.speak(tts_feedback)
            
        try:
            ROBOT_QUEUE.put_nowait({
                'action': action, 
                'text': clean_text, 
                'score': score
            })
        except Exception:
            pass
    else:
        print(f"⚠️ 未能识别指令: {clean_text}")


def on_open(ws):
    """启动音频发送线程"""

    def send_loop():
        status = 0
        payload_base = {
            'common': {'app_id': APP_ID},
            'business': {
                'language': 'zh_cn',
                'domain': 'iat',
                'accent': ACCENT,
                'vinfo': 1,
                'vad_eos': CONFIG.get('speech.vad_eos', 1500),
                'dwa': 'wpgs',
                'nbest': 1,
                'rlang': 'zh-cn'
            }
        }

        while not EXIT_EVENT.is_set() and ws.sock and ws.sock.connected:
            try:
                audio = AUDIO_QUEUE.get(timeout=0.2)
            except queue.Empty:
                audio = b'\x00' * (CHUNK * 2)

            if status == 0:
                payload = dict(payload_base)
                payload['data'] = {
                    'status': 0,
                    'format': 'audio/L16;rate=16000',
                    'encoding': 'raw',
                    'audio': str(base64.b64encode(audio), 'utf-8')
                }
                status = 1
            else:
                payload = {
                    'data': {
                        'status': 1,
                        'format': 'audio/L16;rate=16000',
                        'encoding': 'raw',
                        'audio': str(base64.b64encode(audio), 'utf-8')
                    }
                }

            try:
                ws.send(json.dumps(payload))
            except Exception:
                break

    threading.Thread(target=send_loop, daemon=True).start()


# ==========================================
# 主程序
# ==========================================
def find_best_mic():
    """找最佳麦克风"""
    try:
        devs = sd.query_devices()
    except Exception:
        return 0, []

    mic_list = [(i, d.get('name', '')) for i, d in enumerate(devs) if d.get('max_input_channels', 0) > 0]
    best = None
    for i, name in mic_list:
        if any(x in name.lower() for x in ['microphone', 'mic', '麦克']):
            best = i
            break
    return best if best is not None else (mic_list[0][0] if mic_list else 0), mic_list


def test_devices(dev_ids, duration=1.5):
    """测试设备并保存 WAV"""
    for dev_id in dev_ids:
        try:
            info = sd.query_devices(dev_id)
            sr = RATE
            print(f"\n测试设备 {dev_id}: {info.get('name')} (16000Hz)")

            frames = []

            def cb(indata, frames_count, time_info, status):
                frames.append(indata.copy())

            sd.default.device = dev_id
            with sd.InputStream(samplerate=sr, channels=CHANNELS, dtype='float32', callback=cb):
                print(f"录音 {duration}s... 请说话")
                sd.sleep(int(duration * 1000))

            if not frames:
                print("❌ 无数据")
                continue

            # 合并帧，只取左声道，应用增益
            arr = np.concatenate(frames, axis=0)
            if arr.ndim > 1 and arr.shape[1] > 1:
                arr = arr[:, 0]
                arr = np.clip(arr * 1.5, -1.0, 1.0)
            else:
                arr = arr.reshape(-1)

            rms = np.sqrt(np.mean(np.square(arr))) * 1000
            print(f"RMS: {rms:.1f}")

            # 保存 WAV
            int16 = (arr * 32767).astype(np.int16)
            fname = os.path.join(os.path.dirname(__file__), f'test_dev{dev_id}.wav')
            try:
                import wave
                with wave.open(fname, 'wb') as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(sr)
                    wf.writeframes(int16.tobytes())
                print(f"✅ 已保存: {fname}")
            except Exception as e:
                print(f"保存失败: {e}")

            time.sleep(0.3)
        except Exception as e:
            print(f"跳过设备 {dev_id}: {e}")


def run(device_id):
    """
    主循环
    
    设计原则：
    - WebSocket 重连独立于 robot_worker 线程
    - 语音超时不影响指令执行
    - 异常捕获确保持续监听
    """
    print("\n启动中...")
    sd.default.device = device_id

    # 校准
    global START_THRESHOLD, END_THRESHOLD
    ambient, START_THRESHOLD, END_THRESHOLD = calibrate_mic(device_id)

    sr = RATE  # 强制 16000Hz
    stream = sd.InputStream(samplerate=sr, channels=CHANNELS, dtype='float32',
                            callback=audio_callback, blocksize=int(sr * CHUNK_MS / 1000),
                            device=device_id)
    stream.start()

    print("\n🔊 麦克风已开启，开始监听（按 Ctrl+C 退出）")

    # 【关键修复】在启动语音线程之前，强制进行硬件连接握手
    # 这是避免"虚假初始化"导致所有硬件指令被跳过的必要步骤
    print("\n🔗 正在建立硬件连接...")
    if pm.leader_arm and not pm.leader_arm.connected:
        if pm.leader_arm.connect():
            print(f"✅ 硬件连接已就绪 (Port: {pm.leader_arm.port})")
        else:
            print(f"⚠️  硬件连接失败，请检查：")
            print(f"   1. SO-101 机械臂是否接入 {pm.leader_arm.port}")
            print(f"   2. 12V 电源是否已打开")
            print(f"   3. USB 数据线连接是否正常")
            print(f"   进程将继续运行，但硬件指令会被忽略")
    elif pm.leader_arm and pm.leader_arm.connected:
        print(f"✅ 硬件已连接")
    else:
        print(f"⚠️  主臂硬件模块不可用")
    
    if pm.follower_arm and not pm.follower_arm.connected:
        print(f"\n🔗 正在连接从臂 (Port: {pm.follower_arm.port})...")
        if pm.follower_arm.connect():
            print(f"✅ 从臂连接已建立 (Port: {pm.follower_arm.port})")
        else:
            print(f"⚠️  从臂连接失败，请检查端口 {pm.follower_arm.port} 及 12V 电源")
    elif pm.follower_arm and pm.follower_arm.connected:
        print(f"✅ 从臂已连接 (Port: {pm.follower_arm.port})")
    else:
        print(f"⚠️  从臂硬件模块不可用")

    # 启动机器人指令处理线程
    worker = threading.Thread(target=robot_worker, daemon=True)
    worker.start()

    # 【可选】启动键盘监听线程（用于演示中的防噪音激活）
    kbd_thread = threading.Thread(target=keyboard_listener, daemon=True)
    kbd_thread.start()

    # WebSocket 重连循环
    consecutive_errors = 0
    try:
        while not EXIT_EVENT.is_set():
            try:
                param = IFlyURL(APP_ID, API_KEY, API_SECRET)
                ws = websocket.WebSocketApp(
                    param.create_url(),
                    on_message=on_message,
                    on_error=on_error,
                    on_close=on_close
                )
                ws.on_open = on_open
                ws.run_forever(sslopt={'cert_reqs': ssl.CERT_NONE}, ping_interval=5, ping_timeout=3)

                # 清空过期队列
                while not AUDIO_QUEUE.empty():
                    try:
                        AUDIO_QUEUE.get_nowait()
                    except Exception:
                        break

                consecutive_errors = 0
                time.sleep(0.05)

            except ConnectionError as e:
                consecutive_errors += 1
                if consecutive_errors % 5 == 0:
                    print(f"⚠️  连接错误 (第 {consecutive_errors} 次): {type(e).__name__}")
                time.sleep(0.5)
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors % 10 == 0:
                    print(f"⚠️  异常 (第 {consecutive_errors} 次): {e}")
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n⏹️ 停止...")
    except Exception as e:
        print(f"\n❌ 主循环异常: {e}")
    finally:
        # 【P0修复 问题4】优雅退出：先设置停止信号再关闭
        STOP_SIGNAL.set()
        EXIT_EVENT.set()
        stream.stop()
        stream.close()
        # 【改进】加长超时时间，给 worker 充分机会响应中断信号
        print("⏳ 等待机械臂回放中断（最多 3s）...")
        worker.join(timeout=3)
        if worker.is_alive():
            print("⚠️  robot_worker 未完全退出，但继续清理...")
        pm.cleanup()
        print("✅ 已停止")


def signal_handler(sig, frame):
    EXIT_EVENT.set()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    print("--- 可用音频设备 ---")
    best_id, mic_list = find_best_mic()
    for i, name in mic_list:
        print(f"{i}: {name}")

    ans = input(f"\n输入设备 ID (默认={best_id})，或 't' 测试，或 'q' 退出: ").strip()

    if ans.lower() == 'q':
        sys.exit(0)
    elif ans.lower() == 't':
        devs = [i for i, _ in mic_list][:5]
        test_devices(devs)
        ans = input("\n输入要使用的设备 ID: ").strip()
        device_id = int(ans) if (ans and ans.isdigit()) else best_id
    else:
        # 增加 isdigit() 判断，防止误输入导致崩溃
        device_id = int(ans) if (ans and ans.isdigit()) else best_id

    run(device_id)
