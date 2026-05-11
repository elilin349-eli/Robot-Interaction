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
import wave
import numpy as np
from datetime import datetime
from time import mktime
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from typing import Optional, Tuple
from pathlib import Path
try:
    from control_pkg.robot_system.utils.path_manager import PathManager
except Exception:
    PathManager = None
# Optional modules for extended hardware / LLM integration
try:
    from hardware import MobileBase, VisionSensor
except Exception:
    MobileBase = None
    VisionSensor = None

try:
    from llm_reasoner import LLMReasoner
except Exception:
    LLMReasoner = None

# ==========================================
# 依赖导入
# ==========================================
try:
    import sounddevice as sd
    import websocket
except ImportError as e:
    sd = None
    websocket = None
    print(f"❌ 缺失依赖: {e}\n请运行: pip install sounddevice websocket-client")
    sys.exit(1)

# Whisper 本地语音识别（备用方案）
try:
    import whisper

    WHISPER_AVAILABLE = True
    print("✅ Whisper 库已加载（可用于本地语音识别）")
except ImportError:
    whisper = None
    WHISPER_AVAILABLE = False
    print("⚠️  Whisper 库缺失。运行: pip install openai-whisper")

# 导入 LeRobot 硬件模块
try:
    from lerobot_hardware import SO101LeaderArm, GestureRecorder

    LEROBOT_AVAILABLE = True
except ImportError:
    SO101LeaderArm = None
    GestureRecorder = None
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
    pyttsx3 = None
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


def _resolve_secret(default_val: str, env_key: str) -> str:
    if isinstance(default_val, str) and default_val.strip() == f"${{{env_key}}}":
        return os.environ.get(env_key, "")
    return str(default_val or "")

APP_ID = os.environ.get('IFLY_APP_ID') or _resolve_secret(CONFIG.get('defaults.app_id', ''), 'IFLY_APP_ID')
API_KEY = os.environ.get('IFLY_API_KEY') or _resolve_secret(CONFIG.get('defaults.api_key', ''), 'IFLY_API_KEY')
API_SECRET = os.environ.get('IFLY_API_SECRET') or _resolve_secret(CONFIG.get('defaults.api_secret', ''), 'IFLY_API_SECRET')

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
STOP_SIGNAL = threading.Event()
WS_PCM_BUFFER = bytearray()
WS_PCM_LOCK = threading.Lock()

START_THRESHOLD = 6.0
END_THRESHOLD = 3.0
CALIBRATE_SECONDS = 1.5

# 项目根目录（优先使用 PathManager 自动识别）
if PathManager:
    try:
        PROJECT_ROOT = PathManager.project_root()
    except Exception:
        PROJECT_ROOT = Path(os.path.dirname(__file__)).resolve()
else:
    PROJECT_ROOT = Path(os.path.dirname(__file__)).resolve()

# 日志与临时文件位置（默认不放到共享目录）
log_dir = os.getenv('VOICE_ROBOT_LOG_DIR')
if not log_dir:
    log_dir = Path.home() / '.local' / 'state' / 'voice_robot'
log_dir = Path(log_dir)
try:
    log_dir.mkdir(parents=True, exist_ok=True)
except Exception:
    # fallback to project root
    log_dir = PROJECT_ROOT

LOG_FILE = str(log_dir.joinpath(CONFIG.get('logging.log_file', 'commands.log')))
CMD_FILE = str(log_dir.joinpath(CONFIG.get('logging.cmd_json', 'last_command.json')))
TEMP_RECORDING_WAV = log_dir / "temp_recording.wav"


# ==========================================
# TTS 语音反馈引擎
# ==========================================
class TTSEngine:
    """文字转语音（中文）"""

    def __init__(self):
        self.enabled = TTS_AVAILABLE
        self._queue = queue.Queue()
        self._thread = None
        if TTS_AVAILABLE:
            try:
                self.engine = pyttsx3.init()
                self.engine.setProperty('rate', 160)
                self.engine.setProperty('volume', 0.8)
                self._thread = threading.Thread(target=self._worker, daemon=True)
                self._thread.start()
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
                if self._thread and self._thread.is_alive():
                    self._thread.join(timeout=1.0)
                time.sleep(0.2)
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


        self.is_active = False
        self.active_timer = None
        self.WAKE_WORDS = ["机器人", "开始指令", "小臂小臂", "小臂", "激活", "小灰小灰"]
        self.ACTIVE_DURATION = 20.0
        self.operation_mode = None

        # 注册清理钩子，程序退出时强制杀死子进程
        atexit.register(self.cleanup)

        # 可选模块（移动底盘 / 视觉 / LLM），按环境变量或配置启用
        self.mobile_base = None
        self.vision = None
        self.llm = None

        try:
            if (os.getenv('ENABLE_MOBILE_BASE', '0') == '1') or CONFIG.get('hardware.mobile_base_enabled', False):
                if MobileBase:
                    self.mobile_base = MobileBase(port=CONFIG.get('hardware.mobile_port', 'COM15'))
                else:
                    print("⚠️ MobileBase 未安装或不可用")

            if (os.getenv('ENABLE_CAMERA', '0') == '1') or CONFIG.get('hardware.vision_enabled', False):
                if VisionSensor:
                    self.vision = VisionSensor(camera_id=CONFIG.get('hardware.camera_id', 0))
                    self.vision.start()
                else:
                    print("⚠️ VisionSensor 未安装或不可用")

            if (os.getenv('ENABLE_LLM', '0') == '1') or CONFIG.get('llm.enabled', False):
                if LLMReasoner:
                    self.llm = LLMReasoner()
                else:
                    print("⚠️ LLMReasoner 模块不可用")
        except Exception as e:
            print(f"⚠️ 可选模块初始化异常: {e}")

        # 初始化主臂和从臂（如果可用）
        if LEROBOT_AVAILABLE:
            self._init_leader_arm()
            self._init_follower_arm()

        self.ros_node = None
        self.grasp_pub = None
        self._ros_string_cls = None
        try:
            import rclpy
            from std_msgs.msg import String

            if not rclpy.ok():
                rclpy.init(args=[])
            self.ros_node = rclpy.create_node('voice_robot_bridge')
            self.grasp_pub = self.ros_node.create_publisher(String, '/voice_grasp_cmd', 10)
            self._ros_string_cls = String
            print("✅ ROS2 抓取发布器已就绪")
        except Exception as e:
            print(f"⚠️ ROS2 抓取发布器不可用: {e}")

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
            port = CONFIG.get('hardware.leader_port', '/dev/ttyACM0')
            self.leader_arm = SO101LeaderArm(port=port, role="leader")
            self.gesture_recorder = GestureRecorder("gestures_library.json")
            print(f"📦 主臂模块已初始化 (端口: {port})")
            print("🔗 正在建立硬件连接...")
            if not self.leader_arm.connect():
                print("⚠️  硬件连接失败，请检查：")
                print("   1. SO-101 机械臂是否接入 /dev/ttyACM0")
                print("   2. 12V 电源是否已打开")
                print("   3. USB 数据线连接是否正常")
                print("   进程将继续运行，但硬件指令会被忽略")
            else:
                print("✅ 硬件连接已建立")

        except Exception as e:
            print(f"⚠️  主臂初始化失败: {e}")
    
    def _init_follower_arm(self):
        """初始化从臂硬件接口"""
        try:
            port = CONFIG.get('hardware.follower_port', 'COM11')
            self.follower_arm = SO101LeaderArm(port=port, role="follower")
            print(f"📦 从臂模块已初始化 (端口: {port})")
            if not self.follower_arm.connect():
                print(f"⚠️  从臂连接失败，请检查 {port} 端口")
                self.follower_arm = None
            else:
                print("✅ 从臂连接已建立")
        except Exception as e:
            print(f"⚠️  从臂初始化失败: {e}")
            self.follower_arm = None

    def record_gesture(self, name: str, frames: list) -> bool:
        with self.lock:
            if not self.gesture_recorder or not name or not frames:
                return False
            self.gesture_recorder.current_recording = list(frames)
            return self.gesture_recorder.save_named_action(name)

    def _move_gripper(self, arm, target_pos: int) -> bool:
        if not arm or 6 not in getattr(arm, 'available_motors', [1, 2, 3, 4, 5, 6]):
            return False
        if not arm.connected and not arm.connect():
            return False
        return arm.safe_write(6, target_pos)

    def get_vision_objects(self):
        if not self.vision:
            return []
        try:
            return self.vision.detect_objects() or []
        except Exception as e:
            print(f"⚠️ 视觉检测失败: {e}")
            return []

    def publish_voice_grasp(self, payload) -> bool:
        if not self.grasp_pub or not self._ros_string_cls:
            return False
        try:
            msg = self._ros_string_cls()
            if isinstance(payload, dict):
                msg.data = json.dumps(payload, ensure_ascii=False)
            else:
                msg.data = json.dumps({"action": "VISION_GRASP", "target": str(payload)}, ensure_ascii=False)
            self.grasp_pub.publish(msg)
            return True
        except Exception as e:
            print(f"⚠️ 发布视觉抓取指令失败: {e}")
            return False

    def _resolve_action_name(self, target: str):
        if not self.gesture_recorder or not self.gesture_recorder.library:
            return None
        if target in self.gesture_recorder.library:
            return target
        matches = difflib.get_close_matches(target, list(self.gesture_recorder.library.keys()), n=1, cutoff=0.6)
        return matches[0] if matches else None
    
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
        """异步执行命名动作到从臂"""
        with self.lock:
            if not self.gesture_recorder or action_name not in self.gesture_recorder.library:
                print(f"⚠️  动作库中不存在: {action_name}")
                tts.speak(f"动作库里没有{action_name}这个名字")
                return False
            if not self.follower_arm or not self.follower_arm.connected:
                print(f"⚠️  从臂未连接，无法回放")
                tts.speak("从臂未连接")
                return False
            data = self.gesture_recorder.library[action_name]

        try:
            STOP_SIGNAL.clear()
            if not self.follower_arm.clear_overload_error():
                print("⚠️  硬件恢复失败")
                tts.speak("机械臂恢复失败")
                return False

            follower_data = []
            try:
                for frame in data:
                    angles = frame.get('angles') if isinstance(frame, dict) else frame
                    if not angles:
                        continue
                    if hasattr(self.follower_arm, 'map_leader_to_follower'):
                        mapped = self.follower_arm.map_leader_to_follower(angles)
                    else:
                        mapped = self.sync_arm_positions(angles)
                    new_frame = dict(frame) if isinstance(frame, dict) else {}
                    new_frame['angles'] = mapped
                    follower_data.append(new_frame)
            except Exception:
                follower_data = data

            if self.follower_arm.safe_playback(follower_data, stop_signal=STOP_SIGNAL, use_interpolation=True):
                tts.speak(f"已在从臂上执行{action_name}")
                return True
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

            self.gesture_recorder.record_gesture(angles, label)
            tts.speak(f"已记录姿态：{label}")
            return True
        except Exception as e:
            print(f"❌ 记录姿态失败: {e}")
            return False

    def sync_arm_positions(self, leader_positions):
        """将主臂位置映射为从臂目标。"""
        if self.follower_arm and hasattr(self.follower_arm, 'map_leader_to_follower'):
            return self.follower_arm.map_leader_to_follower(leader_positions)
        return [
            max(
                0,
                min(
                    4095,
                    int(leader_positions[i]) if i < len(leader_positions) and leader_positions[i] is not None else 0,
                ),
            )
            for i in range(6)
        ]

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
        frame_count = self.gesture_recorder.get_frame_count() if self.gesture_recorder else 0
        tts.speak(f"停止记录，共记录 {frame_count} 帧姿态")
        return True

    def cleanup(self):
        """退出时清理资源"""
        STOP_SIGNAL.set()
        self.stop_recording()
        if self.gesture_recorder:
            print(f"📊 动作库共 {self.gesture_recorder.get_action_count()} 个动作")

        if self.ros_node:
            try:
                self.ros_node.destroy_node()
            except Exception:
                pass

        if self.leader_arm and self.leader_arm.connected:
            try:
                self.leader_arm.set_torque(False)
                print("✅ 主臂所有舵机已释放")
            except Exception as e:
                print(f"⚠️  清理舵机力矩时出错: {e}")
            try:
                self.leader_arm.disconnect()
            except Exception as e:
                print(f"⚠️  断开连接时出错: {e}")


pm = ProcessManager()
try:
    pm.path = PathManager()
except Exception:
    pm.path = None


# ==========================================
# 机器人指令处理器
# ==========================================
def keyboard_listener():
    """按 Space 手动激活监听"""
    try:
        import keyboard
        print("💡 提示：可按 [Space] 强制激活监听")
        while not EXIT_EVENT.is_set():
            if keyboard.is_pressed('space'):
                pm.activate(reason="手动按键")
                time.sleep(1)
            time.sleep(0.1)
    except ImportError:
        pass


def robot_worker():
    print("🤖 robot_worker 已启动（后台持续监听指令）")

    error_count = 0
    while not EXIT_EVENT.is_set():
        try:
            cmd = ROBOT_QUEUE.get(timeout=0.5)
        except queue.Empty:
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

            elif action == 'VLA_DEMO':
                print("🧠 [VLA 演示] 多模态接口已触发")
                print("   → 当前阶段：数据采集与归一化接口已就绪")
                print("   → 下一阶段：接入端到端推理")
                tts.speak("视觉语言模型接口已就绪，当前演示数据管道，端到端推理将在下一阶段接入")

            elif action == 'VISION_GRASP':
                target = (cmd.get('target') or text or '').strip()
                steps = cmd.get('steps') if isinstance(cmd.get('steps'), list) else []
                if not target:
                    tts.speak('没有听清要抓取的目标')
                    continue
                plan = {
                    'action': 'VISION_GRASP',
                    'target': target,
                    'steps': steps or [
                        {'stage': 'detect_target', 'action': 'VISION_GRASP', 'target': target},
                        {'stage': 'move_to_front', 'action': 'VISION_GRASP', 'target': target, 'approach_offset': 0.08, 'settle_seconds': 0.9},
                        {'stage': 'grasp', 'action': 'VISION_GRASP', 'target': target, 'gripper_degree': 0, 'settle_seconds': 0.8},
                    ],
                    'text': text,
                }
                if plan['steps']:
                    print(f"🧠 视觉抓取规划: {len(plan['steps'])} 步")
                if pm.publish_voice_grasp(plan):
                    print(f"✅ 已发布视觉抓取指令: {target}")
                    tts.speak(f"已请求视觉系统抓取{target}")
                else:
                    tts.speak('视觉抓取通道未连接')

            elif action == 'LLM_PROCESS':
                if pm.llm:
                    try:
                        leader_angles = pm.leader_arm.read_angles() if pm.leader_arm and pm.leader_arm.connected else None
                        follower_angles = pm.follower_arm.read_angles() if pm.follower_arm and pm.follower_arm.connected else None
                        vision_objects = pm.get_vision_objects()
                        robot_state = {
                            'leader_connected': bool(pm.leader_arm and pm.leader_arm.connected),
                            'follower_connected': bool(pm.follower_arm and pm.follower_arm.connected),
                            'vision_objects': vision_objects,
                            'waiting_for_naming': pm.waiting_for_naming,
                        }
                        result = pm.llm.parse(
                            text,
                            leader_angles=leader_angles,
                            follower_angles=follower_angles,
                            vision_objects=vision_objects,
                            robot_state=robot_state,
                        )
                        act = result.get('action')
                        if act == 'CHAT':
                            tts.speak(result.get('answer', ''))
                        elif act == 'VISION_GRASP':
                            plan = {
                                'action': 'VISION_GRASP',
                                'target': result.get('target') or text,
                                'steps': result.get('steps') if isinstance(result.get('steps'), list) else [],
                                'text': text,
                                'vision_objects': vision_objects,
                            }
                            if plan['steps']:
                                print(f"🧠 视觉抓取规划: {len(plan['steps'])} 步")
                            ROBOT_QUEUE.put_nowait({'action': 'VISION_GRASP', 'text': text, 'score': 1.0, 'plan': plan})
                        elif act:
                            ROBOT_QUEUE.put_nowait({'action': act, 'text': text, 'score': 1.0})
                        else:
                            tts.speak('我没有理解你的意图')
                    except Exception as e:
                        print(f"⚠️ LLM 处理失败: {e}")
                        tts.speak('智能解析失败')
                else:
                    tts.speak('智能模式未启用')

            elif action == '__PLAYBACK__':
                target = (cmd.get('target') or '').strip()
                if not target:
                    tts.speak("没有听清要执行的动作名字")
                    continue

                selected = pm._resolve_action_name(target)
                if not selected:
                    tts.speak("动作库里没有这个名字")
                    continue

                tts.speak(f"正在执行{selected}")
                threading.Thread(target=pm.execute_action, args=(selected,), daemon=True).start()

            elif action in ('PICK', 'PLACE', 'FOLLOWER_PICK', 'FOLLOWER_PLACE'):
                try:
                    is_pick = 'PICK' in action
                    arm = pm.leader_arm if action in ('PICK', 'PLACE') else pm.follower_arm
                    key = 'hardware.gripper_pick_pos' if action == 'PICK' else 'hardware.gripper_place_pos'
                    if action == 'FOLLOWER_PICK':
                        key = 'hardware.follower_gripper_pick_pos'
                    elif action == 'FOLLOWER_PLACE':
                        key = 'hardware.follower_gripper_place_pos'
                    target_pos = int(CONFIG.get(key, 2600 if is_pick else 1600))
                    if pm._move_gripper(arm, target_pos):
                        print(f"✅ {action} 已执行")
                    else:
                        print(f"⚠️  {action} 失败")
                except Exception as e:
                    print(f"⚠️  {action} 执行失败: {e}")
                    threading.Thread(target=lambda: tts.speak("操作执行遇到异常"), daemon=True).start()

            elif action == 'JOINT_CONTROL':
                try:
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

                    current_poses = pm.leader_arm.read_angles()
                    if not current_poses:
                        print("⚠️  无法读取关节位置")
                        continue

                    current_pos = current_poses[joint_id - 1]
                    print(f"📍 当前位置: {current_pos}")

                    if "随机" in text or "乱动" in text:
                        target = np.random.randint(1200, 2800)
                    elif "复位" in text or "回到" in text or "中间" in text:
                        target = 2048
                    elif "转一点" in text or "增加" in text or "往前" in text:
                        target = current_pos + 400
                    elif "反向" in text or "减少" in text or "往后" in text:
                        target = current_pos - 400
                    elif "打开" in text and joint_id == 6:
                        target = 1000
                    elif "闭合" in text and joint_id == 6:
                        target = 2800
                    else:
                        target = 2048

                    final_target = max(1000, min(target, 3000))
                    if final_target != target:
                        print(f"⚠️  目标值 {target} 超出安全范围，已裁剪至 {final_target}")

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
                try:
                    print("🛑 紧急停止")
                    STOP_SIGNAL.set()
                    threading.Thread(target=lambda: tts.speak("已紧急停止"), daemon=True).start()

                    if pm.leader_arm and pm.leader_arm.connected:
                        pm.leader_arm.set_torque(False)
                        print("✅ 主臂舵机已释放")

                    if pm.follower_arm and pm.follower_arm.connected:
                        pm.follower_arm.set_torque(False)
                        print("✅ 从臂舵机已释放")

                    time.sleep(0.15)
                    STOP_SIGNAL.clear()
                    print("✅ 系统已完全停止")

                except Exception as e:
                    print(f"⚠️  释放舵机失败: {e}")
                    try:
                        STOP_SIGNAL.clear()
                    except:
                        pass
            
            elif "命名为" in text:
                try:
                    name = text.split("命名为")[-1].strip().replace("。", "").replace("！", "")
                    if not name:
                        tts.speak("没有听清动作名字")
                    elif pm.record_gesture(name, pm.gesture_recorder.current_recording if pm.gesture_recorder else []):
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
                try:
                    target = text.split("执行")[-1].strip().replace("。", "").replace("！", "")
                    if not target:
                        tts.speak("没有听清要执行的动作名字")
                    elif pm.gesture_recorder:
                        action_to_execute = None
                        if target in pm.gesture_recorder.library:
                            action_to_execute = target
                        else:
                            matches = difflib.get_close_matches(
                                target,
                                pm.gesture_recorder.library.keys(),
                                n=1,
                                cutoff=0.75
                            )
                            if matches:
                                action_to_execute = matches[0]

                        if action_to_execute:
                            playback_thread = threading.Thread(
                                target=pm.execute_action,
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

        # 合并帧，多通道取均值后应用增益
        arr = np.concatenate(frames, axis=0)
        if arr.ndim > 1:
            arr = np.mean(arr, axis=1)
        else:
            arr = arr.reshape(-1)
        arr = np.clip(arr * 1.5, -1.0, 1.0)

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
        mono = np.mean(indata, axis=1, keepdims=True) if indata.shape[1] > 1 else indata.copy()
        mono_amplified = np.clip(mono * 1.5, -1.0, 1.0)
        audio_bytes = (mono_amplified.reshape(-1) * 32768).astype(np.int16).tobytes()

        try:
            AUDIO_QUEUE.put_nowait(audio_bytes)
        except queue.Full:
            pass

        with RMS_LOCK:
            global LATEST_RMS
            rms_raw = float(np.sqrt(np.mean(np.square(mono)))) * 1000
            LATEST_RMS = 0.7 * LATEST_RMS + 0.3 * rms_raw

        vol_bar_len = min(int(LATEST_RMS / 2), 20)
        bar = '█' * vol_bar_len + '-' * (20 - vol_bar_len)
        print(f"\r🎙️  [{bar}] {LATEST_RMS:5.1f}", end='', flush=True)
    except Exception:
        pass


def _reset_ws_pcm_buffer():
    with WS_PCM_LOCK:
        WS_PCM_BUFFER.clear()


def _append_ws_pcm_chunk(chunk: bytes):
    if not chunk:
        return
    with WS_PCM_LOCK:
        WS_PCM_BUFFER.extend(chunk)
        max_bytes = RATE * 2 * 20
        if len(WS_PCM_BUFFER) > max_bytes:
            del WS_PCM_BUFFER[:-max_bytes]


def _save_temp_wav(path: Path):
    with WS_PCM_LOCK:
        pcm = bytes(WS_PCM_BUFFER)
    if not pcm:
        return
    try:
        with wave.open(str(path), 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(RATE)
            wf.writeframes(pcm)
    except Exception as e:
        print(f"⚠️ 临时 WAV 保存失败: {e}")


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
    """处理识别结果"""
    global LAST_TEXT
    try:
        data = json.loads(msg)
    except Exception:
        return

    if data.get('code') != 0:
        return

    result = data.get('data', {}).get('result', {})
    text = ""
    for seg in result.get('ws', []):
        cw_list = seg.get('cw', [])
        if cw_list:
            text += cw_list[0]['w']

    text = text.strip()
    if not text or text in ['。', '？', '！']:
        return

    LAST_TEXT = text
    status = data.get('data', {}).get('status', 0)
    is_ls = result.get('ls', False)

    if result.get('pgs') == 'rpl':
        print(f"✨ 修正: {text}", end='\r')
    else:
        print(f"🎙️ 识别中: {text}", end='\r')

    if (status == 2 or is_ls) and text:
        print(f"\n🏁 最终识别: {text}")
        process_final_text(text)
        LAST_TEXT = ""

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
    """连接关闭时处理剩余文本"""
    global LAST_TEXT
    _save_temp_wav(TEMP_RECORDING_WAV)
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
            tts.speak("没有听清名字，请再说一次")
            return
        try:
            if pm.record_gesture(name, pm.gesture_recorder.current_recording if pm.gesture_recorder else []):
                pm.waiting_for_naming = False
                pm.activate(reason="命名完成")
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
    
    ACCENT_FIX = {
        "腹壁": "从臂", "腹b": "从臂", "从b": "从臂", "2b": "从臂", "铜币": "从臂",
        "试驾": "示教", "支教": "示教", "日教": "示教", "自觉": "示教", "睡觉": "示教",
        "开启指令": "开始指令", "开启录制": "开始录制", "卟哔": "", "卟哔卟哔": ""
    }
    for wrong, right in ACCENT_FIX.items():
        if wrong in clean_text:
            clean_text = clean_text.replace(wrong, right)
            print(f"🔧 口音纠正: '{wrong}' → '{right}'")

    record_stop_whitelist = ["停止录制", "结束录制", "停止记录", "结束记录", "录好了", "学完了", "停止识别", "结束识别", "采集结束"]
    if not any(kw in clean_text for kw in record_stop_whitelist):
        negative_phrases = ["不要", "别执行", "取消指令", "算了", "不用了", "别动"]
        if any(phrase in clean_text for phrase in negative_phrases):
            print(f"🛑 否定语义拦截: {clean_text}")
            return

    if WHISPER_AVAILABLE and whisper_engine.enabled and (len(clean_text) > 16 or "腹壁" in clean_text or "从b" in clean_text):
        temp_wav = str(TEMP_RECORDING_WAV)
        if os.path.exists(temp_wav):
            print(f"🔍 讯飞结果 [{clean_text}] 异常，Whisper 介入...")
            try:
                whisper_text = whisper_engine.transcribe(temp_wav) or ""
                if len(whisper_text) > 1 and whisper_text != clean_text:
                    print(f"✅ Whisper 纠正为: {whisper_text}")
                    clean_text = whisper_text
            except Exception as e:
                print(f"⚠️  Whisper 纠正失败: {e}")
    
    max_len = 60 if (pm.llm and CONFIG.get('llm.intercept_all', False)) else 20
    if len(clean_text) < 2 or len(clean_text) > max_len:
        return

    chinese_chars = len(re.findall(r'[\u4e00-\u9fa5]', clean_text))
    if chinese_chars / max(len(clean_text), 1) < 0.6:
        print(f"🤫 [字符过滤] 疑似背景噪音: {clean_text}")
        return

    if any(word in clean_text for word in pm.WAKE_WORDS):
        pm.activate(reason="语音唤醒")
        for wake_word in pm.WAKE_WORDS:
            if wake_word in clean_text:
                clean_text = clean_text.replace(wake_word, "").strip()
                break
        if not clean_text or len(clean_text) < 2:
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

    # 运行时重载/保存 joint_offsets.json（方便调试与校准）
    if any(kw in clean_text for kw in ("加载偏移", "重载偏移", "加载校准", "重载校准")):
        if pm.follower_arm:
            try:
                ok = pm.follower_arm.load_joint_offsets_file()
                if ok:
                    tts.speak("已重载从臂偏移文件")
                else:
                    tts.speak("重载偏移失败，请检查 joint_offsets.json 是否存在且格式正确")
            except Exception as e:
                print(f"⚠️ 重载偏移异常: {e}")
                tts.speak("重载偏移时发生错误")
        else:
            tts.speak("从臂模块不可用，无法重载偏移")
        return

    if any(kw in clean_text for kw in ("保存偏移", "写入偏移", "保存校准")):
        if pm.follower_arm:
            try:
                ok = pm.follower_arm.save_joint_offsets_file()
                if ok:
                    tts.speak("已将当前偏移保存到 joint_offsets.json")
                else:
                    tts.speak("保存偏移失败")
            except Exception as e:
                print(f"⚠️ 保存偏移异常: {e}")
                tts.speak("保存偏移时发生错误")
        else:
            tts.speak("从臂模块不可用，无法保存偏移")
        return

    if not pm.is_active:
        print(f"😴 [待机中] 忽略: {clean_text}")
        return

    if pm.llm and CONFIG.get('llm.intercept_all', False):
        try:
            ROBOT_QUEUE.put_nowait({'action': 'LLM_PROCESS', 'text': clean_text, 'score': 1.0})
            return
        except Exception:
            pass

    action, score = matcher.match(clean_text)
    
    if action:
        pm.activate(reason="指令重置")
        
        print(f"🎯 匹配指令: {action} (得分: {score:.2f})")
        tts_feedback = matcher.get_tts_feedback(action)
        if tts_feedback:
            tts.speak(tts_feedback)
            
        if action in ('GESTURE_STOP', 'RECORD_STOP'):
            pm.waiting_for_naming = True

        try:
            ROBOT_QUEUE.put_nowait({
                'action': action, 
                'text': clean_text, 
                'score': score
            })
        except Exception:
            pass
    else:
        if pm.llm:
            try:
                ROBOT_QUEUE.put_nowait({'action': 'LLM_PROCESS', 'text': clean_text, 'score': 1.0})
                print(f"🧠 规则未命中，转交 LLM: {clean_text}")
            except Exception:
                print(f"⚠️ 未能识别指令: {clean_text}")
        else:
            print(f"⚠️ 未能识别指令: {clean_text}")


def on_open(ws):
    """启动音频发送线程"""
    _reset_ws_pcm_buffer()

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
                _append_ws_pcm_chunk(audio)
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

            # 合并帧，多通道取均值后应用增益
            arr = np.concatenate(frames, axis=0)
            if arr.ndim > 1:
                arr = np.mean(arr, axis=1)
            else:
                arr = arr.reshape(-1)
            arr = np.clip(arr * 1.5, -1.0, 1.0)

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
    """主循环：语音识别 + 指令处理"""
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
            print(f"⚠️  硬件连接失败，请检查 {pm.leader_arm.port} 端口和 12V 电源")
            print("   进程将继续运行，但硬件指令会被忽略")
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
        STOP_SIGNAL.set()
        EXIT_EVENT.set()
        stream.stop()
        stream.close()
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

def main(args=None):
    """这是 ROS 2 真正能认出来的入口"""
    print("\n--- Voice-Robot 系统启动中 ---")
    root_env = os.getenv("VOICE_ROBOT_ROOT")
    if not root_env:
        print("⚠️  VOICE_ROBOT_ROOT 未设置，正在使用自动检测路径")
    print(f"Voice-Robot root: {PROJECT_ROOT}")
    best_id, mic_list = find_best_mic()
    for i, name in mic_list:
        print(f"{i}: {name}")

    # 为了防止 ROS 运行时卡死，我们稍微优化一下输入逻辑
    # Non-interactive startup: prefer environment variable MIC_DEVICE_ID
    env_choice = os.getenv('MIC_DEVICE_ID')
    if env_choice and env_choice.isdigit():
        device_id = int(env_choice)
    else:
        device_id = best_id

    run(device_id)

# 这一行一定要留着，方便你直接 python3 voice_robot.py 调试
if __name__ == '__main__':
    main()
