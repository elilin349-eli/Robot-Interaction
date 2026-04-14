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

import os, sys, time, ssl, hmac, base64, json, queue, threading, signal, hashlib, atexit, difflib, subprocess
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

APP_ID = os.environ.get('IFLY_APP_ID', CONFIG.get('defaults.app_id', 'YOUR_APP_ID_HERE'))
API_KEY = os.environ.get('IFLY_API_KEY', CONFIG.get('defaults.api_key', 'YOUR_API_KEY_HERE'))
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
    _lock = threading.Lock()
    
    def __init__(self):
        self.enabled = TTS_AVAILABLE
        self.engine = None
        self.speak_thread = None
        if TTS_AVAILABLE:
            try:
                self.engine = pyttsx3.init()
                self.engine.setProperty('rate', 150)  # 语速
                self.engine.setProperty('volume', 0.8)  # 音量
                # 注册退出钩子，防止 GC 时模块已被销毁导致的 NoneType 崩溃
                atexit.register(self.stop)
            except Exception as e:
                print(f"⚠️  TTS 初始化失败: {e}")
                self.enabled = False

    def stop(self):
        """显式停止引擎"""
        if self.enabled and self.engine:
            try:
                self.engine.stop()
            except:
                pass

    def speak(self, text):
        """播放语音反馈（异步，不阻塞主线程）"""
        if not self.enabled or not text:
            return
        
        # 【修复】使用独立线程运行 TTS，避免 run loop 冲突
        def _speak_async():
            try:
                with self.__class__._lock:
                    if self.engine:
                        self.engine.say(text)
                        self.engine.runAndWait()
            except Exception as e:
                # 静默捕获 TTS 错误，防止程序崩溃
                print(f"⚠️  [后台] TTS 播放异常: {e}")
        
        # 启动后台线程，不阻塞主程序
        self.speak_thread = threading.Thread(target=_speak_async, daemon=True)
        self.speak_thread.start()

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
        """
        匹配指令。返回 (action_name, similarity_score)
        similarity_score: 0-1，表示置信度
        """
        if not text:
            return None, 0.0

        best_action = None
        best_score = 0.0

        for action_name, action_config in self.commands.items():
            if not isinstance(action_config, dict):
                continue

            keywords = action_config.get('keywords', [])
            
            # 【精确匹配】：只要包含任何关键词就立刻返回
            for kw in keywords:
                if kw in text:
                    return action_name, 1.0

            # 【模糊匹配】：用 difflib 计算相似度
            for kw in keywords:
                score = difflib.SequenceMatcher(None, text, kw).ratio()
                if score > best_score:
                    best_score = score
                    best_action = action_name

        # 只返回超过阈值的结果
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
    """管理 LeRobot 子进程的生命周期"""
    def __init__(self):
        self.process = None
        self.lock = threading.Lock()
        
        # 主臂硬件接口
        self.leader_arm = None
        self.gesture_recorder = None
        self.recording_gestures = False
        
        # 注册清理钩子，程序退出时强制杀死子进程
        atexit.register(self.cleanup)
        
        # 初始化主臂（如果可用）
        if LEROBOT_AVAILABLE:
            self._init_leader_arm()

    def start_recording(self, env_name='lerobot', robot_path='lerobot/configs/robot/so_101'):
        """启动 LeRobot 数据采集"""
        with self.lock:
            if self.process is not None:
                print("⚠️  LeRobot 进程已在运行")
                return False

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
                return True
            except Exception as e:
                print(f"⚠️  停止 LeRobot 时出错: {e}")
                self.process = None
                return False
    
    def _init_leader_arm(self):
        """初始化主臂硬件接口"""
        try:
            port = CONFIG.get('hardware.leader_port', 'COM12')
            self.leader_arm = SO101LeaderArm(port=port)
            self.gesture_recorder = GestureRecorder("gestures.json")
            
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
        if not LEROBOT_AVAILABLE or not self.leader_arm:
            print("⚠️  主臂硬件不可用")
            return False
        
        self.recording_gestures = True
        print("🎬 开始连续记录主臂姿态（后台线程）")
        tts.speak("开始记录主臂姿态")
        
        def record_loop():
            interval = CONFIG.get('hardware.record_interval_ms', 100) / 1000.0
            frame_count = 0
            while self.recording_gestures and not EXIT_EVENT.is_set():
                try:
                    if not self.leader_arm.connected:
                        if not self.leader_arm.connect():
                            time.sleep(1)
                            continue
                    
                    angles = self.leader_arm.read_angles()
                    if angles:
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
        """清理钩子：程序退出时调用"""
        self.stop_recording()
        if self.gesture_recorder:
            print(f"📊 共记录 {self.gesture_recorder.get_count()} 个姿态")
        
        # 【舵机过载保护】尝试释放所有舵机的力矩，即使某个舵机已过载也不中断清理流程
        if self.leader_arm and self.leader_arm.connected:
            try:
                # 遍历 6 个舵机逐个释放力矩（地址 40 写入 0）
                for motor_id in range(1, 7):
                    try:
                        self.leader_arm.bus._write(40, 1, motor_id, 0)
                    except Exception as e:
                        # 某个舵机可能已过载，记录但继续释放其他舵机
                        print(f"⚠️  释放电机 {motor_id} 力矩时出错（可能已过载）: {e}")
                        continue
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
            elif action == 'GESTURE_RECORD':
                pm.record_gesture_from_leader(label=text)
            elif action == 'GESTURE_START':
                pm.start_gesture_recording()
            elif action == 'GESTURE_STOP':
                pm.stop_gesture_recording()
            
            # 【新增】机械臂实时控制逻辑（使用 safe_write 保护）
            elif action == 'PICK':
                try:
                    print("🦾 执行实时抓取动作 (6 号舵机)...")
                    # 【优化】异步TTS反馈，不阻塞动作执行
                    threading.Thread(target=lambda: tts.speak("收到抓取指令"), daemon=True).start()
                    
                    if pm.leader_arm and pm.leader_arm.connected:
                        # 【关键】实时反馈机制：读取当前位置
                        current_angles = pm.leader_arm.read_angles()
                        if current_angles:
                            print(f"🔍 调试: 6号舵机当前位置 {current_angles[5]} (范围: 1000-2800)")
                        
                        # 执行抓取：使用 safe_write 确保在安全范围内
                        pm.leader_arm.safe_write(6, 2800)  # 闭合位置（安全范围内）
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
                    
                    if pm.leader_arm and pm.leader_arm.connected:
                        current_angles = pm.leader_arm.read_angles()
                        if current_angles:
                            print(f"🔍 调试: 6号舵机当前位置 {current_angles[5]} (范围: 1000-2800)")
                        
                        # 执行放下：使用 safe_write 确保在安全范围内
                        pm.leader_arm.safe_write(6, 1000)  # 打开位置（安全范围内）
                        print("✅ 放下动作已执行")
                    else:
                        print("⚠️  主臂未连接，无法执行放下")
                        threading.Thread(target=lambda: tts.speak("主臂未连接"), daemon=True).start()
                
                except Exception as e:
                    print(f"⚠️  放下动作执行失败: {e}")
                    threading.Thread(target=lambda: tts.speak("操作执行遇到异常"), daemon=True).start()
            
            elif action == 'STOP':
                try:
                    print("🛑 紧急停止：释放所有舵机力矩...")
                    threading.Thread(target=lambda: tts.speak("已紧急停止"), daemon=True).start()
                    
                    if pm.leader_arm and pm.leader_arm.connected:
                        # 逐个释放舵机，忽略过载错误
                        for motor_id in range(1, 7):
                            try:
                                pm.leader_arm.bus._write(40, 1, motor_id, 0)
                            except Exception:
                                pass  # 过载舵机可能无法释放，忽略
                        print("✅ 所有舵机已释放")
                    else:
                        print("⚠️  主臂未连接，无法释放舵机")
                
                except Exception as e:
                    print(f"⚠️  释放舵机失败: {e}")
                    threading.Thread(target=lambda: tts.speak("释放操作失败"), daemon=True).start()
            
            elif action == 'ARM_RELAX':
                # 【新增】进入调试模式：松开所有舵机力矩
                try:
                    print("🎯 进入调试模式，松开机械臂...")
                    if pm.leader_arm and pm.leader_arm.connected:
                        for motor_id in range(1, 7):
                            try:
                                pm.leader_arm.bus._write(40, 1, motor_id, 0)
                            except Exception:
                                pass
                        print("✅ 机械臂已松开，可以手动拖动进行示教")
                        tts.speak("机械臂已进入示教模式")
                except Exception as e:
                    print(f"⚠️  松开失败: {e}")
            
            elif action == 'ARM_LOCK':
                # 【新增】退出调试模式：锁定所有舵机
                try:
                    print("🔒 退出调试模式，锁定机械臂...")
                    if pm.leader_arm and pm.leader_arm.connected:
                        for motor_id in range(1, 7):
                            try:
                                pm.leader_arm.bus._write(40, 1, motor_id, 1)
                            except Exception:
                                pass
                        print("✅ 机械臂已锁定")
                        tts.speak("机械臂已锁定")
                except Exception as e:
                    print(f"⚠️  锁定失败: {e}")
            
            elif action == 'RESET_GRIPPER':
                # 【新增】故障恢复：重启 6 号舵机
                try:
                    print("🔄 正在重启 6 号舵机（夹持器）...")
                    threading.Thread(target=lambda: tts.speak("正在重启夹持器"), daemon=True).start()
                    
                    if pm.leader_arm and pm.leader_arm.connected:
                        # 1. 先释放力矩
                        try:
                            pm.leader_arm.bus._write(40, 1, 6, 0)
                            print("  1️⃣  已释放舵机力矩")
                        except Exception as e:
                            print(f"  ⚠️  释放力矩失败: {e}")
                        
                        # 2. 等待 1 秒
                        time.sleep(1)
                        
                        # 3. 重新启用力矩
                        try:
                            pm.leader_arm.bus._write(40, 1, 6, 1)
                            print("  2️⃣  已重新启用力矩")
                        except Exception as e:
                            print(f"  ⚠️  启用力矩失败: {e}")
                        
                        # 4. 读取当前位置确认
                        angles = pm.leader_arm.read_angles()
                        if angles and angles[5]:
                            print(f"✅ 舵机已恢复，当前位置: {angles[5]}")
                            threading.Thread(target=lambda: tts.speak("夹持器已恢复"), daemon=True).start()
                        else:
                            print("❌ 舵机仍未响应")
                            threading.Thread(target=lambda: tts.speak("舵机恢复失败，请检查硬件"), daemon=True).start()
                    else:
                        print("⚠️  主臂未连接，无法重启舵机")
                        # 尝试重新连接
                        if pm.leader_arm:
                            print("🔗 尝试重新连接硬件...")
                            if pm.leader_arm.connect():
                                print("✅ 硬件重新连接成功")
                                threading.Thread(target=lambda: tts.speak("硬件已重新连接"), daemon=True).start()
                
                except Exception as e:
                    print(f"❌ 舵机重启失败: {e}")
                    threading.Thread(target=lambda: tts.speak("重启失败"), daemon=True).start()
            

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
# 统一的指令解析与入队逻辑
# ==========================================
def process_final_text(text):
    """统一的指令解析与入队逻辑"""
    action, score = matcher.match(text)
    if action:
        print(f"🎯 指令匹配: {action} (置信度: {score:.2f})")
        tts_feedback = matcher.get_tts_feedback(action)
        if tts_feedback:
            tts.speak(tts_feedback)
        try:
            ROBOT_QUEUE.put_nowait({'action': action, 'text': text, 'score': score})
        except Exception:
            pass
    else:
        print(f"⚠️  未能识别指令: {text}")


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

    # 启动机器人指令处理线程
    worker = threading.Thread(target=robot_worker, daemon=True)
    worker.start()

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
        EXIT_EVENT.set()
        stream.stop()
        stream.close()
        worker.join(timeout=2)
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

