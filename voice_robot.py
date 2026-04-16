#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
精简版 - iFlytek IAT 实时语音识别脚本
核心特性：
  - 双声道→单声道正确转换
  - 快速响应 Ctrl+C
  - 多方言关键词识别
  - 机器人指令队列
"""

import os, sys, time, ssl, hmac, base64, json, queue, threading, signal, hashlib
import numpy as np
from datetime import datetime
from time import mktime
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time

# ==========================================
# 依赖导入与配置加载
# ==========================================
try:
    import sounddevice as sd
    import websocket
except ImportError as e:
    print(f"❌ 缺失依赖: {e}\n请运行: pip install sounddevice websocket-client")
    sys.exit(1)

# 从 .env 读取凭据
env_path = os.path.join(os.path.dirname(__file__), '.env')
if os.path.exists(env_path):
    try:
        # 尝试 UTF-8，失败则用 GBK
        try:
            with open(env_path, 'r', encoding='utf-8') as f:
                content = f.read()
        except UnicodeDecodeError:
            with open(env_path, 'r', encoding='gbk') as f:
                content = f.read()

        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            k, v = line.split('=', 1)
            k, v = k.strip(), v.strip().strip('"').strip("'")
            if k and v and k not in os.environ:
                os.environ[k] = v
    except Exception:
        pass

APP_ID = os.environ.get('IFLY_APP_ID', 'YOUR_APP_ID_HERE')
API_KEY = os.environ.get('IFLY_API_KEY', 'YOUR_API_KEY_HERE')
API_SECRET = os.environ.get('IFLY_API_SECRET', 'YOUR_API_SECRET_HERE')

ACCENT = os.environ.get('IFLY_ACCENT', 'mandarin')
RATE = 16000
CHANNELS = 2  # 双声道：Realtek 驱动需要
CHUNK_MS = 80
CHUNK = int(RATE * CHUNK_MS / 1000)
CONFIDENCE_THRESHOLD = float(os.environ.get('IFLY_CONF_THRESHOLD', '0.12'))

# 全局控制
EXIT_EVENT = threading.Event()
AUDIO_QUEUE = queue.Queue(maxsize=150)
ROBOT_QUEUE = queue.Queue()
LATEST_RMS = 0.0
RMS_LOCK = threading.Lock()
LAST_TEXT = ""  # 【关键】用于跨函数记录最后一次识别到的文本（断尾兜底）

# VAD 阈值（校准时设置）
START_THRESHOLD = 6.0
END_THRESHOLD = 3.0
CALIBRATE_SECONDS = 1.5

# 日志与输出
LOG_FILE = os.path.join(os.path.dirname(__file__), 'commands.log')
CMD_FILE = os.path.join(os.path.dirname(__file__), 'last_command.json')


# ==========================================
# 实用工具
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


def parse_command(text):
    """多方言关键词识别"""
    if not text:
        return None

    rules = [
        # 停止动作（普通话、重庆话、上海话）
        ('STOP', [
            '停止', '莫动', '不要', '煞', '停下', '别动', '煞车', '拿停', '勒停',
            '伐要动', '别动', '不要拿'  # 上海话
        ]),
        # 抓取动作（普通话、重庆话、上海话）
        ('PICK', [
            '抓', '拿', '起来', '拿起', '抓起', '捉', '物事', '搿只', '把那', '拿那',
            '侬拿', '帮吾拿', '拿起来', '搿个物事'  # 上海话特有
        ]),
        # 放下动作（普通话、重庆话、上海话）
        ('PLACE', [
            '放', '摆', '下去', '放下', '放倒', '下', '摆下', '落去', '搁下',
            '摆辣海', '摆下去', '伐要', '落地'  # 上海话特有
        ])
    ]

    for action, keywords in rules:
        for kw in keywords:
            if kw in text:
                return action
    return None


def robot_worker():
    """处理机器人指令队列"""
    while not EXIT_EVENT.is_set():
        try:
            cmd = ROBOT_QUEUE.get(timeout=0.5)
        except queue.Empty:
            continue

        action = cmd.get('action')
        text = cmd.get('text', '')
        print(f"▶️  执行: {action} | '{text}'")
        save_cmd_json(action, text, cmd.get('score'))
        time.sleep(0.2)


def process_final_text(text):
    """【新增】统一的指令解析与入队逻辑（供 on_message 和 on_close 共同调用）"""
    action = parse_command(text)
    if action:
        print(f"🤖 指令匹配成功: {action}")
        try:
            ROBOT_QUEUE.put_nowait({'action': action, 'text': text})
        except Exception:
            pass
    else:
        print(f"⚠️ 未能识别指令关键词: {text}")


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

        # 合并帧，转单声道（取第一个声道，避免相位抵消）
        arr = np.concatenate(frames, axis=0)
        if arr.ndim > 1 and arr.shape[1] > 1:
            arr = arr[:, 0]  # 只取左声道
            # 应用同样的增益（1.5倍），使校准更准确
            arr = np.clip(arr * 1.5, -1.0, 1.0)
        else:
            arr = arr.reshape(-1)

        rms = float(np.sqrt(np.mean(np.square(arr)))) * 1000
        # 由于增益后的数值更大，调整阈值倍数
        start_th = max(rms * 1.5, 5.0)  # 更宽松，更容易触发
        end_th = max(rms * 0.8, 2.0)  # 更灵敏，更快结束句子

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
        # 【修复1】核心改动：绝对不能取平均值！强制只取左声道
        # 这彻底解决相位抵消（+1 + (-1) = 0）的问题
        if indata.shape[1] > 1:
            mono = indata[:, 0:1].copy()  # 只取左声道
        else:
            mono = indata.copy()

        # 计算原始 RMS（用于VAD判定）
        rms_raw = float(np.sqrt(np.mean(np.square(mono))))
        rms_display = rms_raw * 1000

        # 【修复2】暴力增加数字增益：振幅放大 1.5 倍
        # 这解决"音量大但讯飞收不到"的问题
        # 目标：使得大声说话时RMS能到 20-50（而不是目前的 0.2）
        mono_amplified = np.clip(mono * 1.5, -1.0, 1.0)

        # 转为 int16 bytes（音量已被放大）
        audio_bytes = (mono_amplified.reshape(-1) * 32768).astype(np.int16).tobytes()

        # 非阻塞写队列
        try:
            AUDIO_QUEUE.put_nowait(audio_bytes)
        except queue.Full:
            pass

        # 实时音量显示
        with RMS_LOCK:
            global LATEST_RMS
            LATEST_RMS = 0.7 * LATEST_RMS + 0.3 * rms_display
            vol_display = LATEST_RMS

        bar = '█' * int(min(vol_display / 2, 20)) + '-' * (20 - int(min(vol_display / 2, 20)))
        print(f"\r🎙️  [{bar}] {vol_display:5.1f}", end='', flush=True)
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
    """处理识别结果（支持动态修正与 LS 兜底触发）"""
    global LAST_TEXT
    try:
        data = json.loads(msg)
    except Exception:
        return

    if data.get('code') != 0:
        return

    # 1. 提取识别文本
    result = data.get('data', {}).get('result', {})
    ws_data = result.get('ws', [])

    text = ""
    for seg in ws_data:
        cw_list = seg.get('cw', [])
        if cw_list:
            text += cw_list[0]['w']  # 只拿最高置信度的候选
    text = text.strip()

    if not text or text in ['。', '？', '！']:
        return

    # 【关键】只要有字，就实时更新全局缓存
    if text:
        LAST_TEXT = text

    # 2. 获取关键状态标志
    pgs = result.get('pgs')  # pgs="rpl" 表示动态修正
    status = data.get('data', {}).get('status', 0)
    is_ls = result.get('ls', False)  # 句子是否结束（即便 status 不是 2）

    # 3. 屏幕反馈
    if pgs == 'rpl':
        print(f"✨ 修正: {text}", end='\r')
    else:
        print(f"🎙️ 识别中: {text}", end='\r')

    # 4. 【核心修复】双重判定触发逻辑
    # 只要满足 (status=2) 或者 (ls=True)，就代表话说明白了，立刻去解析指令
    is_final_trigger = (status == 2) or is_ls

    if is_final_trigger and text:
        print(f"\n🏁 最终识别: {text}")  # 换行打印最终定稿，方便调试
        process_final_text(text)
        LAST_TEXT = ""  # 正常触发后清空，防止 on_close 重复触发

    # 5. 安全关闭连接
    if is_ls:
        try:
            ws.close()
        except Exception:
            pass


def on_error(ws, error):
    print(f"\n❗ WS error: {error}")


def on_close(ws, close_status_code, close_msg):
    """【关键修复】连接关闭时的断尾兜底逻辑"""
    global LAST_TEXT
    # print(f"\n🔌 连接已关闭 (Code: {close_status_code})")
    
    # 【兜底逻辑】如果连接断开时，LAST_TEXT 还有内容没被处理
    if LAST_TEXT and len(LAST_TEXT.strip()) > 0:
        print(f"\n🏁 捕获到断开前的最后文本: {LAST_TEXT}")
        process_final_text(LAST_TEXT)
        LAST_TEXT = ""  # 处理完立即清空


def on_open(ws):
    """发送音频的线程"""

    def send_loop():
        status = 0
        while not EXIT_EVENT.is_set() and ws.sock and ws.sock.connected:
            try:
                audio = AUDIO_QUEUE.get(timeout=0.2)
            except queue.Empty:
                audio = b'\x00' * (CHUNK * 2)

            if status == 0:
                payload = {
                    'common': {'app_id': APP_ID},
                    'business': {
                        'language': 'zh_cn',
                        'domain': 'iat',
                        'accent': 'shanghainese',
                        'vinfo': 1,
                        'vad_eos': 1500,  # 从 3000 缩短到 1500，抢在超时前切断
                        'dwa': 'wpgs',
                        'nbest': 1,  # 只要第 1 候选，减少冗余数据
                        'rlang': 'zh-cn'
                    },
                    'data': {'status': 0, 'format': 'audio/L16;rate=16000', 'encoding': 'raw',
                             'audio': str(base64.b64encode(audio), 'utf-8')}
                }
                status = 1
            else:
                payload = {'data': {'status': 1, 'format': 'audio/L16;rate=16000', 'encoding': 'raw',
                                    'audio': str(base64.b64encode(audio), 'utf-8')}}

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
            # 【修复】强制使用16000Hz（讯飞硬性要求）
            sr = RATE
            print(f"\n测试设备 {dev_id}: {info.get('name')} (使用16000Hz)")

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

            # 合并帧，只取左声道，并应用增益
            arr = np.concatenate(frames, axis=0)
            if arr.ndim > 1 and arr.shape[1] > 1:
                arr = arr[:, 0]  # 只取左声道
                # 应用同样的增益
                arr = np.clip(arr * 1.5, -1.0, 1.0)
            else:
                arr = arr.reshape(-1)

            rms = np.sqrt(np.mean(np.square(arr))) * 1000
            print(f"RMS: {rms:.1f}")

            # 保存 WAV（用16000Hz）
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
    """主循环"""
    print("\n启动中...")
    sd.default.device = device_id

    # 校准
    global START_THRESHOLD, END_THRESHOLD
    ambient, START_THRESHOLD, END_THRESHOLD = calibrate_mic(device_id)

    # 【修复】强制采样率为16000Hz（讯飞硬性要求）
    # 不要用设备的默认采样率（可能是44100），那会导致识别乱码
    sr = RATE  # 强制 16000Hz

    stream = sd.InputStream(samplerate=sr, channels=CHANNELS, dtype='float32',
                            callback=audio_callback, blocksize=int(sr * CHUNK_MS / 1000),
                            device=device_id)
    stream.start()

    print("\n🔊 麦克风已开启，开始监听（按 Ctrl+C 退出）")

    # 启动机器人指令处理线程
    worker = threading.Thread(target=robot_worker, daemon=True)
    worker.start()

    # WebSocket 重连循环
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

                # 【关键修复】减小 ping_interval 加快响应
                ws.run_forever(sslopt={'cert_reqs': ssl.CERT_NONE}, ping_interval=5, ping_timeout=3)

                # 清空过期队列
                while not AUDIO_QUEUE.empty():
                    try:
                        AUDIO_QUEUE.get_nowait()
                    except Exception:
                        break

                time.sleep(0.05)
            except Exception as e:
                print(f"⚠️ 异常: {e}")
                time.sleep(1)
    except KeyboardInterrupt:
        print("\n⏹️ 停止...")
    finally:
        EXIT_EVENT.set()
        stream.stop()
        stream.close()
        worker.join(timeout=2)
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
        device_id = int(ans) if ans else best_id
    else:
        device_id = int(ans) if ans else best_id

    run(device_id)

