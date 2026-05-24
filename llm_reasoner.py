import os
import json
import re
import importlib
import requests
from typing import Any, Dict, List, Optional


VALID_ACTIONS = {
    "PICK", "PLACE", "STOP", "ARM_RELAX", "ARM_LOCK",
    "RECORD_START", "RECORD_STOP", "GESTURE_RECORD", "GESTURE_START",
    "GESTURE_STOP", "FOLLOWER_PICK", "FOLLOWER_PLACE", "VISION_GRASP",
    "VLA_DEMO", "CHAT", "MOVE_FORWARD", "MOVE_BACKWARD",
}

try:
    from control_pkg.robot_system.cognition.intent_schema import normalize_intent
except Exception:
    def normalize_intent(data: Dict[str, Any], fallback_text: str = "") -> Dict[str, Any]:
        payload = dict(data or {})
        action = str(payload.get("action") or "CHAT").strip().upper()
        if action not in VALID_ACTIONS:
            return {"action": "CHAT", "answer": fallback_text or ""}
        payload["action"] = action
        return payload

# 延迟而稳健地尝试导入 openai（兼容旧版/新版 SDK）
openai_module = None
OpenAI = None
HAS_OPENAI = False
try:
    openai_module = importlib.import_module("openai")
    # 新版 openai (openai>=1.0) 提供 OpenAI 类
    OpenAI = getattr(openai_module, "OpenAI", None)
    HAS_OPENAI = True
except Exception:
    openai_module = None
    OpenAI = None
    HAS_OPENAI = False


class LLMReasoner:
    def __init__(self):
        self.client = None
        self.model = None
        # provider: 'ollama' | 'openai' | 'deepseek'
        self.provider = os.getenv("LLM_PROVIDER", "ollama").lower()

        self.client = None
        self.base_url = os.getenv("LLM_BASE_URL", "http://localhost:11434/v1")
        self.model = os.getenv("LLM_MODEL", "qwen2.5-coder:7b")
        # reuse HTTP session for Ollama / HTTP providers
        self.session = requests.Session()
        self.request_timeout = float(os.getenv("LLM_HTTP_TIMEOUT", "15.0"))

        # 如果指定使用 OpenAI / Deepseek 风格 SDK，优先使用可用的 openai 库
        if self.provider in ("openai", "deepseek"):
            if not HAS_OPENAI or openai_module is None:
                print("⚠️ LLMReasoner 未启用: 缺少 openai 依赖")
            else:
                try:
                    # 新版 OpenAI Python SDK (OpenAI class)
                    if OpenAI is not None:
                        api_key = os.getenv("DEEPSEEK_API_KEY") if self.provider == "deepseek" else os.getenv("OPENAI_API_KEY")
                        base_url = os.getenv("DEEPSEEK_BASE_URL") if self.provider == "deepseek" else os.getenv("OPENAI_BASE_URL")
                        # 延迟创建客户端实例（如果 SDK 需要），否则保留 module
                        try:
                            self.client = OpenAI(api_key=api_key, base_url=base_url) if api_key or base_url else OpenAI()
                        except Exception:
                            # 某些环境下不需要实例化
                            self.client = openai_module
                    else:
                        self.client = openai_module
                except Exception as e:
                    print(f"⚠️ LLMReasoner 初始化失败 (openai): {e}")
                    self.client = None
        else:
            # 默认使用 Ollama HTTP API 回退（不依赖 openai 库）
            # 我们使用 requests 直接调用 HTTP 接口到 base_url (例如 http://localhost:11434/v1)
            self.client = None

        self.system_prompt = (
            "你是机器人指令解析器。根据用户输入，从下列动作中选一个返回JSON：\n"
            '{"action": "PICK"}          // 主臂抓取\n'
            '{"action": "PLACE"}         // 主臂放下\n'
            '{"action": "ARM_RELAX"}     // 进入示教模式\n'
            '{"action": "STOP"}          // 紧急停止\n'
            '{"action": "FOLLOWER_PICK"} // 从臂抓取\n'
            '{"action": "FOLLOWER_PLACE"} // 从臂放下\n'
            '{"action": "VISION_GRASP", "target": "物体名", "steps": [...]} // 视觉抓取指定目标\n'
            '{"action": "MOVE_FORWARD"}  // 小车前进\n'
            '{"action": "MOVE_BACKWARD"} // 小车后退\n'
            "其中 VISION_GRASP 必须返回 steps，且顺序固定为：detect_target -> move_to_front -> grasp。\n"
            "move_to_front 步骤应尽量给出 approach_offset（默认 0.08）。\n"
            "如果指令无法匹配任何动作，返回 {\"action\": \"CHAT\", \"answer\": \"抱歉，我还不理解这个指令\"}."
        )

    def _extract_json_from_text(self, text: str) -> Optional[dict]:
        """Try to robustly extract the first JSON object from text.

        Strategy:
        1. direct json.loads
        2. search for first balanced `{...}` substring and try loads
        3. fallback to None
        """
        if not text or not isinstance(text, str):
            return None
        t = text.strip()
        # try direct
        try:
            return json.loads(t)
        except Exception:
            pass

        # find first '{' and attempt to find balanced '}'
        start = t.find('{')
        if start == -1:
            return None
        depth = 0
        for i in range(start, len(t)):
            ch = t[i]
            if ch == '{':
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0:
                    candidate = t[start:i + 1]
                    try:
                        return json.loads(candidate)
                    except Exception:
                        # continue searching for next balanced region
                        start = t.find('{', start + 1)
                        if start == -1:
                            break
                        depth = 0
        return None

    def _build_context_prompt(
        self,
        leader_angles: Optional[List[int]] = None,
        follower_angles: Optional[List[int]] = None,
        vision_objects: Optional[List[Dict[str, Any]]] = None,
        robot_state: Optional[Dict[str, Any]] = None,
    ) -> str:
        parts = []
        if leader_angles:
            parts.append(f"- 主臂关节角度: {leader_angles}")
        if follower_angles:
            parts.append(f"- 从臂关节角度: {follower_angles}")
        if vision_objects:
            parts.append(f"- 视觉检测结果: {vision_objects}")
        if robot_state:
            parts.append(f"- 机器人状态: {robot_state}")
        if not parts:
            return ""
        return "\n\n当前现场状态：\n" + "\n".join(parts) + "\n"

    def _default_vision_grasp_steps(self, target: str) -> List[Dict[str, Any]]:
        return [
            {"stage": "detect_target", "action": "VISION_GRASP", "target": target},
            {"stage": "move_to_front", "action": "VISION_GRASP", "target": target, "approach_offset": 0.08, "settle_seconds": 0.9},
            {"stage": "grasp", "action": "VISION_GRASP", "target": target, "gripper_degree": 0, "settle_seconds": 0.8},
        ]

    def parse(
        self,
        text: str,
        leader_angles: Optional[List[int]] = None,
        follower_angles: Optional[List[int]] = None,
        vision_objects: Optional[List[Dict[str, Any]]] = None,
        robot_state: Optional[Dict[str, Any]] = None,
    ) -> dict:
        if not self.client and os.getenv("LLM_PROVIDER", "ollama").lower() != "ollama":
            return {"action": "CHAT", "answer": "LLM 未启用"}

        # 构造消息体
        system_prompt = self.system_prompt + self._build_context_prompt(
            leader_angles=leader_angles,
            follower_angles=follower_angles,
            vision_objects=vision_objects,
            robot_state=robot_state,
        )
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": text}
        ]

        try:
            provider = os.getenv("LLM_PROVIDER", "ollama").lower()
            content = ""
            result = None

            # 优先使用 openai SDK（新版或旧版兼容）
            if HAS_OPENAI and openai_module is not None and self.provider in ("openai", "deepseek"):
                try:
                    # 新版 OpenAI SDK: client.chat.completions.create
                    if OpenAI is not None and hasattr(self.client, "chat"):
                        resp = self.client.chat.completions.create(
                            model=self.model,
                            messages=messages,
                            temperature=0.1,
                            max_tokens=200
                        )
                        content = resp.choices[0].message.content
                    else:
                        # 兼容旧版 openai.ChatCompletion
                        resp = openai_module.ChatCompletion.create(
                            model=self.model,
                            messages=messages,
                            temperature=0.1,
                            max_tokens=200
                        )
                        content = resp.choices[0].message['content'] if isinstance(resp.choices[0].message, dict) else resp.choices[0].message.content

                except Exception as e:
                    print(f"⚠️ openai SDK 调用失败: {e}")
                    raise

            else:
                # Ollama / HTTP fallback: use session to post
                url = self.base_url.rstrip('/') + '/chat/completions'
                payload = {
                    'model': self.model,
                    'messages': messages,
                    'temperature': 0.1,
                    'max_tokens': 200
                }
                try:
                    r = self.session.post(url, json=payload, timeout=self.request_timeout)
                    r.raise_for_status()
                    j = r.json()
                except Exception as e:
                    print(f"⚠️ Ollama/HTTP 请求失败: {e}")
                    raise

                # 尝试解析常见返回格式
                if isinstance(j, dict) and 'choices' in j and len(j['choices']) > 0:
                    choice = j['choices'][0]
                    # 支持 {message: {content: ...}} 或直接 text
                    if isinstance(choice, dict) and 'message' in choice and isinstance(choice['message'], dict):
                        content = choice['message'].get('content', '')
                    else:
                        content = choice.get('text', '') or choice.get('content', '')

            if not content:
                raise ValueError('empty content from LLM')

            # clean code fences and try to extract JSON robustly
            content = content.replace("```json", "").replace("```", "").strip()
            result = None
            # try robust extractor
            try:
                result = self._extract_json_from_text(content)
            except Exception:
                result = None
            if result is None:
                # fallback: treat entire text as chat answer
                result = {"action": "CHAT", "answer": content}

            if isinstance(result, dict):
                return normalize_intent(result, fallback_text=content)
            return {"action": "CHAT", "answer": content}


        except Exception as e:
            print(f"⚠️ LLM 推理失败: {e}")
            return {"action": "CHAT", "answer": "我暂时无法思考，请再说一次。"}

