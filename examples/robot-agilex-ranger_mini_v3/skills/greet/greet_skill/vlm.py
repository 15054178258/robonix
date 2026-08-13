# SPDX-License-Identifier: Apache-2.0
"""VLM second stage of the greeting pipeline.

YOLO has already confirmed people are in front of the robot and counted them
(see detector.py). This module is only reached on that positive — its single
job is to turn the current frame + head count into ONE short, customised
spoken greeting. Backed by an OpenAI-compatible endpoint (ofox + a cheap Qwen
vision model). It must never block the loop on failure, so any error falls
back to a fixed template.
"""
from __future__ import annotations

import base64
import logging

import requests

log = logging.getLogger("greet_skill")


def fallback_greeting(count: int) -> str:
    n = count if count > 0 else 1
    return f"前面的{n}位朋友,麻烦让一让,机器人要通过啦,谢谢。"


_SYSTEM = (
    "你是一台移动机器人的语音助手,人设是个热情、呆萌、爱自嘲的\"打工机器人\"。"
    "前置检测器已确认前方有人。看这张机器人正前方的画面,生成一句让机器人用扬声器"
    "喊出来的中文,礼貌又风趣地请大家让条道、让机器人通过。\n"
    "要求:\n"
    "- 口语、简短(一句话,不超过 30 字),活泼有梗、能让人会心一笑;\n"
    "- 核心意思是\"请各位让一让,机器人要通过\";\n"
    "- 一定要结合画面里实际看到的人灵活定制——人数、穿着、性别、神态、疑似身份"
    "(学生/老师/保安/小孩),对不同的人换不同的调侃口吻(对学生俏皮、对老师客气、"
    "对小朋友卖萌);\n"
    "- 可适度自嘲(小身板、搬砖、打工、求生欲)、适度玩梗,但别低俗、别冒犯、别尬;\n"
    "- 绝对不要用波浪号 ～ 或 ~,也不要用省略号、颜文字等特殊符号 —— TTS 会把它"
    "们吞掉导致前后词连读;只用逗号、句号、感叹号这类正常标点;\n"
    "- 只输出这句话本身,不要引号、不要解释、不要任何多余文字。\n"
    "语气示范(只示范风格,别照抄):\n"
    "- \"让一让让一让,打工机器人这就过去,谢谢各位老板!\"\n"
    "- \"前面这位穿白大褂的老师好,麻烦赏个脸让条道,机器人给您鞠躬啦。\"\n"
    "- \"三位小同学借过一下下,我这小身板可不敢撞你们,哈哈。\"\n"
    "- \"滴滴滴,机器人载着满满的求生欲路过,劳驾让一让。\""
)


class VlmGreeter:
    def __init__(self, base_url: str, api_key: str, model: str, timeout_s: float = 20.0):
        self.url = base_url.rstrip("/") + "/chat/completions"
        self.api_key = api_key
        self.model = model
        self.timeout_s = timeout_s

    def greet(self, jpeg: bytes, count: int) -> str:
        """One customised spoken line for the `count` people in `jpeg`.

        Never raises: on any transport / parse error returns a fixed template
        so the robot still says something rather than going silent."""
        b64 = base64.b64encode(jpeg).decode("ascii")
        user_text = f"画面里大约有 {count} 个人。请生成一句友好风趣的避让提醒。"
        payload = {
            "model": self.model,
            # doubao-seed-1-6-vision is a REASONING model: it spends tokens
            # "thinking" before the answer. A small cap gets eaten by reasoning
            # and leaves the spoken line empty → we'd fall back to the template.
            # Give it room for thinking + a short line.
            "max_tokens": 512,
            "temperature": 0.9,
            "messages": [
                {"role": "system", "content": _SYSTEM},
                {"role": "user", "content": [
                    {"type": "text", "text": user_text},
                    {"type": "image_url",
                     "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                ]},
            ],
        }
        try:
            r = requests.post(
                self.url,
                headers={"Authorization": f"Bearer {self.api_key}"},
                json=payload,
                timeout=self.timeout_s,
            )
            r.raise_for_status()
            line = r.json()["choices"][0]["message"]["content"].strip()
            line = line.strip().strip('"').strip("「」").strip()
            return line or fallback_greeting(count)
        except Exception as e:  # noqa: BLE001
            log.warning("[vlm] greet failed (%s); using template", e)
            return fallback_greeting(count)
