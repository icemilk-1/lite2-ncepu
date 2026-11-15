"""实机验收自动化工具：一条命令跑完链路/深度心跳/站趴/使能/低速/急停/复位，
并生成可归档的 JSON 记录。

安全设计：
* 默认 **只读**（只查健康与链路，不发任何运动/姿态/急停指令）。
* 需要运动验收时必须同时给出 ``--allow-motion`` 与
  ``--confirm "现场有人监护"``，且速度默认 0.05 m/s。
* 随时可以用物理急停/遥控器接管；工具自身在异常路径会发 stop。

用法：
    python host_ui/tools/real_robot_acceptance.py            # 只读体检
    python host_ui/tools/real_robot_acceptance.py --allow-motion \
        --confirm "现场有人监护" --speed 0.05
    # 指定地址/令牌：
    python host_ui/tools/real_robot_acceptance.py --base http://127.0.0.1:8710 \
        --token xxx --out tmp/host_ui_real_acceptance.json
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

import httpx
import websockets

CONFIRM_TEXT = "现场有人监护"
# 20 s 姿态动作 + 5 s 机载回执宽限，再留轮询和链路调度余量。
POSTURE_CONFIRM_TIMEOUT_S = 32.0


class Acceptance:
    def __init__(self, base: str, token: str = "") -> None:
        self.base = base.rstrip("/")
        self.token = token
        self.checks: list[dict] = []
        self.started = time.time()

    # ------------------------------------------------------------------ 工具
    def _headers(self) -> dict:
        return {"X-Auth-Token": self.token} if self.token else {}

    def status(self) -> dict:
        with httpx.Client(timeout=8, trust_env=False) as client:
            response = client.get(f"{self.base}/api/status",
                                  headers=self._headers())
            response.raise_for_status()
            return response.json()

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.checks.append({"name": name, "ok": bool(ok), "detail": detail,
                            "at": round(time.time(), 3)})
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}".rstrip())
        return ok

    # ------------------------------------------------------------------ 只读
    def read_only(self) -> dict:
        started = self.status()
        time.sleep(1.2)
        later = self.status()
        link = later.get("link") or {}
        kind = link.get("kind")
        frames_before = (started.get("counters") or {}).get("frames", 0)
        frames_after = (later.get("counters") or {}).get("frames", 0)
        age = link.get("last_frame_age_s")
        health = later.get("health") or {}
        frame = later.get("frame") or {}
        link_ok = bool(link.get("connected"))
        self.check("链路连接", link_ok, f"kind={kind} detail={link.get('detail')}")
        if kind == "mock":
            self.check("状态帧增长", frames_after > frames_before,
                       f"{frames_before} -> {frames_after}")
            self.check("深度心跳", True, "跳过（mock 无深度）")
        else:
            self.check("状态帧增长", frames_after > frames_before,
                       f"{frames_before} -> {frames_after}")
            self.check("状态帧龄", age is not None and age <= 1.0,
                       f"age={age} s")
            depth = health.get("depth_age_s")
            self.check("深度心跳 ≤0.5s", depth is not None and depth <= 0.5,
                       f"depth_age={depth} s（若失败：检查相机/深度驱动与 clock 源）")
            self.check("执行门禁未报 stale_depth",
                       bool(health.get("bridge_ok")) and bool(health.get("execution_reason"))
                       and health.get("execution_reason") != "stale_depth",
                       f"reason={health.get('execution_reason')}")
        battery = later.get("battery") or {}
        self.check("电量", battery.get("percent") is not None,
                   f"{battery.get('percent')}% low={battery.get('low')}")
        self.check("basic 状态可读", frame.get("basic") is not None,
                   f"basic={frame.get('basic')} mode_ready={frame.get('mode_ready')}")
        self.check("急停未锁存", not (later.get("safety") or {}).get("estop"),
                   f"estop={(later.get('safety') or {}).get('estop')}")
        system = health.get("versions") or {}
        self.check("版本信息", bool(system.get("host_ui")),
                   json.dumps(system, ensure_ascii=False))
        return {
            "link": {"kind": kind, "frames": frames_after, "age_s": age},
            "health": health,
            "battery": battery,
            "basic": frame.get("basic"),
        }

    # ------------------------------------------------------------------ 运动
    async def motion(self, speed: float) -> dict:
        url = self.base.replace("http://", "ws://").replace("https://", "wss://")
        ws_url = url + "/ws" + (f"?token={self.token}" if self.token else "")
        record: dict = {}
        request_id = 0
        async with websockets.connect(ws_url, max_size=32 * 1024 * 1024) as ws:
            async def recv_until(predicate, timeout=12.0):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    raw = await asyncio.wait_for(ws.recv(), timeout=timeout)
                    message = json.loads(raw)
                    if predicate(message):
                        return message
                raise AssertionError("等待消息超时")

            async def request(payload: dict, timeout: float = 15.0) -> dict:
                nonlocal request_id
                request_id += 1
                payload = dict(payload, id=request_id)
                await ws.send(json.dumps(payload))
                return await recv_until(
                    lambda m: m.get("type") == "ack"
                    and m.get("id") == request_id, timeout)

            hello = await recv_until(lambda m: m.get("type") == "hello")
            record["client_id"] = hello["data"]["client_id"]
            ack_message = await request({"type": "claim_control"})
            self.check("取得操控权", ack_message.get("ok"), ack_message.get("message", ""))

            snapshot = self.status()
            if (snapshot.get("safety") or {}).get("estop"):
                self.check("急停锁存，运动验收中止", False,
                           "先人工确认安全并长按复位急停，再重新运行运动验收")
                return record
            basic = (snapshot.get("frame") or {}).get("basic")
            if basic == 1:
                ack_message = await request({"type": "posture", "action": "stand"},
                                            POSTURE_CONFIRM_TIMEOUT_S)
                self.check("起立请求", ack_message.get("ok"), ack_message.get("message", ""))
                deadline = time.monotonic() + POSTURE_CONFIRM_TIMEOUT_S
                while time.monotonic() < deadline:
                    snapshot = self.status()
                    if (snapshot.get("posture") or {}).get("state") == "ok" and \
                            (snapshot.get("frame") or {}).get("basic") == 6:
                        break
                    await asyncio.sleep(0.3)
                basic = (snapshot.get("frame") or {}).get("basic")
                self.check("起立确认 basic=6", basic == 6, f"basic={basic}")
                record["posture_stand"] = snapshot.get("posture")
            elif basic == 6:
                self.check("起立确认 basic=6", True, "已站立")

            ack_message = await request({"type": "arm"}, POSTURE_CONFIRM_TIMEOUT_S)
            self.check("使能请求", ack_message.get("ok"), ack_message.get("message", ""))
            deadline = time.monotonic() + POSTURE_CONFIRM_TIMEOUT_S
            armed = False
            while time.monotonic() < deadline:
                snapshot = self.status()
                armed = (snapshot.get("health") or {}).get("armed") is True
                if armed:
                    break
                await asyncio.sleep(0.3)
            self.check("自动模式确认", armed, json.dumps(snapshot.get("execution"), ensure_ascii=False))
            record["execution"] = snapshot.get("execution")
            if not armed:
                await ws.send(json.dumps({"type": "stop"}))
                return record

            before = self.status()
            odometer_before = (before.get("counters") or {}).get("odometer_m", 0.0)
            pose_before = before.get("pose")
            duration = 2.0
            end = time.monotonic() + duration
            while time.monotonic() < end:
                await ws.send(json.dumps({"type": "teleop", "linear": speed,
                                          "angular": 0.0, "deadman": True}))
                await asyncio.sleep(0.05)
            await ws.send(json.dumps({"type": "stop"}))
            stop_pose = self.status().get("pose")
            await asyncio.sleep(0.8)
            settled = self.status()
            odometer_after = (settled.get("counters") or {}).get("odometer_m", 0.0)
            pose_after = settled.get("pose")
            moved = None
            if pose_before and pose_after:
                moved = ((pose_after[0] - pose_before[0]) ** 2
                         + (pose_after[1] - pose_before[1]) ** 2) ** 0.5
            self.check("低速运动", moved is not None and moved > 0.01,
                       f"位移 {moved if moved is None else round(moved, 3)} m"
                       f"（里程计 +{odometer_after - odometer_before:.3f} m）")
            output = (settled.get("safety") or {}).get("output") or {}
            self.check("停车后零速", abs(output.get("linear", 0)) < 1e-6,
                       f"output={output}")
            stop_distance = None
            if stop_pose and pose_after:
                stop_distance = ((pose_after[0] - stop_pose[0]) ** 2
                                 + (pose_after[1] - stop_pose[1]) ** 2) ** 0.5
                self.check("停车距离记录", True, f"{stop_distance:.3f} m（判据待现场标定）")
            record.update({"speed_m_s": speed, "duration_s": duration,
                           "displacement_m": (None if moved is None
                                              else round(moved, 3)),
                           "odometer_delta_m": round(odometer_after - odometer_before, 3),
                           "stop_distance_m": (None if stop_distance is None
                                               else round(stop_distance, 3)),
                           "pose_before": pose_before, "pose_after": pose_after})

            # 急停 / 复位
            ack_message = await request({"type": "estop", "engaged": True,
                                         "reason": "验收脚本急停"})
            self.check("软件急停", ack_message.get("ok"), ack_message.get("message", ""))
            deadline = time.monotonic() + 3
            estop_state = {}
            while time.monotonic() < deadline:
                snapshot = self.status()
                estop_state = snapshot.get("safety") or {}
                if estop_state.get("estop"):
                    break
                await asyncio.sleep(0.1)
            self.check("急停锁存确认", bool(estop_state.get("estop")),
                       json.dumps(estop_state.get("output"), ensure_ascii=False))
            record["estop"] = {"reason": (snapshot.get("health") or {}).get("execution_reason"),
                               "output": estop_state.get("output")}
            ack_message = await request({"type": "estop_reset", "hold_s": 2.0})
            self.check("急停复位", ack_message.get("ok"), ack_message.get("message", ""))
            ack_message = await request({"type": "disarm"})
            self.check("解除使能", ack_message.get("ok"), ack_message.get("message", ""))
        return record

    # ------------------------------------------------------------------ 汇总
    def finish(self, extra: dict, output: Path) -> int:
        failed = [check for check in self.checks if not check["ok"]]
        report = {
            "started_at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.started)),
            "base": self.base,
            "checks": self.checks,
            "failed": len(failed),
            "passed": len(self.checks) - len(failed),
            **extra,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                          encoding="utf-8")
        print(f"\n结果：{report['passed']} 通过 / {report['failed']} 失败")
        print(f"记录：{output}")
        return 0 if not failed else 1


async def run(args) -> int:
    acceptance = Acceptance(args.base, args.token)
    read_only = acceptance.read_only()
    extra: dict = {"read_only": read_only}
    if args.allow_motion:
        if args.confirm != CONFIRM_TEXT:
            print(f"拒绝执行运动验收：必须同时传入 --allow-motion 与 "
                  f'--confirm "{CONFIRM_TEXT}"')
            return 2
        extra["motion"] = await acceptance.motion(args.speed)
    output = Path(args.out or
                  f"tmp/host_ui_real_acceptance_{time.strftime('%Y%m%d_%H%M%S')}.json")
    return acceptance.finish(extra, output)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Lite2 实机验收自动化")
    parser.add_argument("--base", default="http://127.0.0.1:8710")
    parser.add_argument("--token", default="")
    parser.add_argument("--out", default=None)
    parser.add_argument("--allow-motion", action="store_true",
                        help="允许站/趴/使能/低速运动/急停（需现场监护）")
    parser.add_argument("--confirm", default="",
                        help=f'现场确认文本，必须是 "{CONFIRM_TEXT}"')
    parser.add_argument("--speed", type=float, default=0.05,
                        help="运动验收线速度（默认 0.05 m/s）")
    return parser


if __name__ == "__main__":
    sys.exit(asyncio.run(run(build_parser().parse_args())))
