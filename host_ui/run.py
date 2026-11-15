"""上位机启动入口。

用法（在仓库根目录 F:\\airace 下）::

    python host_ui/run.py                       # 离线演示模式，浏览器打开 http://127.0.0.1:8710
    python host_ui/run.py --link udp --ip 192.168.1.120
    python host_ui/run.py --link ros             # 需要机载/开发机已 source ROS1 环境
    python host_ui/run.py --host 0.0.0.0 --port 8710 --open
    python host_ui/run.py --map host_ui/maps/lab.json

实机部署时建议先用 ``--link mock`` 确认界面与急停链路，再切到 ``udp``。
"""
from __future__ import annotations

import argparse
import secrets
import sys
import threading
import webbrowser
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import uvicorn  # noqa: E402

from host_ui.app.config import Config  # noqa: E402
from host_ui.app.server import create_app  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Lite2 机器狗实机调试上位机（浏览器版）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--host", default=None, help="监听地址，默认取配置 web.host")
    parser.add_argument("--port", type=int, default=None, help="监听端口，默认取配置 web.port")
    parser.add_argument("--config", default=None,
                        help="配置文件路径（JSON）；缺省自动使用 host_ui/config.json（若存在）")
    parser.add_argument("--link", default=None,
                        choices=("mock", "udp", "ros", "ssh_ros"),
                        help="启动时连接的链路类型")
    parser.add_argument("--token", default=None,
                        help="访问令牌；绑定非回环地址时必填（不填会自动生成并打印）")
    parser.add_argument("--ip", default=None, help="厂商 UDP：机器人 IP")
    parser.add_argument("--robot-port", type=int, default=None,
                        help="厂商 UDP：机器人速度指令端口")
    parser.add_argument("--bind-port", type=int, default=None,
                        help="厂商 UDP：本机接收状态包端口")
    parser.add_argument("--map", default=None, help="启动时加载的地图（json/yaml/pgm）")
    parser.add_argument("--no-connect", action="store_true", help="启动时不自动连接链路")
    parser.add_argument("--no-open", action="store_true", help="不自动打开浏览器")
    parser.add_argument("--log-level", default="info", help="uvicorn 日志级别")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    default_config = REPO_ROOT / "host_ui" / "config.json"
    config_path = args.config or (str(default_config) if default_config.exists() else None)
    config = Config.load(config_path)

    patch: dict = {}
    if args.link:
        patch.setdefault("link", {})["kind"] = args.link
    if args.ip:
        patch.setdefault("link", {}).setdefault("options", {})["robot_ip"] = args.ip
    if args.robot_port:
        patch.setdefault("link", {}).setdefault("options", {})[
            "robot_port"] = args.robot_port
    if args.bind_port:
        patch.setdefault("link", {}).setdefault("options", {})[
            "bind_port"] = args.bind_port
    if args.map:
        patch.setdefault("map", {})["file"] = str(Path(args.map).expanduser())
    if args.host:
        patch.setdefault("web", {})["host"] = args.host
    if args.port:
        patch.setdefault("web", {})["port"] = args.port
    if args.token:
        patch.setdefault("web", {})["token"] = args.token
    if patch:
        config.update(patch)

    host = config.get("web.host", "127.0.0.1")
    port = int(config.get("web.port", 8710))
    token = str(config.get("web.token") or "")
    if host not in ("127.0.0.1", "::1", "localhost") and not token:
        token = secrets.token_urlsafe(18)
        config.update({"web": {"token": token}})
        print(f"[安全] 绑定 {host} 但未配置令牌，已自动生成：{token}")
    url = f"http://{'127.0.0.1' if host in ('0.0.0.0', '::') else host}:{port}/"
    if token:
        url += f"?token={token}"

    print("=" * 68)
    print("Lite2 机器狗实机调试上位机")
    print(f"  界面地址 : {url}")
    if token:
        print(f"  访问令牌 : {token}（也可用 --token 固定，或写入配置 web.token）")
    print(f"  默认链路 : {config.get('link.kind')}")
    print(f"  配置文件 : {config.path or '（仅内存，未持久化）'}")
    print(f"  速度上限 : {config.get('limits.max_forward_m_s')} m/s  "
          f"{config.get('limits.max_yaw_rad_s')} rad/s")
    print("  急停     : 界面红色按钮 / Esc；复位需长按 ≥ "
          f"{config.get('safety.estop_reset_hold_s')} s。")
    print("  注意     : 请同时准备物理急停/遥控器；进程崩溃后的停车仍以实测为准。")
    print("=" * 68)

    app = create_app(config, auto_connect=not args.no_connect)
    if not args.no_open:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        uvicorn.run(app, host=host, port=port, log_level=args.log_level)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())