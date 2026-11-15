# 常驻待命心跳

机载桥启动后自动进入未使能、零速待命并立即发送 5 Hz 厂商心跳。
普通网页关闭、SSH stream 退出和 stream 替换调用 `/airace/detach`，清空运动及
待处理动作，保留待命心跳。进程被强杀时由原有 0.35 秒运动门禁停车；systemd
负责重启桥，重启后不继承使能和运动指令。

显式 `/airace/release` 交还 App；急停或实体遥控接管也停止发送厂商心跳及速度包。
该状态原子保存至用户的 `~/.local/state/airace/control_owner.json`，所以服务重启
不会抢回控制权。可通过 `AIRACE_CONTROL_OWNER_FILE` 改变路径。文件损坏时启动失败，
不擅自抢占。现场恢复到正常趴下或站立、遥控摇杆回中、确认 App 已退出后，显式执行：

```bash
rosservice call /airace/claim
```

此命令只恢复待命心跳，保持未使能，不发送站趴或自动模式指令。自动重连仅调用
`/airace/acquire`，不能解除交还、急停或接管锁存。

## 部署

先将本目录的 `lite2_live.py`、`bridge_runtime.py`、`vendor_protocol.py`、
`execution_guard.py`、`run_bridge_service.sh` 及原有共享导航内核复制到部署目录。
部署桥和主机 stream 必须同一批更新，旧桥没有 `/airace/detach`。
以下模板对应当前感知主机用户及路径；改变部署路径时同步修改 unit 的两个路径。
确认实际平稳趴下、未使能、App 已退出后，由唯一控制方执行服务切换：

```bash
sudo install -m 644 /home/ysc/airace_deployment_20261114/hardware/airace-vendor-bridge.service /etc/systemd/system/airace-vendor-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable airace-vendor-bridge.service
sudo systemctl start airace-vendor-bridge.service
systemctl status airace-vendor-bridge.service --no-pager
journalctl -u airace-vendor-bridge.service -n 50 --no-pager
```

安装前须停止旧的**本项目桥进程**；两个 UDP 端口在固定 ROS 节点注册前独占绑定，
重复桥启动会失败，既有节点不会因此被 ROS 踢下线。不使用模糊进程清理命令，
不重启运动固件服务。现有未受监督桥不得和 systemd 实例同时运行。

## 已覆盖与仍需实机验证

离线测试覆盖立即待命、65 秒持续心跳、断开不释放、重连不继承使能、明确交还后
无后续速度包、交还锁存跨重启、正常显式恢复、UDP 端口冲突及取消竞态。
现有反馈新鲜度、深度、姿态和遥控门禁继续约束运动。

当前最小版本在 ROS 初始化完成后启动心跳。systemd 从开机独立启动并在 ROS 尚未
可用时重试，但如果 ROS/网络就绪晚于固件通信保护时限，仍可能在首个心跳前触发
保护；这需要跨主机启动时间实测，或后续将纯 UDP 心跳下沉到不依赖 ROS 的进程。
已经锁存的固件保护不会被此改动自动清除；历史成功恢复为保持心跳时重启运动服务，
该恢复须单独确认实际姿态后执行，本服务不会自动执行。
