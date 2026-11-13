## 11. 核对记录：本次逐篇复核中发现的字段修正

> 一句话导读：逐篇画图的过程等于把 54 条文献重读了一遍，确实翻出**若干与综述不一致的地方**。结论以这里为准，综述那几处需要回改。**没有一条是「推测补全」**——每一条都写了证据来源。

### 11.1 需要回改综述的 3 处

| # | 位置 | 综述原文 | 本次核对结果 | 建议动作 |
|---|---|---|---|---|
| 1 | §3.7 NavDP | 「363.2 km / 1244 场景」 | arXiv **v3**（2025-12-24）摘要原文为 `over one million meters of navigation experience across 3,000 scenes`；v1 提交于 2025-05-13，v2 2025-05-15，v3 2025-12-24 | 按「超过 100 万米 / 3000 场景」引用；若坚持旧数字须注明来源版本，否则属数字不一致 |
| 2 | §3.7 MuJoCo Playground | 「`RSS 2025` Outstanding Demo Paper」＋「Go2 locomotion + ONNX 导出/部署参考」 | arXiv 该条**无 Comments 字段**；GitHub README 可印证属 RSS 2025，但**「Outstanding Demo Paper」这一奖项等级未核实**；且两份 README 里**都没有 ONNX 导出流程**；仓库 2026 已重写为 PyTorch 实现，论文版退到 `rss2025/` 目录 | 奖项降级为「RSS 2025（奖项等级未核实）」；「ONNX 导出参考」这条用途删掉或标未核实 |
| 3 | §3.7 SQUID | 综述列入「venue 未核实」并行 | arXiv `2605.13665` **本次抓取成功**：2026-05-13 提交，George Mason University；SQUID 全称 **S**kill-fused **Q**uadrupedal locomotion **U**sing **I**mitation and **D**istillation；四类隧道截面为 Triangle / Circle / Half-Circle / Gap；专家观测 108 维地形网格、12 维关节目标经 PD；DAgger + 隧道类别标签查询匹配专家 + 类别均衡回放；学生输入为深度图 + 本体历史 | venue / 代码 / 实机型号仍「未核实」，但摘要级事实可升级为已核实 |

### 11.2 需要守住的 4 处（核对后确认「确实不能写实」）

| # | 条目 | 状态 | 说明 |
|---|---|---|---|
| 4 | CERBERUS Lessons Learned 2024 的 arXiv 编号 | **未核实** | 二手镜像给 `2207.04914`，arXiv 官方 abs 页未核实到；本文档与综述都不写编号 |
| 5 | 《基于多模态信息融合的四足机器人避障方法》（工程科学学报 2024） | **摘要未核实** | 期刊站点有安全验证（反爬），只核到标题与 DOI 指向该刊同名文章；本文档 §8.8 的流程图是**据综述描述重建**，引用前必须自行下载原文核对 |
| 6 | 《基于深度强化学习的四足机器人决策算法研究》（张苏晨 硕士论文 2024） | **正文未核实** | 仅高校图书馆检索页；只当线索 |
| 7 | NavFoM / Parkour in the Wild 的会议录用信息 | **未核实** | arXiv 页面无 venue 字段 |

### 11.3 工程字段核对（与本仓库代码对齐）

流程图里的「赛场映射」段引用了下面这些**本仓库真实存在**的配置键，已逐个在代码里核实，不是编的：

| 引用处 | 字段 | 实际位置 | 值 |
|---|---|---|---|
| §7.1 §7.3 | `lidar.range_noise_m` | `lite2_sim/config/randomization/sim2real_v1.json` | 区间 `[0.0, 0.03]`（综述与本文引用的是**上限** 0.03） |
| §7.1 §7.3 | `lidar.dropout_probability` | 同上 | 区间 `[0.0, 0.08]`（上限 0.08） |
| §7.3 | `lidar.beam_block_probability` | 同上 | 区间 `[0.0, 0.05]` |
| §7.1 | 参数来源标签 | `randomization.py` / `sim2real_v1.json` | `sim_assumed` / `vendor_reference` / `measured_pending` 三档 |

> 注意：`sim2real_v1.json` 里 `lidar.dropout_probability` 的 provenance 标的是 **`measured_pending`**，也就是说这个数**还没有实测回填**。写报告书时不能把它写成「实测值」，只能写「仿真假设区间」。

### 11.4 只改了渲染、没改语义的 1 处

§4.1 的流程图节点原写作「障碍高度达腿长 33%」，因 Mermaid 标签里 `%` 存在渲染风险，图表内改为「腿长的三分之一」；正文说明保持「33% 腿长」原样。**数值不变，仅图表写法调整。**

---

## 附录 A：篇目总表（54 篇）

| 节 | 篇目 | 类型 | 本队用途 |
|---|---|---|---|
| 2.1 | Miki 2022 Science Robotics | 感知融合 | 降级语义 |
| 2.2 | Hoeller 2024 ANYmal Parkour | 分层导航 | 技能库组织 |
| 2.3 | Miki 2024 ICRA 3D confined | 限高 | 3D 表征 + 6D 指令 |
| 2.4 | Zhuang 2023 Robot Parkour | 技能学习 | crawl / squeeze |
| 2.5 | Buchanan 2021 JFR 全身规划 | 全身规划 | 通行/放弃判据 |
| 2.6 | Kulathunga 2025 RTEB | 局部规划 | **start_blocked 解法** |
| 3.1–3.5 | A* / Dijkstra / RRT* / Frontier / DWA | 经典 | 全局与降级基线 |
| 3.6 | Rudin 2021 legged_gym | 训练基座 | 域随机化来源 |
| 3.7–3.9 | Lee 2020 / RMA / Agarwal 2022 | 课件基线 | 报告书理论依据 |
| 4.1–4.8 | 落脚点 / 地形 / 限高 / 承载 | 感知地形 | 限高与落足 |
| 5.1–5.7 | Super Odometry / CompSLAM / FAST-LIO / LIO-SAM / LOAM / ORB-SLAM3 / RTAB-Map | 定位 | 只取退化语义 |
| 6.1–6.3 | TEB 三代 | 局部规划 | 多拓扑 / 稀疏优化 |
| 6.4–6.9 | MPPI / CBF-QP / Wei 综述 / iPlanner / ViPlanner / Dai CBF | 规划替代与安全层 | 选型边界 |
| 7.1–7.6 | CERBERUS ×3 / GBPlanner / CSIRO / USAR 综述 | 系统级 | 恢复与放弃语义 |
| 8.1–8.9 | VLA / 仿真基座 / 国内资料 | 前沿与引用 | 趋势 + 国内直连 |

> 说明：§2 与 §3.2 中 Miki 2022 重复出现，总表按一篇计。合计 6 + 9 + 8 + 7 + 9 + 6 + 9 = 54。

## 附录 B：这份文档是怎么做出来的（可复核）

1. **字段底座**：全部作者/年份/venue/arXiv/DOI 抄自 `plan/四足机器人导航避障文献综述与选型建议.md`，不新增字段。
2. **逐篇核对**：有 arXiv 编号的篇目逐条抓 `https://arxiv.org/abs/<id>` 的 abstract、Comments、Journal reference 核对；抓不到的（国内期刊反爬、硕士论文）标「未核实」而不是补全。
3. **机器校验**（两个脚本都在 `tmp/` 下，可重跑）：
   - `tmp/validate_mermaid.py`：检查 59 张流程图的渲染风险——首行格式、`subgraph`/`end` 配对、**标签是否全部加引号**、标签内是否含 `; # % |`、边标签是否加引号、是否有 `style`/`classDef` 等禁用关键字。当前 **59 图 0 错误**。
   - `tmp/crosscheck_citations.py`：把全部分片里的 arXiv 编号与 DOI 抽出来，与综述逐条比对，任何「综述里没有的编号」都会被报出来（当前唯一 1 条 `2403.06341` 经查是综述第 135 行已出现的 RTAB-Map 编号，属正常）。
4. **人工复核**：章节编号 2.1→8.9 连续、每篇四块（一句话 / 出处 / 流程图 / 图里读什么 + 赛场映射 + 边界）齐全、每篇至少一条失败或退化分支。

> 复核命令（Windows / PowerShell，本机已装 python）：
> ```powershell
> python F:\airace\tmp\validate_mermaid.py F:\airace\tmp\flownotes\*.md
> python F:\airace\tmp\crosscheck_citations.py
> ```