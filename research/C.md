# 方向 C：四足 / 腿足机器人导航前沿论文（2023–2025 优先）

> 说明：本文件由 Track C 负责，收录 4 篇 2023–2025 年"顶会/顶刊级"四足（腿足）机器人导航前沿论文，另附 8 篇备选池。
> **凡 DOI、年份、作者、卷期均经 arXiv API 与 Crossref DOI 元数据逐条比对；`web_search` 端点本次鉴权失败（HTTP 401），故改用 `web_fetch` 直接查询 arXiv API / Crossref API / 官方项目页完成检索与核实（核实日期见文件末尾）。** 未能核实项一律标注"待核实"，不做猜测填充。

覆盖的前沿子方向：

| 编号 | 子方向 | 平台 |
|---|---|---|
| 1 | 低视角 VLN（Vision-Language Navigation）for 四足 + sim2real | 四足 |
| 2 | VLM 常识推理做高层规划 + 技能库（复杂地形/死胡同） | Go1 四足 |
| 3 | Extreme parkour / 敏捷极限地形（端到端 RL） | 小型四足 |
| 4 | 物理具身导航 benchmark（腿足 vs 人形 vs 轮式） | 四足/人形/轮式 |

---

## 1. Ground-Level Viewpoint Vision-and-Language Navigation in Continuous Environments（GVNav）

**英文原题**：Ground-Level Viewpoint Vision-and-Language Navigation in Continuous Environments

**第一作者 / 团队**：Zerui Li（第一作者），Australian Institute for Machine Learning (AIML), The University of Adelaide；合作者 Gengze Zhou、Wenqi Lyu、Yanyuan Qiao，通讯团队为 Qi Wu 组（另含 University of Queensland、浙江工业大学合作者）。

**Venue + 年份**：IEEE ICRA 2025（2025 IEEE International Conference on Robotics and Automation），2025。
**DOI / arXiv**：DOI = `10.1109/ICRA55743.2025.11127275`；arXiv:2502.19024（arXiv 备注 "Accepted by ICRA 2025"）。
**核验状态**：✅ DOI 与会议名、作者顺序已用 Crossref 记录核对。

**方案核心机制**：
- 建立在 VLN-CE（continuous environments）的 **waypoint predictor + 局部规划**框架上：单目 RGB 历史观测 → 预测候选 waypoint → 由底层控制器执行。
- 核心问题是**观测高度不一致**：人类视角指令/训练数据 vs 四足机器人贴地相机（低视角、视野易被家具/障碍遮挡），造成训练—部署分布偏移。
- 提出 **Ground-level Viewpoint Navigation（GVNav）**：把**加权历史观测**作为增强的时空上下文，对落在同一 cell 内的相同特征按视角/高度赋不同权重，缓解"特征碰撞"（feature collisions）；从而让低视角机器人克服遮挡与感知错配。
- 引入 **HM3D / Gibson 的连通图（connectivity graph）作为额外空间先验**迁移进来，提升 waypoint predictor 在真实环境中的泛化。
- 在仿真与真实四足机器人上均做了部署验证。

**关键创新**：
1. 首次系统性地指出并量化 **VLN 跨"视觉观测高度"的泛化 gap**，把"腿足机器人贴地视角"当成一等公民问题，而不是当成域随机化细节。
2. 用轻量的"历史观测加权 + 外部场景连通图先验"就显著缩小该 gap，未依赖更大模型或重训基座。
3. 完整走通 sim（VLN-CE 仿真）→ 真实四足部署链路。

**局限**：
- 底层仍是离散 waypoint 预测 + 预训练连通图先验；在**无先验图的全新户外/非结构化场景**中先验失效，性能回落。
- 评测场景以**室内**为主，未涉及野外、楼梯、非平坦地形；四足运动控制被抽象为 waypoint 跟踪，未与越障能力联合优化。
- 低视角下的严重遮挡（沙发后、桌下）仍难处理，真实实验覆盖场景数量有限。
- 指标沿用 VLN 的 SR/SPL，未体现腿足特有的稳定性、能耗、碰撞代价。

**对本项目的启示**：绝影 Lite3 相机高度明显低于人类视角，直接套用人类视角 VLN 数据会有系统性偏差；训练时值得做**高度条件化 / 视角增广**，并把"机身高度的先验地图"作为可选输入。

---

## 2. Commonsense Reasoning for Legged Robot Adaptation with Vision-Language Models（VLM-PC）

**英文原题**：Commonsense Reasoning for Legged Robot Adaptation with Vision-Language Models

**第一作者 / 团队**：Annie S. Chen（第一作者，Stanford University）；合作者 Alec M. Lessing、Andy Tang、Govind Chada（Stanford），Laura Smith、Sergey Levine、Chelsea Finn（UC Berkeley）。Levine + Finn 联合指导。

**Venue + 年份**：IEEE ICRA 2025，2025。
**DOI / arXiv**：DOI = `10.1109/ICRA55743.2025.11127234`；arXiv:2407.02666。
**核验状态**：✅ DOI、会议名、作者与单位归属已用 Crossref 记录核对（Crossref 单位字段：Stanford / UC Berkeley）。

**方案核心机制**：
- **VLM-Predictive Control（VLM-PC）**：把视觉语言模型放在**高层决策回路**，VLM 观察机器人第一视角图像与任务描述，在**预训练低层技能库**（爬越、匍匐、转向等运动技能）之上选择并组合技能，闭环执行直到任务完成。
- 论文指出两个对"让 VLM 真正管用"最关键的组件：
  1. **in-context adaptation**：把过往交互（尝试了什么、成功/失败、结果如何）写回 VLM 上下文，实现**在线适应**，无需梯度更新；
  2. **多技能前瞻规划 + 重规划**：一次规划多步技能再执行，遇到失败/新情况重规划，而非逐步贪心选动作。
- 评测载体为 **Unitree Go1 四足机器人**的多个真实世界障碍赛道，包含死胡同、需要攀爬与匍匐通过的场景。

**关键创新**：
1. 将 VLM 从"单步动作分类器"提升为**技能级长时程规划器 + 在线经验适应器**，在长尾、模糊、意外场景中替代环境专用工程与人工引导。
2. 用"历史交互 + 多步前瞻"两个机制显著提升 VLM 决策的真实可用性（论文把这二者作为消融核心）。
3. 在真实四足上展示了**跨场景自主**：无需为每个环境重新设计控制器或奖励。

**局限**：
- 依赖**强闭源 VLM + 网络推理**：单次推理延迟高、频率低，只能做"慢回路"高层决策，无法支撑高速动态动作。
- 低层技能库仍需**人工设计/预训练**，技能覆盖不到的动作（如需要精确落脚的大落差）无法由 VLM 凭空生成。
- 无度量地图与几何安全保证：VLM 可能给出物理不可行的技能序列，安全依赖低层控制器的隐式约束。
- 实验规模有限（少量障碍赛道、单机型），缺少统计意义上的大规模重复与跨机型迁移验证。

**对本项目的启示**：这是"**VLM 慢回路 + RL/MPC 快回路**"分层架构最直接的可行性证据；若给绝影 Lite3 做野外/废墟场景，可复用"技能库 + 交互记忆 + 前瞻重规划"三件套，同时保留几何避障层做安全兜底。

---

## 3. Extreme Parkour with Legged Robots

**英文原题**：Extreme Parkour with Legged Robots

**第一作者 / 团队**：Xuxin Cheng（共同第一作者）、Kexin Shi（共同第一作者）、Ananye Agarwal、Deepak Pathak；Carnegie Mellon University（CMU）机器人研究所 / Pathak 组。

**Venue + 年份**：IEEE ICRA 2024，2024（同时为 CoRL 2023 Generalist / Roboletics / Deployable Workshop **Oral**）。
**DOI / arXiv**：DOI = `10.1109/ICRA57147.2024.10610200`；arXiv:2309.14341。
**核验状态**：✅ DOI 与会议名、作者单位（Crossref 记录全部为 Carnegie Mellon University）已核对；CoRL 2023 workshop oral 见官方项目页。

**方案核心机制**：
- **端到端强化学习**：单一神经网络策略直接从**单目前视深度相机**图像（论文强调其低频、抖动、易出伪影）+ 本体感知 + 目标方向，输出关节级动作，不经独立感知/规划/控制模块，也不依赖精确标定。
- 在**仿真中大规模 RL 训练**后迁移到真实**小型低成本四足**（执行器不精确）。项目页给出的关键训练要素包括 **clearance reward（离地/越障间隙奖励）** 与 **direction distillation（方向蒸馏，便于用摇杆控制）**，并展示去掉它们后的失败模式。
- 能力：跳过约 **2 倍身高**的障碍、跨越约 **2 倍体长**的缺口、倒立、在倾斜 ramp 上奔跑，并可泛化到物理属性不同的新障碍赛道，全程无人工遥操作。

**关键创新**：
1. 证明"**低质量单目深度 + 不精确执行器**"这两个常被认为致命的短板，可以被大规模 RL + 精心奖励设计**端到端**吃掉，得到毫米级精度要求的动态行为。
2. 用**单一策略**统一跳高/跳远/倒立/斜坡奔跑等多种极限技能，并具备跨赛道泛化，而非针对固定赛道逐条工程化。
3. 把"极限 parkour"从实验室标定环境推进到**低成本硬件 + 单目深度**的可复制设定（代码开源）。

**局限**：
- 本质是**局部敏捷运动**，没有全局导航与语义目标（无长程规划、无地图、无语言指令），不是完整导航系统。
- 只在**结构化障碍元素**（台阶/缺口/栏杆/斜坡）上验证，不是开放野外；对草地、泥地、动态障碍、极端光照/材质未见评估。
- 依赖仿真中较精确的动力学与深度相机模型，sim2real 仍需系统辨识/真实世界调参；策略对负载变化、硬件磨损的鲁棒性未系统验证。
- 单目深度相机的物理局限（视场窄、反光/透明物体失效）在部署中仍是风险点。

**对本项目的启示**：若赛道包含越障/跳台，优先考虑该文的"**大规模 RL + 间隙奖励 + 方向蒸馏**"配方；工程上建议改成 teacher-student（特权信息→深度蒸馏）以降低 sim2real 成本，并把该策略作为底层"越障技能"接入全局规划层。

---

## 4. Rethinking the Embodied Gap in Vision-and-Language Navigation: A Holistic Study of Physical and Visual Disparities（VLN-PE）

**英文原题**：Rethinking the Embodied Gap in Vision-and-Language Navigation: A Holistic Study of Physical and Visual Disparities

**第一作者 / 团队**：Liuyi Wang（第一作者，Tongji University）；合作者 Xinyuan Xia、Hui Zhao、Hanqing Wang、Tai Wang、Yilun Chen（Shanghai AI Laboratory），Chengju Liu、Qijun Chen（Tongji University），Jiangmiao Pang（Shanghai AI Lab）。同济大学 + 上海人工智能实验室联合。

**Venue + 年份**：IEEE/CVF ICCV 2025（2025 IEEE/CVF International Conference on Computer Vision），2025。
**DOI / arXiv**：DOI = `10.1109/ICCV51701.2025.00882`；arXiv:2507.13019（arXiv 备注 "Accepted by ICCV 2025"）。
**核验状态**：✅ DOI、会议名、作者顺序与单位已用 Crossref 记录核对。

**方案核心机制**：
- 提出 **VLN-PE**：一个**物理真实**的 VLN 评测平台，同时支持 **humanoid、quadruped、wheeled** 三类具身（腿足/人形带真实运动控制与碰撞、跌倒物理）。
- 在同一平台上系统横评三类技术路线：
  1. **分类式**单步离散动作预测；
  2. **扩散式**稠密 waypoint 预测；
  3. **免训练、基于地图的 LLM + 路径规划**。
- 结论性发现：真实物理部署下性能显著下降，退化来源被拆解为**观察空间受限、环境光照变化、碰撞与跌倒等物理挑战**；并暴露腿足机器人在复杂环境中的**运动学/稳定性约束**。
- 平台可扩展：可接入 MP3D 之外的新场景，便于持续评测。

**关键创新**：
1. 首个把"**物理具身差异**"（而非仅视觉域差异）系统纳入 VLN 评测的工作，指出离散动作抽象会**掩盖**碰撞、跌倒、关节限位等失败模式。
2. 跨三类具身、跨三类方法族的统一横评协议，可直接作为"腿足导航策略"的评测基准。
3. 平台可扩展新场景，缓解 VLN 评测长期被 MP3D 单一数据集绑定的问题。

**局限**：
- 仍是**仿真**（物理引擎 + 运动控制器近似），与真实四足的 sim2real 差距（接触摩擦、感知噪声、通信延迟）未完全覆盖。
- 被评测的 VLN 方法本身并非针对腿足控制联合优化，"物理具身"多体现为执行层约束，而非端到端联合学习。
- 场景仍以**室内 MP3D** 为主，缺少野外/户外与长距离（km 级）评测。
- 指标沿用 SR/SPL 等 VLN 指标，未纳入能耗、稳定性、关节负载等腿足特有指标；无真实机器人对照实验。

**对本项目的启示**：可直接借用其"**同一策略 × 三类具身 × 三类方法族**"的评测思路，给自己项目的导航策略建立"物理可行性"评测（碰撞率、跌倒率、恢复时间），避免只在离散 waypoint 抽象下刷 SR。

---

## 备选池（5–8 篇，均已核验标题与 venue）

| # | 英文标题 | Venue / 年份 | DOI 或 arXiv | 备注与核验 |
|---|---|---|---|---|
| A1 | **SayNav: Grounding Large Language Models for Dynamic Planning to Navigation in New Environments** | ICAPS 2024（Proceedings of the International Conference on Automated Planning and Scheduling, Vol. 34） | DOI `10.1609/icaps.v34i1.31506`；arXiv:2309.04077 | SRI International（Rajvanshi 第一作者）。机制：LLM + **增量式 3D scene graph grounding** + 预训练 point-goal 低层规划器，动态逐步生成/修正子目标。⚠️ 实验载体为**轮式地面机器人**（非四足），机制可迁移，不能当作"四足论文"引用。✅ Crossref 已核 |
| A2 | **NaVid: Video-based VLM Plans the Next Step for Vision-and-Language Navigation** | RSS 2024（Robotics: Science and Systems） | arXiv:2402.15852（arXiv 备注 "Accepted by RSS 2024"） | 视频 VLM 端到端输出下一步动作，**不用地图/里程计/深度**，主张规避 odometry 噪声与 map/depth 的 sim2real gap。RSS 2024 论文 DOI 需按 RSS 卷期另行确认（`待核实`），此处仅引 arXiv。 |
| A3 | **Uni-NaVid: A Video-based Vision-Language-Action Model for Unifying Embodied Navigation Tasks** | arXiv 2024（会议 venue `待核实`） | arXiv:2412.06224 | 360 万条导航样本、统一 VLN/ObjectNav/EQA/tracking 的 VLA；PKU-EPIC（Jiazhao Zhang 第一作者）。适合作为"导航基座模型"路线参考。 |
| A4 | **Wild Visual Navigation: Fast Traversability Learning via Pre-Trained Models and Online Self-Supervision** | RSS 2023（原始版标题 *Fast Traversability Estimation for Wild Visual Navigation*）；2024 扩展版为 arXiv 预印本 | arXiv:2305.08510（RSS 2023 版，备注 "Accepted by RSS 2023"）；扩展版 arXiv:2404.07110 | ETH Zurich（Mattamala/Frey/Hutter/Fallon）。野外草地/森林在线自监督可通行性学习，**ANYmal 四足**实机，5 分钟内在线 bootstrap，1.4 km 小径跟随；"野外自主导航 + sim2real 轻量化"最相关备选。✅ arXiv 备注已核 |
| A5 | **ViNT: A Foundation Model for Visual Navigation** | CoRL 2023（Oral） | arXiv:2306.14846（备注 "Accepted for oral presentation at CoRL 2023"） | Berkeley（Dhruv Shah/Sergey Levine）。视觉导航基础模型 + 扩散式 subgoal 提议，km 级导航；跨平台（含四足），可作"导航基座"备选。✅ arXiv 备注已核 |
| A6 | **NavMorph: A Self-Evolving World Model for Vision-and-Language Navigation in Continuous Environments** | ICCV 2025 | DOI `10.1109/ICCV51701.2025.00525` | CASIA（Xuan Yao 第一作者）。**world model** 方向的 VLN 代表，填补本文四篇未覆盖的 world-model 子方向。✅ Crossref 已核 |
| A7 | **QUAR-VLA: Vision-Language-Action Model for Quadruped Robots** | ECCV 2024 | arXiv:2312.14457（备注 "Accepted by ECCV2024"） | 西湖大学等（Pengxiang Ding 第一作者）。四足 VLA + QUARD 多任务数据集（含导航/复杂地形运动/全身操作），4000 次评测；适合做"四足 VLA 数据集"参考。✅ arXiv 备注已核 |
| A8 | **SARO: Space-Aware Robot System for Terrain Crossing via Vision-Language Model** | arXiv 2024（会议 venue `待核实`） | arXiv:2407.16412 | 清华（Shaoting Zhu、Hang Zhao 组）。VLM 高层任务分解 + 闭环子任务执行 + RL 低层控制，跨 3D 地形；与本文第 2 篇构成"VLM 高层 + 腿足"对照。 |

> 备选池使用建议：A1/A2/A3 属"大模型高层规划/导航基座"；A4/A5 属"野外/通用视觉导航 + sim2real"；A6 属"world model"；A7/A8 属"四足 VLA / 地形通过"。若要替换正文任一篇，优先 A4（野外自主导航）或 A1（SayNav 系语义地图 grounding，需注明平台为轮式）。

---

## 横向对比与选型建议（给总体方案用）

| 维度 | GVNav (ICRA'25) | VLM-PC (ICRA'25) | Extreme Parkour (ICRA'24) | VLN-PE (ICCV'25) |
|---|---|---|---|---|
| 类型 | 方法（VLN） | 方法（VLM 分层控制） | 方法（端到端 RL） | 平台/benchmark |
| 感知输入 | 单目 RGB 历史 | 第一视角 RGB | 单目深度 | 仿真多传感器 |
| 高层规划 | waypoint predictor + 图先验 | VLM 技能级前瞻规划 | 无（目标方向向量） | 被评测方法自带 |
| 语言指令 | ✅ | ✅（任务描述） | ❌ | ✅ |
| 真实四足 | ✅ | ✅ Go1 | ✅ 小型四足 | ❌ 纯仿真 |
| 越障/极限地形 | 弱（室内平地为主） | 中（攀爬/匍匐） | 强（2×身高/2×体长） | 中（有跌倒物理） |

**落地建议**：
1. **分层架构最稳**：高层用 GVNav/VLM-PC 式"语言→子目标/技能"，低层用 Extreme Parkour 式 RL 越障技能，中间用几何地图/避障层兜底。
2. **评测先行**：按 VLN-PE 的思路先建"物理可行性"评测（碰撞、跌倒、恢复），再谈 SR。
3. **sim2real 成本最低的组合**：特权信息 teacher + 深度/视觉 student（Extreme Parkour 路线的工程化变体）+ 在线自监督可通行性（A4 WVN）。

---

## 核验记录

- 核验方式：`web_fetch` 直查 **arXiv API**（`export.arxiv.org/api/query`，取标题/作者/摘要/comment 中的 venue 声明）与 **Crossref REST API**（`api.crossref.org/works`，取 DOI、container-title、published date-parts、作者与单位），辅以官方项目页（extreme-parkour.github.io）。
- 核实日期：2026-11-12（Asia/Shanghai）。
- 工具限制：`web_search` 端点返回 HTTP 401（鉴权失败），本次全部改用上述 API 直查完成；因此个别"会议 venue 仅见 arXiv comment 声明"的条目（A2、A7 等）以 arXiv comment 为据，并在表中标注是否经 Crossref 复核。
- 所有正文 4 篇的 DOI 均来自 Crossref 权威记录，可直接在 https://doi.org/ 解析。
- 明确标注 `待核实` 的项：NaVid 的 RSS 2024 正式 DOI、Uni-NaVid 与 SARO 的正式会议 venue。**未做任何猜测性填充。**