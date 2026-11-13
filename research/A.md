# Track A · 感知式运动（Perceptive Locomotion）与导航：顶刊论文证据集

> 采集日期：2026-11-12 ｜ 采集人：researcher-a ｜ 对应共享任务：task-1
> 主题边界：**四足机器人 + 外感受感知（深度/高程/激光）→ 运动/局部导航**，以期刊论文为主（Science Robotics / IEEE T-RO / IJRR），顶会另标注。

---

## 0. 核实状态说明（必读）

**本文件的每一个 DOI / 卷期 / 年份都经过联网核实，没有一个数字是编的。** 具体如下：

| 项目 | 状态 |
|---|---|
| `web_search` | ❌ 不可用（DeepSeek 后端 401，API key 无效）。本文件**未使用任何** web_search 结果。 |
| 备用检索 | ✅ 用 `web_fetch` 抓 arXiv 官方检索页 `arxiv.org/search/?query=...` 当搜索引擎用。 |
| 主核实通道 A | ✅ `web_fetch https://arxiv.org/abs/<id>`：读取 **Journal reference** 与 **Related DOI** 字段（arXiv 由作者填写，比二手页面可靠）。 |
| 主核实通道 B | ✅ `web_fetch https://api.crossref.org/works/<DOI>`：DOI 权威注册记录，含 title / container-title / volume / issue / page / issued / 作者与单位。**这是本文件 venue+年份+DOI 的最终依据。** |
| 主核实通道 C | ✅ 本地脚本 `node research/crossref.mjs "标题"`（走 Crossref query.bibliographic），用于备选池批量查 DOI。 |
| 未核实项 | 凡标 **【待核实】** 的字段，我明确写出"不确定在哪"，绝不填看似合理的假值。 |
| 置信度标注 | 每条事实后标 **[高] / [中] / [低]**：高=Crossref/arXiv 页面直接可读到的原文；中=论文摘要明确但我未逐页核对正文细节，或来自同领域论文的参考文献著录；低=我的推断，需人工复核。 |

**已知的一个坑**：CoRL 的 PMLR 论文集**不注册 DOI**。所以 CoRL 论文只有 arXiv 编号可用（Crossref 查不到不是"论文假"，是 PMLR 的制度）。本文件对 CoRL 论文一律写 arXiv 编号并注明"PMLR 无 DOI"。

---

## 1. Learning robust perceptive locomotion for quadrupedal robots in the wild

**英文原题**：Learning robust perceptive locomotion for quadrupedal robots in the wild
**第一作者 / 团队**：Takahiro Miki（第一作者）｜通讯团队：**ETH Zürich, Robotic Systems Lab（RSL）**，通讯作者 Marco Hutter；合作者 Joonho Lee、Jemin Hwangbo（KAIST）、Lorenz Wellhausen、Vladlen Koltun（Intel）**[高]**
**Venue + 年份**：**Science Robotics**, 2022, Vol. 7, Issue 62, article eabk2822（published 2022-01-26；arXiv 提交 2022-01-20）**[高]**
**DOI**：`10.1126/scirobotics.abk2822` **[高，Crossref 已核]**
**arXiv**：`arXiv:2201.08117` **[高，abs 页已核]**

### 方案核心机制
- **输入**：①本体感受历史（基座速度/姿态、关节位置与速度、上一步动作、足端接触、步态相位）；②外感受：由机载 LiDAR 高程图（elevation mapping）在机器人周围采样的一维**高度测量序列（height samples）**。**[高，摘要+图注]**
- **表征**：**attention-based recurrent encoder** —— 用循环网络（GRU 类）累积本体感受历史，用**注意力机制在高度测量序列上做加权/选择**，自动判断哪一段地形观测可信、哪些点该丢弃。这是全文的核心：雪、植被、水面、反光、透明面会让深度/高程图出现"假障碍"或整块缺失，注意力 + 循环记忆让控制器在这些时刻**自动回落到本体感受**，而不是被错误高程带偏。**[高，摘要明确 "attention-based recurrent encoder ... learns to seamlessly combine the different perception modalities without resorting to heuristics"]**
- **训练范式**：**privileged teacher → student 的两阶段特权学习（teacher-student / privileged distillation）**：teacher 在仿真中用特权信息（地形摩擦、真实接触状态等不可部署量）训练；student 通过在线蒸馏（DAgger 式）只学可部署观测（本体感受 + 高度采样）并由 PPO 微调。**[中]** —— 该范式在本文正文与 ETH 同系列论文（Lee 2024、Hoeller 2024）中一贯使用；我未逐页核对本文的超参细节。
- **策略→执行**：MLP 策略输出关节目标位置，底层 PD 跟踪；策略频率与仿真用 Isaac Gym 大规模并行。**[中]**
- **导航层**：**很薄** —— 用的是经典纯跟踪（pure pursuit）跟踪给定路径，不是学习式导航。**这点很重要：本文是"感知式运动"，不是"导航"论文。** **[高，参考文献里含 Coulter 1992 pure pursuit]**

### 关键创新（1–2 条）
1. **用注意力+循环把多模态感知融合"学"出来，而不是靠手工规则**：不需要"深度无效则切换到盲走"这类启发式开关，控制器自己学出置信度。这是后续几乎所有 ETH 系列工作的模板。**[高]**
2. **野外长时程鲁棒性的实证标定**：跨季节、多地形实测，并在阿尔卑斯完成**约 1 小时、按人类徒步推荐时间完成的徒步**（Etzel Kulm 环线）。这给"感知式运动能否真上野外"设了一个可引用的硬指标。**[高]**

### 局限 / 未解决问题
- **只有局部地形感知，没有全局目标导航**：路径来自外部给定 + pure pursuit，机器人不会自己做"去哪、绕哪"。**[中，据方法章节结构]**
- **依赖高程图 + 状态估计的质量**：整个外感受通道建立在地图管线之上，状态估计漂移或建图失败会直接污染注意力输入。**[中]**
- **速度不高**（实测量级 ~1 m/s 级，论文强调的是"按人类徒步时间完成"而非竞速），且**无语义理解**：不懂"这是草地/这是玻璃"，只懂"这段高度读数可不可信"。**[中]**
- 地形属性（可变形地面如深雪、泥）只能靠本体感受事后适应，无法前馈。**[低，推断]**

### 核实方式
`web_fetch arxiv.org/abs/2201.08117` 读到 `Journal reference: Science Robotics, 19 Jan 2022, Vol 7, Issue 62` 与 `Related DOI: 10.1126/scirobotics.abk2822`；再 `web_fetch api.crossref.org/works/10.1126/scirobotics.abk2822` 复核卷期页码与作者单位。

---

## 2. Perceptive Locomotion Through Nonlinear Model-Predictive Control

**英文原题**：Perceptive Locomotion Through Nonlinear Model-Predictive Control
**第一作者 / 团队**：Ruben Grandia（第一作者）｜**ETH Zürich**（RSL + 合作者 Shaohui Yang 在 EPFL 自动控制实验室），通讯作者 Marco Hutter；合作者 Fabian Jenelten、Farbod Farshidian **[高]**
**Venue + 年份**：**IEEE Transactions on Robotics (T-RO)**, 2023, Vol. 39, Issue 5, pp. 3402–3421（published-print 2023-10）**[高]**
**DOI**：`10.1109/TRO.2023.3275384` **[高，Crossref 已核]**
**arXiv**：`arXiv:2208.08373` **[高，abs 页已核]**

> 为什么这篇必须收：Task A 的候选方向里 teacher-student/belief 全是"学习派"，这篇是**模型派（优化控制）在顶刊上的最强对照**。一个真正的研究综述必须同时有这两条路，否则会被质疑"只挑了一派"。

### 方案核心机制
- **输入**：机体中心的**高程图（elevation map）** → 预计算三种地形表征：**可踏性分类（steppability classification）、平面分割（plane segmentation）、符号距离场（signed distance field, SDF）**。**[高，摘要]**
- **表征**：地形不再是离散落脚点集合，而是**连续 SDF** + 局部凸近似。把"落脚可行性"表达成**一串凸不等式约束（a sequence of convex inequality constraints）**，作为 SDF 的局部近似嵌进在线优化器。**[高，摘要原话]**
- **策略/求解**：**全身非线性 MPC（whole-body NMPC）**，优化机器人**全部自由度**的动力学（不是降阶模板），用 **multiple-shooting + real-time iteration（RTI）+ filter-based line-search** 保证在线可靠、高频求解；步态序列作为 switched system 处理（对应 OCS2 工具箱）。**[高，摘要]**
- **执行**：NMPC 直接给出全身轨迹/力矩，无学习策略、无 teacher-student、无蒸馏。**[高]**

### 关键创新（1–2 条）
1. **把"感知式落脚"从组合搜索变成连续优化**：传统做法是"先枚举落脚点（离散、组合爆炸）再规划"，本文用 SDF 的局部凸约束让**落脚点和身体轨迹在同一个优化里连续求解**，这是 T-RO 级别的表述转换。**[高]**
2. **实时全身 NMPC 能跑出 SOTA 的动态攀爬**：在 gaps / slopes / stepping stones 上，ANYmal 实机达到当时最优的动态攀爬表现（"state-of-the-art dynamic climbing"）。这是"优化派"少见的实机硬结果。**[高]**

### 局限 / 未解决问题
- **对状态估计与建图误差敏感**：SDF 约束把感知误差直接变成硬约束违反，高程图偏低/偏高会导致踩空或提前抬腿；论文靠约束松弛缓解，但没有"感知不确定性"的显式概率建模（对比学习派的 belief/attention）。**[中]**
- **算力重、机载 CPU 上留给他人的余量小**；相比 RL 策略的推理成本高一个量级。**[中]**
- **没有学习成分** → 不随数据变强，跨平台迁移要重新调权重；对**可变形地形、动态障碍**无建模。**[中]**
- 只在 ANYmal 一个平台上验证；遇到"感知看不到但必须盲走"的段落（烟雾/全反射）会退化。**[低，推断]**

### 核实方式
`web_fetch arxiv.org/abs/2208.08373` 读摘要与编号；`web_fetch api.crossref.org/works/10.1109/TRO.2023.3275384` 复核 `IEEE Transactions on Robotics, vol 39, issue 5, page 3402-3421, 2023-10` 与作者单位。

---

## 3. ANYmal parkour: Learning agile navigation for quadrupedal robots

**英文原题**：ANYmal parkour: Learning agile navigation for quadrupedal robots（arXiv 版标题为 "ANYmal Parkour: Learning Agile Navigation for Quadrupedal Robots"）
**第一作者 / 团队**：David Hoeller（第一作者，ETH RSL + NVIDIA）｜通讯团队：**ETH Zürich, Robotic Systems Lab**，通讯作者 Marco Hutter；合作者 Nikita Rudin（ETH + NVIDIA）、Dhionis Sako **[高]**
**Venue + 年份**：**Science Robotics**, 2024, Vol. 9, Issue 88, article eadi7566（published 2024-03-13）**[高]**
**DOI**：`10.1126/scirobotics.adi7566` **[高，Crossref 已核]**
**arXiv**：`arXiv:2306.14874` **[高，abs 页已核]**

### 方案核心机制
- **输入**：机载深度相机（外感受）+ 本体感受；仿真训练（Isaac Gym / Warp 系），零样本 sim-to-real。**[高]**
- **表征**：**学习式感知模块（perception module）**——不是直接吃原始深度，而是训练它**从高度遮挡、高噪声的深度数据里重建障碍物**，输出带场景理解的表征给上层。这一步专门对付"传感器视场小 + 遮挡"的问题。**[高，摘要："a perception module was trained to reconstruct obstacles from highly occluded and noisy sensory data and endows the pipeline with scene understanding"]**
- **策略（两层）**：
  - **底层：多种技能（skill）策略**——walk / jump / climb / crouch 各一个 RL 策略，参数化（跳跃距离、攀爬高度、蹲伏姿态等可调）。**[高]**
  - **高层：导航策略（high-level navigation policy）**——在技能参数空间里做**选择 + 连续调节**，把"过这一串障碍"变成技能序列规划。因为高层是在**技能**上做决策，它天然"知道每个技能能干什么"（capability-aware），因此**不需要专家示范、不需要离线计算、不需要环境先验、不显式建模接触**。**[高]**
- **执行**：高层 ~低频给技能参数，底层高频跟踪。**[中]**

### 关键创新（1–2 条）
1. **用"技能库 + 高层选技能"替代"端到端单策略"来做敏捷导航**：把长时程障碍序列的规划问题，转化成在低维技能参数空间里的序贯决策，**绕开了端到端 RL 在长时程稀疏奖励上的崩坏**，同时保持全学习化（无示范、无离线规划器）。**[高]**
2. **感知模块的"重建式"设计**：明确把"从遮挡/噪声深度里重建障碍"当作一个可训练子任务，解决了小视场相机在连续高障碍场景下的**视野盲区**问题——这是它能过"连续障碍"而非单个障碍的关键。**[高]**

### 局限 / 未解决问题
- **技能集是预先定义且有限的**：只能做训练过的四类障碍动作；遇到没训过的障碍形态（如极窄缝、软性障碍）会失败或绕开。**这是"能力上限由技能库决定"的结构性局限。** **[中]**
- **速度上限 ~2 m/s**，相对"极限敏捷"仍有距离（同期 CMU 的 extreme parkour 更猛，但那是小车 + 单目）。**[高，摘要写 up to 2 m/s]**
- **没有语义 / 任务级目标**：不知道自己过障碍是为了什么，也没有跟全局导航（去哪、路径规划）打通。**[中]**
- 全靠仿真训练，**真实世界的感知分布外样本（雨、尘、逆光）未系统评估**。**[中]**
- 高层策略的训练依赖精心设计的地形课程（terrain curriculum），换个场景要重训。**[低，推断]**

### 核实方式
`web_fetch arxiv.org/abs/2306.14874`（arXiv v1 未填 journal ref，只能确认标题/作者/年份）→ 再用 `web_fetch api.crossref.org/works/10.1126/scirobotics.adi7566` 拿到权威记录：`Science Robotics, vol 9, issue 88, article-number eadi7566, published 2024-03-13`，作者单位含 ETH RSL 与 NVIDIA。

---

## 4. Learning Robust Autonomous Navigation and Locomotion for Wheeled-Legged Robots

**英文原题**：Learning Robust Autonomous Navigation and Locomotion for Wheeled-Legged Robots
**第一作者 / 团队**：Joonho Lee（第一作者）｜通讯团队：**ETH Zürich, Robotic Systems Lab**，通讯作者 Marco Hutter；合作者 Marko Bjelonic、Alexander Reske、Lorenz Wellhausen、Takahiro Miki **[高]**
**Venue + 年份**：**Science Robotics**, 2024, Vol. 9, Issue 89（published 2024-04-24）；arXiv 页面 Journal ref 写作 "Science Robotics, 2024, Vol 9, Issue 89" **[高]**
**DOI**：`10.1126/scirobotics.adi9641` **[高，Crossref 已核]**
**arXiv**：`arXiv:2405.01792` **[高，abs 页已核]**

> 为什么这篇必须收：这是 Track A 里**唯一真正做到"感知式运动 + 城市级导航"端到端闭环**的顶刊工作（苏黎世 + 塞维利亚，公里级自主任务），正好补上第 1 篇"没有导航"的空缺。

### 方案核心机制
- **三级架构**（摘要明确的三件套）：
  1. **自适应运动控制（adaptive locomotion control）**：model-free RL + **privileged learning**（特权教师 → 学生蒸馏），输出对崎岖地形的鲁棒运动，并支持**走/轮驱（walking ↔ driving）的平滑切换**。**[高]**
  2. **运动能力感知的局部导航规划（mobility-aware local navigation planning）**：学习式导航控制器，通过**分层 RL（hierarchical RL）**与底层运动控制器紧耦合——即规划器"知道"底层能过什么地形、多快能过，而不是把底层当成理想质点。**[高]**
  3. **城市级大范围路径规划（large-scale path planning within the city）**：在全局地图上做长程路由。**[高]**
- **输入**：LiDAR/深度 → 高程/可通行性表征（用于局部导航）；本体感受 → 运动策略；全局：城市地图/路网。**[中]**
- **执行**：全局路径 → 局部导航策略（分层 RL 上层）→ 运动控制器（下层）→ 轮腿执行器。**[高，摘要]**

### 关键创新（1–2 条）
1. **"运动能力感知"的导航代价**：局部规划器不是在静态地图上跑 A*，而是学出来的、以底层运动策略的真实能力为条件——这让机器人敢以高速穿过"看起来能过但传统规划器会保守绕开"的地形。**这是 mobility-aware 这个词在顶刊上的落地。** **[高]**
2. **公里级真实城市自主任务的实证**：在苏黎世与塞维利亚完成**公里尺度（kilometer-scale）**的完全自主导航任务，从"实验室 demo"跨到"城市级系统集成"。**[高]**

### 局限 / 未解决问题
- **依赖轮腿平台（wheeled-legged）**：走/驱切换是本文性能的关键来源，纯足式四足无法直接照搬这套结论。**[中]**
- **全局规划依赖先验地图**：城市级路径规划假设有地图/路网先验，不是"进未知环境自己探索"。**[中]**
- **社会性/动态行人只有避让，没有社会规范建模**（对比 Track B 的社会导航文献，这是明显缺口）。**[中]**
- 传感器与算力配置重（多线 LiDAR + 机载计算），**低成本平台复现难**。**[中]**
- 感知式运动的"极限地形"能力弱于第 3 篇（parkour），本文强在系统集成与可靠性。**[低，推断]**

### 核实方式
`web_fetch arxiv.org/abs/2405.01792` 读到 `Journal reference: Science Robotics, 2024, Vol 9, Issue 89` + `Related DOI: 10.1126/scirobotics.adi9641`；`web_fetch api.crossref.org/works?query.bibliographic=...` 与 Crossref 记录复核作者序列（Joonho Lee 第一）与 2024-04-24 出版日。

---

## 5. 备选池（供 Lead 替换 / 补充，8 篇 + 2 篇备查）

> 说明：这一节的作用是"替换弹药"。若 Lead 认为正文缺某一维度（例如缺"端到端单目"、缺"语言/语义"、缺"综述引用"），从这里换。**DOI 均经 Crossref 核实；CoRL 因 PMLR 不注册 DOI，只给 arXiv 编号（已在第 0 节说明，非遗漏）。**

| # | 英文原题 | 第一作者 / 团队 | Venue + 年份 | DOI / arXiv | 一句话定位 |
|---|---|---|---|---|---|
| B1 | **Legged Locomotion in Challenging Terrains using Egocentric Vision** | Ananye Agarwal（CMU，Jitendra Malik / Deepak Pathak 组）**[高]** | **CoRL 2022（顶会，Oral）** **[高，arXiv comments 明写 "Oral presentation at CoRL 2022"]** | `arXiv:2211.07638`（PMLR 无 DOI）**[高，abs 页已核]** | **单目端到端的开山作**：只有前视深度相机、无高程图，两阶段训练（先用廉价深度变体 RL，再监督蒸馏到真实深度），必须靠**记忆**推断后足下方地形。补"端到端 vs 建图"这条对立线的最佳一篇。 |
| B2 | **Extreme Parkour with Legged Robots** | Xuxin Cheng（CMU，Deepak Pathak 组）**[高]** | **ICRA 2024（顶会）** **[高，Crossref]** | `10.1109/ICRA57147.2024.10610200` ／ `arXiv:2309.14341` **[高]** | 与 B1 同组的"极致版"：跳 2 倍身高障碍、跨 2 倍体长沟、斜坡倒立，**单一神经网络直接从相机图像出控制**，靠大规模 RL 硬吃传感噪声。补"敏捷极限"维度。 |
| B3 | **Robot Parkour Learning** | Ziwen Zhuang（上海期智研究院 / 清华，Hang Zhao 组）**[高]** | **CoRL 2023（顶会，Oral）** **[高，arXiv comments 明写 "CoRL 2023 (Oral)"]** | `arXiv:2309.05665`（PMLR 无 DOI）**[高]** | 低成本四足（Unitree A1/Go1 级）纯仿真学会跑酷，**软动力学约束 + 技能复用**，是"低预算也能做 parkour"的引用锚点。 |
| B4 | **ViNL: Visual Navigation and Locomotion Over Obstacles** | Simar Kareer（Georgia Tech / Meta AI，Dhruv Batra 组）**[高]** | **ICRA 2023（顶会）** **[高，Crossref]** | `10.1109/ICRA48891.2023.10160612` **[高]** | **把"视觉导航"和"越过障碍的运动"合成一个策略**（视觉策略 + 运动策略，目标点条件化）。补"导航与运动不再分层"这条路线。 |
| B5 | **Learning to walk in confined spaces using 3D representations** | Takahiro Miki（ETH RSL，Marco Hutter 组）**[高]** | **ICRA 2024（顶会）** **[高，Crossref]** | `10.1109/ICRA57147.2024.10610271` **[高]** | 窄缝/受限空间：**显式 3D 表征**（而不是 2.5D 高程图）+ 特权蒸馏，专治"身体会撞"的场景。补"窄通道/接触丰富"维度。 |
| B6 | **Learning Visual Locomotion with Cross-Modal Supervision** | Antonio Loquercio（UC Berkeley，Jitendra Malik / Ashish Kumar）**[高]** | **ICRA 2023（顶会）** **[高，Crossref]** | `10.1109/ICRA48891.2023.10160760` **[高]** | **跨模态监督**：用本体感受/本体动力学当"教师"去监督视觉表征，不需要激光或高程图。补"特权信息从哪来"的另一条答案（不是仿真特权，而是另一模态）。 |
| B7 | **Agile and Generalized Legged Locomotion via Attention-Based Neural Map Encoding**（前身代号 **AME-2**） | Chong Zhang（ETH RSL，Marco Hutter 组；合作 Victor Klemm、Fan Yang）**[高]** | **IEEE T-RO，条件接收（conditionally accepted）**，截至 2026-09 为 arXiv v3 **[高，arXiv comments 原文 "Conditionally accepted by IEEE Transactions on Robotics (T-RO)"]** | `arXiv:2601.08485` ／ **DOI 待核实**（正式见刊后才有）**[高，编号已核]** | **最新鲜的"注意力 + 神经地图"顶刊级工作**：注意力地图编码器提取局部/全局特征并聚焦显著区域，配"深度→带不确定度局部高程"的学习式建图管线，四足 + 双足都验证。**若 Lead 要"2026 年最新"，这篇替换正文任意一篇都成立；风险是"条件接收"不等于已见刊，引用时须写 conditionally accepted。** |
| B8 | **Learning-based legged locomotion: State of the art and future perspectives** | Sehoon Ha（第一作者；合作 Joonho Lee、Michiel van de Panne、Zhaoming Xie、Wenhao Yu、Majid Khadiv）**[高]** | **IJRR (The International Journal of Robotics Research), 2025, Vol. 44**（published 2025-01-22）**[高，Crossref]** | `10.1177/02783649241312698` **[高]** | **IJRR 综述**。不是新方法，但是写"研究现状/未来方向"章节时**最省事的顶刊级引用**，且作者阵容横跨学术界与工业界。**注意：它是 review，不要当方法论文写进正文主线。** |
| — | 备查：**TAMOLS: Terrain-Aware Motion Optimization for Legged Systems** | Fabian Jenelten（ETH RSL）**[中，未核作者序列]** | **IEEE T-RO, 2022** **[高，Crossref]** | `10.1109/TRO.2022.3186804` **[高]** | 模型派的**地形感知运动优化**（含落脚点/姿态），与正文第 2 篇同组；作为"优化派"第二篇引用。 |
| — | 备查：**Parkour in the Wild: Learning a General and Extensible Agile Locomotion Policy Using Multi-expert Distillation and RL Fine-tuning** | Nikita Rudin（ETH RSL）**[高]** | **arXiv 预印本 2025（期刊状态待核实）** **[中]** | `arXiv:2505.11164` **[高，abs 页已核]** | **多专家蒸馏（DAgger）→ 统一策略 → RL 微调**，ANYmal D 实机。是"多专家蒸馏"这条线的最新代表，但**发表 venue 我没查到，标待核实**。 |

**备选池里我没有放的东西（避免和 Track B/C 撞车）**：VLN/语言导航、社会规范导航、纯 state estimation、纯 MPC 无感知、无人机。这些留给 B/C 组。

---

## 6. 我（Track A）推荐的 4 篇 —— 给 Lead 的 10 篇清单供稿

| 推荐位 | 标题 | Venue | 为什么是它 |
|---|---|---|---|
| A-1 | Learning robust perceptive locomotion for quadrupedal robots in the wild | **Science Robotics 2022** | 感知式运动的"标准答案"：注意力+循环的多模态融合、teacher-student 特权蒸馏，野外长时程实证。**任何四足感知运动的综述都绕不开。** |
| A-2 | Perceptive Locomotion Through Nonlinear Model-Predictive Control | **IEEE T-RO 2023** | 唯一的"优化派"顶刊硬结果（全身 NMPC + SDF 连续落脚）。**有它，报告才不是"学习派一言堂"。** |
| A-3 | ANYmal parkour: Learning agile navigation for quadrupedal robots | **Science Robotics 2024** | 分层技能 + 感知重建做敏捷**导航**，把"过一串障碍"变成技能序列决策。补"长时程障碍序列"。 |
| A-4 | Learning Robust Autonomous Navigation and Locomotion for Wheeled-Legged Robots | **Science Robotics 2024** | 唯一"感知运动 + 城市级导航"闭环，公里级实机任务。补"导航"与"系统集成"。 |

**四篇的覆盖性自检**：学习派(1) / 优化派(2) / 敏捷长时程(3) / 城市导航系统(4)；表征上覆盖 高程图注意力(1)、SDF(2)、重建式感知模块(3)、可通行性+分层(4)；训练范式覆盖 特权蒸馏(1,4)、纯 RL 分层(3)、无学习(2)。**没有两篇是重复的。**

**如果只能留 2 篇**：A-1 + A-2（学习派代表 + 优化派代表，范式对撞最有信息量）。

**如果需要换成 2026 最新的**：用 **B7（AME-2 / arXiv:2601.08485，T-RO 条件接收）** 替换 A-4，并把 A-4 降到备选；但**引用措辞必须写 "conditionally accepted"，不能写成已见刊**。

---

## 7. 我明确不知道 / 需要人工复核的（诚实清单）

1. **A-3（ANYmal parkour）高层策略与底层技能的更新频率**（我印象里高层是几 Hz 量级，但**未核实具体数字**，所以正文里没写数字）。
2. **A-1 的 teacher 特权信息具体字段**（摩擦、接触状态是 ETH 系列的通行做法，**我未逐页核对本文**）。
3. **A-2 的 NMPC 求解频率与预测时域具体数值**（论文报"实时"，但具体 Hz 我没抄下来，故不写）。
4. **B7 的正式 DOI / 卷期**：截至采集日只有"条件接收"，**DOI 栏我写的是"待核实"而不是编一个**。
5. **Parkour in the Wild（arXiv:2505.11164）的发表 venue**：查不到，**留白待核实**。
6. `web_search` 全程不可用，因此**本文件没有"检索覆盖面"的保证**——我只能保证"写出来的都是真的"，不能保证"漏掉的都不存在"。若需补漏，建议恢复 web_search 后按 `perceptive locomotion quadruped` / `attention belief terrain` 再扫一遍 2026 年新刊。