# Track B：四足机器人"分层式 / 模型式"导航（perception-aware MPC、地形感知规划、分层 RL）

> 撰写：researcher-b（Team Lead: lead）｜共享任务 task-2
> 完成时间：2026-11-12
> 检索与核实方法：**Crossref REST API**（`api.crossref.org`，按标题检索 → 取 DOI / 期刊或会议名 / 年 / 卷期页 / 第一作者与单位）+ **arXiv Atom API**（`export.arxiv.org/api/query`，取英文原题 / arXiv 号 / 作者序 / 摘要）+ arXiv HTML 全文片段（取各模块运行频率、技能库、训练方式等机制细节）。
> 说明：`web_search` 工具在本会话不可用（后端 API 401），改用上述两个官方元数据 API + arXiv 全文做"可复查式"核实；凡未经一手页面确认的内容均标注"待核实"，不做推测性填写。
> 结论先行：本 Track 给出 **4 篇**均为顶刊/顶会、DOI 可查的工作；另附 **8 篇备选池**（全部带真实 DOI）。

## 汇总表

| 编号 | 论文（缩写） | 分层/机制类型 | Venue, 年份 | DOI / arXiv |
|---|---|---|---|---|
| B1 | Perceptive Locomotion Through Nonlinear MPC | 感知→NMPC（全自由度、含落脚约束）→WBC（优化） | IEEE T-RO, 2023 | 10.1109/TRO.2023.3275384 / arXiv:2208.08373 |
| B2 | ANYmal Parkour: Learning Agile Navigation… | 感知模块 + 5 Hz 高层导航策略 + 50 Hz 技能库（分层 RL） | Science Robotics, 2024 | 10.1126/scirobotics.adi7566 / arXiv:2306.14874 |
| B3 | iPlanner: Imperative Path Planning | 感知→规划网络 + 可微轨迹优化（双层，学习+优化混合） | RSS, 2023 | 10.15607/RSS.2023.XIX.064 / arXiv:2302.11434 |
| B4 | ViNL: Visual Navigation and Locomotion Over Obstacles | 2 Hz 导航策略 + 50 Hz 运动策略（两级独立训练、零样本联调，RL） | ICRA, 2023 | 10.1109/ICRA48891.2023.10160612 / arXiv:2210.14791 |

方向覆盖：perception-aware / terrain-aware MPC ✔（B1）；elevation map + 落脚点可行性约束 ✔（B1）；分层 RL（高层导航 + 低层运动）✔（B2、B4）；学习式局部规划 ✔（B3）；语义可通行性、社会性导航 → 见"备选池"与"覆盖缺口"。

---

## B1. Perceptive Locomotion Through Nonlinear Model-Predictive Control

- **英文原题**：Perceptive Locomotion Through Nonlinear Model-Predictive Control
- **第一作者 / 团队**：**Ruben Grandia**（第一作者）；ETH Zurich, Robotic Systems Lab（负责人 **Marco Hutter**）；合作者 Fabian Jenelten、Shaohui Yang（EPFL）、Farbod Farshidian、Marco Hutter
- **Venue + 年份**：**IEEE Transactions on Robotics (T-RO)**, vol. 39, no. 5, pp. 3402–3421, **2023 年 10 月**（在线首发 2023-05-29）
- **DOI / arXiv**：DOI **10.1109/TRO.2023.3275384**；arXiv:**2208.08373**
- **分层结构与地图表征**：
  - 三层流水线（各层频率来自论文正文）：
    1. **感知层 20 Hz**：点云 → **elevation map（2.5D 高程图）**；每次地图更新时**预计算**可踏性分类（steppability classification）、**平面分割**（plane segmentation）、**SDF（符号距离场）**；
    2. **核心层 100 Hz**：非线性 MPC，一次性优化**全部自由度**（基座 6-DoF 位姿 + 关节 + 接触力），把操作员速度指令、感知信息、机器人实测状态耦合进来；
    3. **执行层 400 Hz**：状态估计 + 全身力矩控制（WBC）+ 反应式行为。
  - 地图表征：elevation map 上每格做"可踏 / 不可踏"分类；不可踏单元格抬高 2 cm 并膨胀一格（修正高程图边缘被低估的问题）；SDF 由 2.5D 高程图直接、稠密地算出（3D 体素存值与 3D 梯度）；另有 **torso reference map** 依据落脚可达性决定躯干高度与姿态。
- **优化 or 学习**：**完全基于优化**（NMPC）。地形信息被转成**一串凸不等式约束**（落脚可行性 / 平面近似），嵌入在线 MPC；求解用 **multiple-shooting + real-time iteration (RTI) + filter 线搜索**，以保证高频率下可靠收敛。
- **关键创新**：
  1. 把"可感知地形"以**凸约束序列**的形式直接放进 NMPC，从而实时优化**全自由度且含落脚点选择与碰撞避免**的运动（此前多为分层：规划器选落脚点、控制器再跟踪）；
  2. 每次高程图更新做一次性预计算（分类/平面/SDF），把在线优化中的感知查询代价压到可实时；
  3. 在 ANYmal 上做出缝隙、斜坡、踏石（stepping stones）的动态通过，论文自述达到 state-of-the-art 动态攀爬能力。
- **局限**：
  1. **无全局导航层**：本质是局部 MPC（滚动时域），没有拓扑/全局规划，长距离场景可能被局部极小值困住；
  2. 强依赖**定位精度与高程图质量**，地图外区域不可见 → 对动态障碍、可移动物体、柔软/可变形地形无建模；
  3. 地形以**平面片近似**，对高度非结构化（悬垂、孔洞、植被）仍脆弱；
  4. 计算重（GPU 高程图 + 高效 NMPC 求解器 + 手调权重），复现与移植到算力受限平台成本高；
  5. 不含语义信息（无法区分"能踩但不应踩"，如草坪/地毯/玻璃），也无社会性约束。

---

## B2. ANYmal Parkour: Learning Agile Navigation for Quadrupedal Robots

- **英文原题**：ANYmal Parkour: Learning Agile Navigation for Quadrupedal Robots
- **第一作者 / 团队**：**David Hoeller**（ETH Zurich Robotic Systems Lab & NVIDIA）；合作者 Nikita Rudin、Dhionis Sako、Marco Hutter（ETH RSL）
- **Venue + 年份**：**Science Robotics**, vol. 9, **2024**（2024-03-13 在线）
- **DOI / arXiv**：DOI **10.1126/scirobotics.adi7566**；arXiv:**2306.14874**
- **分层结构与地图表征**：
  - 三层**全学习**结构（频率来自论文正文）：
    1. **感知模块**：从"高度遮挡 + 噪声"的传感数据中**重建障碍物**并给出场景理解，同时融合本体感知并估计机器人在障碍上的位置（用于抗噪声与抗漂移）；
    2. **低层 locomotion module，50 Hz**：技能库（catalog of policies）——**行走 / 上攀 / 下攀 / 蹲伏 / 跳跃**，每个技能由 RL 单独训练，输出关节位置指令；
    3. **高层 navigation policy，5 Hz**：输入最终目标的相对位置、剩余时间、感知特征，输出**技能选择 + 技能参数（目标点、朝向、时间）**。
  - 地图表征：**不是显式几何地图**，而是训练出的**障碍物重建表征**（隐式、面向下游策略），论文强调"无需环境先验、无需离线计算"。
- **优化 or 学习**：**分层强化学习**。技能策略与导航策略都在仿真中训练；训练高层时**低层技能冻结**（hierarchical RL，内层 50 Hz / 外层 5 Hz 双环）。高层策略因"知道每个技能的能力边界"而具备 capability-aware 行为。
- **关键创新**：
  1. **技能库 + 高层选择/参数化**的分层 RL：不依赖专家演示、离线轨迹优化、接触显式建模，就能规划跨越连续障碍的路径；
  2. 感知模块专门训练以**重建被遮挡地形**，弥补有限视场；
  3. 纯仿真训练 → 零样本迁移真机，连续穿越多种障碍、速度最高约 **2 m/s**（论文摘要自述）。
- **局限**：
  1. **技能库是离散预设**（5 类），未训练过的障碍类型（窄梁、动态障碍、极端接触）无法覆盖，扩展需要重新训练技能；
  2. 高层仅 **5 Hz** 决策，对高速动态环境/快速重规划不友好；
  3. **没有全局地图/SLAM 与拓扑规划层**，只在局部地形片段上朝目标前进，长距离未知环境的全局一致性未解决；
  4. 重度依赖仿真与域随机化，真实传感器退化（雨雾、反光、强光）下的感知重建可能失配；训练成本高；
  5. 无**语义/社会性**信息，无法表达"可走但不该走"（人群、草地、禁行区）。

---

## B3. iPlanner: Imperative Path Planning

- **英文原题**：iPlanner: Imperative Path Planning
- **第一作者 / 团队**：**Fan Yang**（ETH Zurich, Robotic Systems Lab）；合作者 Chen Wang（SUNY Buffalo, Spatial AI and Robotics Lab）、Cesar Cadena、Marco Hutter（ETH RSL）
- **Venue + 年份**：**Robotics: Science and Systems (RSS) XIX**, **2023**（2023-07-10）
- **DOI / arXiv**：DOI **10.15607/RSS.2023.XIX.064**；arXiv:**2302.11434**
- **分层结构与地图表征**：
  - 两级"学习 + 优化"混合结构：
    1. **感知/规划网络**：ResNet-18 编码**单帧深度图**得到感知嵌入，与目标点特征拼接 → 预测 **n 个关键点路径** $\mathcal{K}$，并额外输出**碰撞概率 μ**（"fear" 头）；
    2. **可微轨迹优化层**：以关键点为约束，用**三次样条**插值/平滑（PyPose 实现的可微层），生成动力学可行的平滑轨迹；训练时该层与网络更新构成 **Bi-Level Optimization (BLO)**，任务级代价可端到端反传到网络。
  - 地图表征：训练期用 Matterport3D 重建点云生成 **ESDF 代价图**（高斯平滑后作为可微 cost map）提供**隐式监督**；部署时**不需要代价图**，策略直接从自中心深度里"解码"可通行性并规划。
  - 体系位置：输出轨迹作为**局部规划层**，真机实验在 **ANYmal**（板载 NVIDIA Jetson Orin，前置 Intel RealSense D435 深度相机）上执行，目标点由操作员按序给出（waypoint）。
- **优化 or 学习**：**两者结合**——网络用"imperative learning"（无标签、无专家演示，靠可微 ESDF 的轨迹代价 + BLO 直接梯度下降）训练；推理时仍保留轻量样条轨迹优化层保证平滑与可行性。
- **关键创新**：
  1. 提出 **Imperative Learning**：用**可微代价图**把"模型式规划目标"变成网络的隐式监督，既避免监督学习的标签/演示，也避免 RL 的稀疏奖励与低样本效率；
  2. **BLO 双层优化**（网络更新 ⊗ 轨迹优化）端到端可微，把任务级代价一路反传到感知前端；
  3. 实测：比经典 motion-primitive 规划**快约 4×**；相比学习类基线 **SPL 提升 26–87%**；对定位噪声鲁棒，泛化到未见环境。
- **局限**：
  1. 只是**局部规划器**，全局目标/waypoint 由外部（人或全局规划器）提供，本身不含 SLAM/拓扑规划闭环；
  2. 代价图是 **2D（平面）ESDF + 几何避障**，不建模**落脚可行性/地形可通行性**（真机中爬楼梯靠底层运动策略完成），非结构化 3D 地形表达不足；
  3. 训练依赖 Matterport3D 类**室内**场景分布，室外/植被/极端光照的泛化"待核实"（论文实验以室内为主）；
  4. 碰撞概率阈值（μ<0.5 才执行）等超参需调；无**语义**（不能区分同类几何但语义不同的区域），也无社会规范建模；
  5. 底层跟踪控制器在论文中的具体来源/型号，摘要与正文前段未明确给出——**待核实**（本条目不对其做断言）。

---

## B4. ViNL: Visual Navigation and Locomotion Over Obstacles

- **英文原题**：ViNL: Visual Navigation and Locomotion Over Obstacles
- **第一作者 / 团队**：**Simar Kareer**（Georgia Institute of Technology）；合作者 Naoki Yokoyama、Dhruv Batra、Sehoon Ha、Joanne Truong（Crossref 记录的单位均为 Georgia Institute of Technology）
- **Venue + 年份**：**IEEE International Conference on Robotics and Automation (ICRA)**, **2023**, pp. 2018–2024
- **DOI / arXiv**：DOI **10.1109/ICRA48891.2023.10160612**；arXiv:**2210.14791**
- **分层结构与地图表征**：
  - 两级**全 model-free** 结构（频率来自论文正文）：
    1. **视觉导航策略，2 Hz**：顶层深度相机（上仰 15°）→ 输出**线速度/角速度指令**，把机器人导向目标坐标；在 **Habitat** 中用 **DD-PPO** 训练，训练时随机化相机 roll/pitch ±30° 以抗机体抖动；
    2. **视觉运动策略，50 Hz**：底层深度相机（下俯 30°）→ 直接输出关节控制，使机器人"抬脚跨过"地面小障碍并跟随速度指令；在 **Isaac Gym** 中用 PPO 训练，教师策略吃 privileged **高度图（1.6 m × 1.0 m 网格）**，再用 **Learning by Cheating** 把 privileged 编码器蒸馏成深度编码器。
  - 地图表征：**不使用显式地图**（无 map、纯自中心视觉）；地形知识只存在于教师策略的 privileged height map 中，并通过蒸馏转移到学生。
  - 联调方式：两个策略**在两个不同仿真器中各自独立训练**，部署时把导航策略的速度指令直接喂给运动策略，**零样本联合部署、完全不 co-train**。
- **优化 or 学习**：**纯学习**（两级 RL / 蒸馏），无优化层、无建图。
- **关键创新**：
  1. 作者自述为**首个全学习**方案，同时用视觉完成"陌生室内环境导航"与"跨过路径上小障碍的视觉运动"（此前工作通常只做其中之一）；
  2. **两仿真器独立训练 + 速度指令接口零样本联调**，工程上证明了模块化学习策略可组合；
  3. 在"远距离目标 + 未知环境"任务上，仅用自中心视觉就明显优于"用 privileged 地形图做鲁棒运动"的基线：成功率 **+32.8%**，每米碰撞 **−4.42** 次（摘要数据）；
  4. 真机 **AlienGo** 双深度相机验证。
- **局限**：
  1. 两级之间只有速度指令这一条"窄接口"，**没有联合优化/闭环反馈**：导航策略不知道运动策略已失败（如被卡住），易出现"指令与能力不匹配"；
  2. 只处理**室内小型杂物**（鞋、玩具、线缆），不处理大障碍、楼梯、高台——不是真正的地形导航；
  3. **无地图、无长期记忆**，长距离导航会累积漂移，也无法做全局一致性规划；
  4. 两个策略分属不同仿真器，视觉外观/物理分布差异靠零样本硬扛，鲁棒性边界未系统评估；
  5. 无**语义、无社会性**层（不会避让人群、不懂可通行语义）。

---

## 备选池（8 篇，DOI 均已逐条核实）

> 用途：若主表需要替换/扩充（例如想强调"纯高程图建图"、"可通行性学习"、"经典落脚点规划"），可直接取用。

| # | 英文标题 | 第一作者 | Venue, 年份 | DOI / arXiv | 一句话定位 |
|---|---|---|---|---|---|
| A1 | Traversability-Aware Legged Navigation by Learning From Real-World Visual Data | Hongbo Zhang（Zhongyu Li, Xuanqi Zeng 等） | IEEE T-RO, 2026（DOI 含 2025，在线版 2025） | 10.1109/TRO.2025.3645885 | 用真机视觉数据学习**可通行性代价**并驱动腿足导航；最贴近"语义/可通行性导航"缺口（卷期页待核实） |
| A2 | Learning robust perceptive locomotion for quadrupedal robots in the wild | Takahiro Miki | Science Robotics 7(62), 2022 | 10.1126/scirobotics.abk2822 | 外感+本体注意力 RNN 编码器，teacher-student，雪地/植被/水面鲁棒；"感知式运动"事实标准基线 |
| A3 | Learning quadrupedal locomotion over challenging terrain | Joonho Lee | Science Robotics, 2020 | 10.1126/scirobotics.abc5986 | privileged 学习范式的奠基工作（教师用高程/摩擦等特权信息，学生只用本体感知） |
| A4 | Fast Traversability Estimation for Wild Visual Navigation | Jonas Frey | RSS 2023 | 10.15607/RSS.2023.XIX.054 | 自监督**可通行性分割**（DINO 特征 + 在线自监督），5 分钟内野外自适应；语义可通行性方向首选 |
| A5 | Elevation Mapping for Locomotion and Navigation using GPU | Takahiro Miki | IROS 2022 | 10.1109/IROS47612.2022.9981507 | **elevation map** 的 GPU 实现（robot-centric、带不确定性），是 B1/B2 类系统的地图基座 |
| A6 | Navigation planning for legged robots in challenging terrain | Martin Wermelinger | IROS 2016 | 10.1109/IROS2016.7759199 | 经典"高程图 + 落脚点/运动基元搜索"的腿足导航规划，理解分层导航传统的必读 |
| A7 | Real-time Optimal Navigation Planning Using Learned Motion Costs | Bowen Yang | ICRA 2021 | 10.1109/ICRA48506.2021.9561861 | 学习"运动代价"驱动的实时最优导航规划（PRM* + 学习代价），衔接经典规划与学习 |
| A8 | TerraPN: Unstructured Terrain Navigation using Online Self-Supervised Learning | Adarsh Jagan Sathyamoorthy | IROS 2022 | 10.1109/IROS47612.2022.9981942 | 在线自监督学习**非结构化地形可通行性**（含打滑/崎岖代价），面向野外地面机器人 |

**备选池中"待核实"条目（不占正式名额，避免假信息）**：
- *SNAP: Semantic Navigation Among Pedestrians*（J. Wolfram, P. Roth, F. Yang, C. Cadena, M. Hutter；arXiv:**2611.03498**，2026）——语义 BEV 图 + 人群社会规范的单策略导航，zero-shot 迁到腿足机器人。**Crossref 无记录 → venue 待核实（疑似在投）**。
- *TACET: Context-Appropriate Acoustic-Social Navigation for Quadrupeds*（S. Park, Y.-S. Shin, S. Kim；arXiv:**2610.03828**，2026）——把"走路多吵"当社会变量，慢速 VLM 推理 + 快速反应控制器分层。**venue 待核实**。

---

## 覆盖缺口与选型建议（给 Lead 的结论）

1. **已覆盖**：perception-aware / terrain-aware MPC（B1）；elevation map + 落脚可行性约束（B1、A5）；分层 RL 高层导航 + 低层运动（B2、B4）；学习式局部规划 + 可微代价（B3）。
2. **缺口（社会性导航）**：2021–2025 在 T-RO / ICRA / RSS 上**没有**可直接引用的"四足社会性导航"旗舰论文（Crossref 检索到的多为轮式/双足或非顶会）。若要写这一节，建议以 arXiv 2026 的 SNAP / TACET 为线索并标注"venue 待核实"，或改引"人群导航 + 腿足平台"的组合证据。
3. **缺口（语义可通行性）**：可用 A1（T-RO，真机视觉学可通行性）+ A4（RSS，自监督可通行性）组合，比单篇更稳。
4. **写作/答辩可用的对比轴**：① 地图表征（elevation map / ESDF / 隐式表征 / 无图）；② 优化 vs 学习 vs 混合；③ 分层切口（感知-规划-控制 vs 高层导航-低层技能 vs 学习-优化双层）；④ 全局层有无（四篇主表都缺全局 SLAM/拓扑规划，这是可指出的共同局限，也是竞赛方案的差异化机会）。

## 核实台账（可复查）

| 条目 | 核实来源 | 核实到的字段 |
|---|---|---|
| B1 | Crossref `query.title` + arXiv Atom + arXiv HTML | DOI、T-RO 39(5):3402–3421、2023-10、5 位作者及 ETH/EPFL 单位；20/100/400 Hz、SDF/可踏性/平面分割、2 cm 膨胀 |
| B2 | Crossref `query.title` + arXiv Atom + arXiv HTML | DOI、Science Robotics vol. 9、2024-03-13、Hoeller/Rudin/Sako/Hutter；5 Hz 高层 / 50 Hz 技能库、5 类技能、2 m/s |
| B3 | Crossref `query.title` + arXiv Atom（journal_ref/doi 字段）+ arXiv HTML | DOI、RSS XIX 2023-07-10、4 位作者；ESDF 可微代价、BLO、4× 提速、SPL +26–87%、ANYmal + Jetson Orin + D435 |
| B4 | Crossref `query.title` + arXiv Atom + arXiv HTML | DOI、ICRA 2023 pp. 2018–2024、5 位作者（Georgia Tech）；2 Hz / 50 Hz、AlienGo、双深度相机 15°/30°、±32.8% 成功率、−4.42 碰撞/米 |
| A1–A8 | Crossref `query.title`（逐条） | 标题、DOI、venue、年、第一作者（A1 卷期页待核实） |
| 待核实项 | — | B3 底层跟踪控制器来源；A1 卷期页；SNAP/TACET 的 venue |