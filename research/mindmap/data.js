// 数据源：四足机器狗导航 · 10 篇顶刊方案整合（纯数据，无 DOM 依赖）
// 所有权重信息（DOI / venue / 年份）均经 Crossref REST API + arXiv Atom API 核实（2026-11-12）。

const PAPERS = {
  miki: {
    short: 'Miki 2022',
    title: 'Learning robust perceptive locomotion for quadrupedal robots in the wild',
    zh: '野外鲁棒感知式运动',
    venue: 'Science Robotics 7(62), 2022',
    doi: '10.1126/scirobotics.abk2822',
    arxiv: 'arXiv:2201.08117',
    team: 'ETH Zürich RSL（Takahiro Miki / Marco Hutter）',
    paradigm: '学习派 · 端到端 RL',
    mech: [
      '输入：本体感受历史（速度/姿态/关节/接触/步态相位）+ LiDAR 高程图的高度采样序列',
      '表征：attention-based recurrent encoder —— 循环记忆累积历史，注意力在高度序列上选择「哪一段地形观测可信」',
      '训练：privileged teacher → student 在线蒸馏（DAgger 式），学生只用可部署观测，再由 PPO 微调',
      '执行：MLP 输出关节目标位置 + PD 跟踪，Isaac Gym 大规模并行训练',
      '导航层很薄：路径外部给定 + pure pursuit —— 它是「感知式运动」，不是自主寻路',
    ],
    innov: [
      '多模态融合靠学习而非启发式开关：雪/植被/水面/反光让高程失真时，控制器自动降权高程、回落到本体感受',
      '野外长时程实证：跨季节多地形，阿尔卑斯约 1 小时徒步（按人类推荐时间完成）',
    ],
    limit: ['无全局导航/无自主寻路', '依赖高程图与状态估计质量', '无语义理解（不懂「草地 vs 玻璃」）', '速度量级低（约 1 m/s）'],
  },

  he2025: {
    short: 'He 2025',
    title: 'Attention-based map encoding for learning generalized legged locomotion',
    zh: '注意力神经地图编码',
    venue: 'Science Robotics, 2025-08-27',
    doi: '10.1126/scirobotics.adv3604',
    arxiv: 'arXiv:2506.09588',
    team: 'ETH Zürich RSL（Junzhe He, Chong Zhang, Fabian Jenelten, Marco Hutter）',
    paradigm: '学习派 · 表征学习',
    mech: [
      '输入：本体感受 + 局部地形地图（高程/深度）',
      '表征：以本体感受为条件（conditioned）的「注意力地图编码器」，作为端到端 RL 控制器的一部分联合训练',
      '行为：网络自己学会「未来落脚点应该看哪些可踏区域」——把地图从输入特征变成注意力对象',
      '输出：关节动作；刻意绕开模型规划器，省掉其算力与手工建模',
    ],
    innov: [
      '稀疏可踏地形上同时拿到「精确落脚」与「学习式鲁棒性」，性能压过纯学习法与计算昂贵的混合法',
      '给出一条「地图表征可学」的路线：不再预先决定高程图长什么样',
    ],
    limit: ['局部地图窗口，无全局目标层', '注意力与特定策略共生，换平台/换任务需重训', '依赖仿真训练分布'],
  },

  hoeller: {
    short: 'Hoeller 2024',
    title: 'ANYmal parkour: Learning agile navigation for quadrupedal robots',
    zh: '技能库 + 高层导航策略（Parkour）',
    venue: 'Science Robotics 9(88), 2024',
    doi: '10.1126/scirobotics.adi7566',
    arxiv: 'arXiv:2306.14874',
    team: 'ETH Zürich RSL + NVIDIA（David Hoeller, Nikita Rudin, Marco Hutter）',
    paradigm: '分层派 · 分层 RL',
    mech: [
      '感知模块：从「高度遮挡 + 高噪声」的深度数据中重建障碍物并给出场景理解（专治小视场盲区）',
      '低层技能库 50 Hz：走 / 跳 / 上攀 / 下攀 / 蹲伏，每类一个参数化 RL 策略，输出关节位置',
      '高层导航策略 5 Hz：输入目标相对位置、剩余时间、感知特征，输出「技能选择 + 技能参数」',
      '全学习：零专家示范、零离线规划、零环境先验；纯仿真训练、零样本迁移真机',
    ],
    innov: [
      '把「连续过一串障碍」的长时程规划，转成低维技能参数空间里的序贯决策，绕开端到端 RL 的稀疏奖励崩坏',
      '感知做成「重建式」子任务，直击连续高障碍场景的视野盲区',
    ],
    limit: ['技能集固定（5 类）即能力上限', '高层仅 5 Hz，动态环境重规划慢', '无全局地图/SLAM/拓扑规划层', '仿真地形课程与域随机化依赖强', '无语义/社会性'],
  },

  lee2024: {
    short: 'Lee 2024',
    title: 'Learning robust autonomous navigation and locomotion for wheeled-legged robots',
    zh: '运动能力感知的城市级导航（轮腿）',
    venue: 'Science Robotics 9(89), 2024',
    doi: '10.1126/scirobotics.adi9641',
    arxiv: 'arXiv:2405.01792',
    team: 'ETH Zürich RSL（Joonho Lee, Marko Bjelonic, Marco Hutter）',
    paradigm: '分层派 · 系统集成',
    mech: [
      '三层架构：① RL 运动控制（特权学习，走 ↔ 轮驱平滑切换）② 运动能力感知的局部导航规划（分层 RL，与底层紧耦合）③ 城市级全局路径规划',
      '局部规划器不是在静态地图上跑 A*，而是以底层运动策略的真实能力为条件（mobility-aware）',
      '输入：LiDAR/深度 → 可通行性表征；全局层：城市地图 / 路网',
    ],
    innov: [
      '「mobility-aware 代价」在顶刊落地：敢高速通过传统规划器会保守绕开、但底层其实能过的地形',
      '公里级真实城市完全自主任务（苏黎世 + 塞维利亚），从 demo 走到系统',
    ],
    limit: ['依赖轮腿平台（走/驱切换是性能关键），纯足式不能直接照搬', '全局规划依赖先验地图，不是未知环境探索', '行人只避让，无社会规范建模', '传感器/算力重，低成本复现难'],
  },

  kim2025: {
    short: 'Kim 2025',
    title: 'High-speed control and navigation for quadrupedal robots on complex and discrete terrain',
    zh: '分层 planner + tracker 高速离散地形',
    venue: 'Science Robotics, 2025-05-28',
    doi: '10.1126/scirobotics.ads6192',
    arxiv: 'arXiv:2506.02835',
    team: 'KAIST（RAIBO 团队，in-house 动态四足 RAIBO）',
    paradigm: '分层派 · 规划+跟踪',
    mech: [
      'Planner：采样式优化 + 启发式与神经网络快速序贯滤波，生成物理可行的落脚点计划；再用物理仿真 rollout 逐条验证，按工程代价函数取最优',
      'Tracker：精确踩中规划器给出的落脚点；训练时用生成模型与 tracker 竞争式训练，让目标落脚分布难度「恰到好处」',
      '分工：规划端吃长时程非凸 + 高自由度，跟踪端负责高频精确落脚',
    ],
    innov: [
      '「采样 + 学习滤波 + 物理仿真验证」三件套，同时要计算效率和物理一致性',
      '用对抗式难度生成训练 tracker，能踩上此前方法踩不了的离散地形',
    ],
    limit: ['强依赖建图与状态估计（在线仿真验证假设一致性）', '计算量大（在线仿真 rollout）', '场景覆盖有限，无语义与长距离全局任务'],
  },

  kang2026: {
    short: 'Kang 2026',
    title: 'Agile perceptive multiskill locomotion for quadrupedal robots in the wild',
    zh: '动作预训练多技能（APT-RL）',
    venue: 'Science Robotics, 2026-07-15',
    doi: '10.1126/scirobotics.adz7397',
    arxiv: 'arXiv 编号待核实（Semantic Scholar 指向 2607.13579，未在 arXiv API 命中）',
    team: 'Kang, Park, Song, …, Hae-Won Park（KAIST 系；单位待核实）',
    paradigm: '分层派 · 技能预训练',
    mech: [
      '用简化动力学的轨迹优化，批量生成大规模、特征丰富的「二维运动数据集」',
      '数据集 → 预训练多样化、可复用的运动技能（action-pretrained transformer 思路）',
      '技能作为强先验，再高效学下游任务；策略自主做技能切换（skill transition）',
      '仅用机载感知与计算；2D 技能自然扩展到 3D，得到平滑的高速多技能运动',
    ],
    innov: [
      '数据来源从「纯 RL 采样」换成「轨迹优化生成 + 动作预训练」，样本效率与技能复用性同时提升',
      '真实世界完成室内复杂障碍与野外敏捷动作（含动态 drop-down）',
    ],
    limit: ['简化动力学数据的质量即上限', '技能覆盖范围限制泛化', '极端天气/材质与长距离全局任务未评估'],
  },

  grandia: {
    short: 'Grandia 2023',
    title: 'Perceptive locomotion through nonlinear model-predictive control',
    zh: '全身 NMPC + SDF 连续落脚',
    venue: 'IEEE T-RO 39(5):3402–3421, 2023',
    doi: '10.1109/TRO.2023.3275384',
    arxiv: 'arXiv:2208.08373',
    team: 'ETH Zürich RSL（Ruben Grandia, Fabian Jenelten, Farbod Farshidian, Marco Hutter）',
    paradigm: '模型派 · 优化控制',
    mech: [
      '感知 20 Hz：点云 → 2.5D 高程图；每次更新预计算「可踏性分类 + 平面分割 + SDF」，不可踏格抬高 2 cm 并膨胀一格',
      '优化 100 Hz：全身非线性 MPC，一次性优化全部自由度（基座 6-DoF + 关节 + 接触力）；地形转成一串凸不等式约束嵌入优化',
      '执行 400 Hz：状态估计 + 全身力矩控制 WBC + 反应式行为；步态序列按 switched system 处理',
      '求解：multiple-shooting + real-time iteration（RTI）+ filter 线搜索',
    ],
    innov: [
      '把「感知式落脚」从离散组合搜索变成连续优化：落脚点与身体轨迹在同一个优化里解',
      '高程图预计算把在线感知查询压到可实时；ANYmal 上做出缝隙/斜坡/踏石的动态通过',
    ],
    limit: ['无全局导航层，滚动时域易陷局部极小', '对定位与高程图质量敏感（硬约束）', '平面片近似，对悬垂/孔洞/植被脆弱', '算力重、权重手调，移植成本高', '无语义'],
  },

  zhang2026: {
    short: 'Zhang 2026',
    title: 'Traversability-aware legged navigation by learning from real-world visual data',
    zh: '用控制器价值函数定义可通行性',
    venue: 'IEEE T-RO, 2026（在线 2025）',
    doi: '10.1109/TRO.2025.3645885',
    arxiv: 'arXiv:2410.10621',
    team: 'Hongbo Zhang, Zhongyu Li, Xuanqi Zeng 等（单位见原文）',
    paradigm: '机器人中心 · 可通行性',
    mech: [
      '可通行性估计器 = 底层运动控制器的 value function（robot-centric），而不是人工标注的地形标签',
      '估计器接入学习式 RGB-D 导航框架，多阶段训练',
      '导航规划器直接在真实世界用 sample-efficient RL 训练：在线数据 + 离线数据集混合',
    ],
    innov: [
      '「可通行性 = 控制器价值函数」：自动反映底层真实能力与不同地形的能耗差异，替掉 human-centric 人工标注',
      '绕开仿真地形真值，直接在真实世界训 planner',
    ],
    limit: ['价值函数估计误差会一路传播到规划', '真实世界训练成本与安全风险', '平台/地形覆盖有限'],
  },

  lin2026: {
    short: 'Lin 2026',
    title: 'One filter to deploy them all: Robust safety for quadrupedal navigation in unknown environments',
    zh: '观测条件化可达性安全滤波器',
    venue: 'IEEE T-RO, 2026（在线 2024-12）',
    doi: '10.1109/TRO.2025.3644957',
    arxiv: 'arXiv:2412.09989',
    team: 'Albert Lin, Shuang Peng, Somil Bansal 等（单位见原文）',
    paradigm: '机器人中心 · 安全层',
    mech: [
      'OCR（observation-conditioned reachability）安全滤波框架：OCR 价值网络预测最优安全价值函数',
      '两个关键件：① LiDAR 输入 → 在线构建安全区（应对新出现的障碍）② 扰动估计模块 → 吸收野外动力学不确定性',
      '名义控制器不安全时由滤波器接管（override）；不假设环境先验、也不绑定某个特定策略',
    ],
    innov: [
      '「One filter to deploy them all」：一个滤波器给多种分层四足控制器做安全兜底',
      '把可达性分析从「离线 + 已知环境」推到「在线 + 未知环境」',
    ],
    limit: ['价值网络近似误差无形式化保证', 'LiDAR 视场/遮挡限制', '高动态动作下接管偏保守', '仿真 + Go1 验证，规模有限'],
  },

  bjelonic: {
    short: 'Bjelonic 2026',
    title: 'Towards bridging the gap: Systematic sim-to-real transfer for diverse legged robots',
    zh: '系统化 sim2real + 一阶原理能量模型',
    venue: 'The International Journal of Robotics Research (IJRR), 2026',
    doi: '10.1177/02783649261459628',
    arxiv: 'arXiv:2509.06342',
    team: 'ETH Zürich RSL（Filip Bjelonic, Fabian Tischhauser, Marco Hutter）',
    paradigm: '工程派 · sim2real',
    mech: [
      'sim2real RL + 永磁同步电机（PMSM）的物理能量模型：用最小参数集刻画现实差距',
      '极简四项奖励 + 一阶原理能量损失（平衡电耗与机械耗散），不依赖复杂手调奖励',
      '自底向上的动力学参数辨识：执行器 → 空中轨迹 → 地面运动',
      '关键：不做动力学参数随机化，也能可靠迁移',
    ],
    innov: [
      '用「物理能量模型 + 系统辨识」替代「大范围域随机化」，顺带把能效做上去',
      '3 个主力平台验证 + 额外 10 台机器人部署，可复现性强',
    ],
    limit: ['偏部署/工程，不含感知与导航决策', '能量模型仍依赖执行器建模精度', '跨执行器类型（液压/直驱）待验证'],
  },
};

const GROUPS = [
  {
    label: '学习派：端到端 RL + 感知编码',
    note: '把「看什么、信多少」交给网络学，不建显式规划器',
    children: [{ paper: 'miki' }, { paper: 'he2025' }],
  },
  {
    label: '分层派：技能/规划器 + 运动策略',
    note: '慢思考管长时程，快反射管落脚',
    children: [{ paper: 'hoeller' }, { paper: 'lee2024' }, { paper: 'kim2025' }, { paper: 'kang2026' }],
  },
  {
    label: '模型派：优化控制（NMPC + 地形约束）',
    note: '硬约束、可解释、算力重',
    children: [{ paper: 'grandia' }],
  },
  {
    label: '机器人中心：可通行性与安全',
    note: '代价与安全边界从「机器人真实能力」出发',
    children: [{ paper: 'zhang2026' }, { paper: 'lin2026' }],
  },
  {
    label: '工程派：sim2real 与实机部署',
    note: '顶刊越来越看「能不能稳定跑起来」',
    children: [{ paper: 'bjelonic' }],
  },
];

var DATA = {
  label: '四足机器狗导航 · 10 篇顶刊方案整合',
  children: [
    {
      label: '① 十大工作（按方案流派分组）',
      note: '点击任意论文节点，右侧显示机制 / 创新 / 局限',
      children: GROUPS,
    },
    {
      label: '② 感知层：地形怎么被「看见」',
      children: [
        {
          label: '输入源（4 类）',
          children: [
            { label: 'LiDAR 高程图 / 高度采样序列（Miki、Grandia）' },
            { label: '深度相机 → 局部高程（Hoeller、Kim）' },
            { label: 'RGB-D 单目或双目（Zhang）' },
            { label: '纯本体感受「盲走」——所有方案的兜底' },
          ],
        },
        {
          label: '地形表征（5 类，代际演进）',
          children: [
            { label: '2.5D 高程图 + 可踏性分类 + SDF（Grandia）' },
            { label: '注意力地图编码：地图本身可学（He 2025）' },
            { label: '隐式障碍重建，根本不建图（Hoeller）' },
            { label: '可通行性 = 底层控制器的价值函数（Zhang）' },
            { label: '在线安全区集合（LiDAR 构建，Lin）' },
          ],
        },
        {
          label: '融合机制（4 类）',
          children: [
            { label: '注意力 + 循环记忆，自动判断观测可信度（Miki）' },
            { label: '本体感受条件化注意力：用身体状态指引看哪（He）' },
            { label: 'privileged teacher → student 蒸馏（ETH 系主流）' },
            { label: '跨模态监督：拿本体动力学当视觉的教师' },
          ],
        },
        {
          label: '感知失效对策',
          children: [
            { label: '不可踏格抬高 2 cm + 膨胀一格（Grandia）' },
            { label: '重建被遮挡地形，弥补小视场（Hoeller）' },
            { label: '在线自监督可通行性，几分钟内适应（WVN, RSS\'23）' },
            { label: '无信心时回落到本体感受（Miki 的注意力）' },
          ],
        },
      ],
    },
    {
      label: '③ 决策层：四种架构范式',
      children: [
        {
          label: 'A 端到端单策略（无规划层）',
          children: [
            { label: '方向/目标向量条件化，策略直接吃图像出动作' },
            { label: '优点：无接口损耗 · 缺点：长时程稀疏奖励、无中间量可查' },
            { label: '代表：Extreme Parkour（ICRA\'24）、He 2025' },
          ],
        },
        {
          label: 'B 分层：高层导航策略 + 低层技能（5 Hz / 50 Hz）',
          children: [
            { label: 'ANYmal Parkour：在技能参数空间里做决策' },
            { label: 'ViNL：2 Hz 导航策略 + 50 Hz 运动策略（ICRA\'23）' },
            { label: '共同缺陷：两层之间只有速度指令这条窄接口' },
          ],
        },
        {
          label: 'C 分层：全局路径 + 局部规划 + 控制（20 / 100 / 400 Hz）',
          children: [
            { label: 'Grandia：高程图 20 Hz → NMPC 100 Hz → WBC 400 Hz' },
            { label: 'Lee 2024：全局路网 + mobility-aware 局部规划 + RL 运动' },
            { label: 'Kim 2025：planner（采样+学习滤波+仿真验证）+ tracker' },
          ],
        },
        {
          label: 'D 混合：学习式规划器 + 可微优化',
          children: [
            { label: 'iPlanner（RSS\'23）：imperative learning + 双层优化' },
            { label: '免标签、免演示，用可微 ESDF 代价做隐式监督' },
          ],
        },
        {
          label: 'E 外挂安全层（接管式）',
          children: [
            { label: 'Lin 2026：不安全时 override 名义控制器' },
            { label: '与 A–D 正交，可叠加在任意架构上' },
          ],
        },
      ],
    },
    {
      label: '④ 控制与训练：动作怎么被产生',
      children: [
        {
          label: '底层控制器（3 种）',
          children: [
            { label: 'RL 策略输出关节目标位置 + PD 跟踪（多数）' },
            { label: '全身 NMPC 直接出力矩 + WBC（Grandia）' },
            { label: '步态/接触序列作为 switched system 分段优化' },
          ],
        },
        {
          label: '训练范式（5 条路线）',
          children: [
            { label: '特权教师 → 学生蒸馏：最主流的 sim2real 手段' },
            { label: '大规模并行 RL（Isaac Gym 类）' },
            { label: '轨迹优化生成数据 → 动作预训练（Kang 2026）' },
            { label: '真实世界 sample-efficient RL：在线 + 离线（Zhang）' },
            { label: '对抗式难度生成：生成模型 vs tracker（Kim 2025）' },
          ],
        },
        {
          label: '奖励与代价设计',
          children: [
            { label: '四项极简奖励 + 一阶原理能量损失（Bjelonic）' },
            { label: 'clearance reward + 方向蒸馏（Extreme Parkour）' },
            { label: '能耗 / 可通行性进规划代价（Zhang、Lee）' },
          ],
        },
        {
          label: 'sim2real 两种哲学',
          children: [
            { label: '域随机化：扰动铺开，逼策略鲁棒（ETH 早期主流）' },
            { label: '系统辨识 + 物理能量模型，不随机化参数（Bjelonic）' },
          ],
        },
      ],
    },
    {
      label: '⑤ 评测与部署',
      children: [
        {
          label: '平台分布',
          children: [
            { label: 'ANYmal（ETH）：本清单占 5 篇' },
            { label: 'RAIBO（KAIST）：Kim 2025、Kang 2026' },
            { label: '轮腿平台（Swiss-Mile）：Lee 2024' },
            { label: 'Go1 / AlienGo 等低成本平台：安全层与外挂验证' },
          ],
        },
        {
          label: '能力标尺（可直接引用的硬指标）',
          children: [
            { label: '野外徒步约 1 小时（Miki，按人类推荐时间）' },
            { label: '公里级城市完全自主（Lee 2024）' },
            { label: '约 2 m/s 连续穿越障碍（Hoeller、Kim）' },
            { label: '野外敏捷多技能 + 动态 drop-down（Kang 2026）' },
          ],
        },
        {
          label: '指标缺口',
          children: [
            { label: 'SR / SPL 掩盖碰撞与跌倒（VLN-PE, ICCV\'25 实证）' },
            { label: '普遍缺能耗、稳定性、关节负载、接管次数' },
            { label: '建议自建：碰撞率 / 跌倒率 / 恢复时间 / 每米能耗' },
          ],
        },
      ],
    },
    {
      label: '⑥ 整合结论：6 条趋势',
      children: [
        { label: '从「感知式运动」到「能力感知导航」：规划代价来自底层真实能力（Lee、Zhang）' },
        { label: '地图在退化也在升级：显式高程图 → 学习式价值/注意力表征' },
        { label: '分层没有消失，而是切成「慢思考 1–5 Hz + 快反射 50–400 Hz」' },
        { label: '安全层成为标配：可达性 / 不变集 / 接管式滤波' },
        { label: '训练范式多元化：RL 不再是唯一答案（轨迹优化预训练、真实世界 RL、系统辨识）' },
        { label: '两个明显空白：未知环境长距离全局一致性、语义/语言/社会规范（顶刊缺位）' },
      ],
    },
    {
      label: '⑦ 对本项目（绝影 Lite2）的落地映射',
      children: [
        {
          label: '现状基线（据《总体技术方案》）',
          children: [
            { label: '2D 占据栅格 + scan-to-map + A*/frontier + move_base + 速度仲裁' },
            { label: 'RGB-D 424×240 低视角 + 2D 雷达；仿真真值容差 0.25 m' },
            { label: '缺口：无 3D 地形/可通行性、无越障动力学、无回环检测' },
          ],
        },
        {
          label: '可立即借鉴（低成本、不动主干）',
          children: [
            { label: '通行代价层：坡度/净空/粗糙度 + 定位质量进代价图（Zhang 思路的工程近似）' },
            { label: '接管式安全层：速度仲裁前加限幅/急停（Lin 思路 → 简化 CBF、速度障碍）' },
            { label: '低视角补偿：目标识别与深度定位做高度/视角条件化（GVNav, ICRA\'25）' },
          ],
        },
        {
          label: '中期可试（需要训练资源）',
          children: [
            { label: '分层切口不变：全局 A*/frontier 保留，局部换学习式 planner（iPlanner 思路）' },
            { label: 'teacher-student：先在高程图特权信息下训局部策略，再蒸到深度/RGB-D' },
            { label: '评测升级：按 VLN-PE 协议加碰撞率/跌倒率/恢复时间/能耗' },
          ],
        },
        {
          label: '不要照搬的部分',
          children: [
            { label: '轮腿走/驱切换（平台不匹配，Lite2 是纯足式）' },
            { label: '依赖先验城市路网的全局规划（本项目是未知废墟场景）' },
            { label: '闭源 VLM 高频在线推理（机载算力与延迟不现实）' },
          ],
        },
      ],
    },
    {
      label: '⑧ 参考文献与核验',
      children: [
        {
          label: '10 篇顶刊构成',
          children: [
            { label: 'Science Robotics ×6：2022 / 2024×2 / 2025×2 / 2026' },
            { label: 'IEEE T-RO ×3：2023 / 2026×2' },
            { label: 'IJRR ×1：2026' },
          ],
        },
        {
          label: '补充顶会（非顶刊，仅作趋势参照）',
          children: [
            { label: 'iPlanner (RSS 2023) · ViNL (ICRA 2023)' },
            { label: 'Extreme Parkour (ICRA 2024)' },
            { label: 'VLM-PC (ICRA 2025) · GVNav (ICRA 2025)' },
            { label: 'VLN-PE (ICCV 2025) · WVN (RSS 2023) · SayNav (ICAPS 2024)' },
          ],
        },
        {
          label: '核验说明（诚实标注）',
          children: [
            { label: 'DOI / venue / 年份 / 卷期：Crossref REST API 逐条核对' },
            { label: '标题 / 摘要 / 机制：arXiv Atom API + 官方项目页' },
            { label: '本次 web_search 后端 401 不可用，改用上述 API 直查' },
            { label: '未核实项已标「待核实」，未编造任何 DOI 或年份' },
          ],
        },
      ],
    },
  ],
};

if (typeof module !== 'undefined' && module.exports) {
  module.exports = { DATA, PAPERS };
}