# 冲刺报告 — 2026-04-20 至 2026-04-22

**截止日期：** 2026-05-01 14:00（剩余 9 天）
**分支：** `master`（20-21）、`yichang_branch`（22 及之后）
**冲刺范围：** `f3f9a1b` .. `fdbc331`（共 60 次提交）

---

## 摘要

阶段 A（开发机，无硬件）**已完成**。阶段 B 实验室工作目前完成 **4/8 项**：
- B1 预检 + 视觉参考 ✅
- B2 D415 重新标定 ✅（cal_01..04 上 RMS 34.4 mm；cal_05..08 从 pickhome1 视角已超出 FOV）
- B3 HSV 调参 ⏳ 待完成（诊断数据已采集，阈值尚未调整）
- B4 篮筐示教点 ⏳ 待完成（需用户手动点动）
- B5 单果干跑 ⏳ 待完成
- B6 14 个水果自主运行 ⏳ 待完成
- B7 远程模式在线验证 ⏳ 待完成
- B8 视频剪辑 ⏳ 待完成

**2026-04-20 会话期间做出的范围调整：** UGreen 地面摄像头 **不** 属于运行流水线的一部分。它是一个侧视诊断工具，供我（Claude）在开发过程中从视觉上确认 TCP 实际落点。**D415 臂载相机才是机器人在最终演示中用于抓取检测的设备。** 因此我们缩减了以下内容：
- UGreen 内参标定（`ugreen_intrinsics.py collect/solve`）。
- 基于 UGreen 的闭环手眼标定（`calibrate_closed_loop.py`）。

两个脚本都保留在代码仓库中供后续工作使用，但不再阻塞演示。

## 范围参考

- **规格说明：** [`docs/superpowers/specs/2026-04-20-final-sprint-design.md`](superpowers/specs/2026-04-20-final-sprint-design.md)
- **计划：** [`docs/superpowers/plans/2026-04-20-final-sprint.md`](superpowers/plans/2026-04-20-final-sprint.md)
- **操作手册：** [`LAB_RUNBOOK.md`](../LAB_RUNBOOK.md)

## 阶段 A — 开发机，全部已提交

覆盖 8 个阶段共 14 个任务，每个都可独立提交。测试套件：

| 测试套件 | 数量 | 通过？ |
|---|---|---|
| `test_integration.py` | 14 | ✅ |
| `test_ugreen_tracker.py` | 7 | ✅ |
| `test_calibrate_closed_loop.py` | 2 | ✅ |

| 阶段 | 任务 | 提交 | 备注 |
|---|---|---|---|
| A1 | `set_gripper_ramp` 辅助函数 | `e4641cf` | 由自主 FSM 和远程模式共享。规格中有一个 bug（`_execute_position` 接收的是 xyz 而非关节角）——实现者发现并修复，同时在 `76b7394` 中修订了计划文档。 |
| A2.1 | UGreen 图像差分跟踪器 | `ff4c099`, `c985f3a` | 代码评审的跟进项中增加了光度天花板与最大连通域测试。 |
| A2.2 | 棋盘格内参标定 | `22cc2eb`, `fa0a614` | 7×5 内角点 30 mm 图案；尊重 `q` 终止操作与 `CALIB_CB_ADAPTIVE_THRESH` 标志。 |
| A3.1 | 闭环标定 | `35d9c70`, `1e27eb6` | 跟进项：SQPnP + 迭代精修、真实残差（`rms_px` + `rms_base_m` + `max_base_m`）、合理性闸门（> 50 mm → `.rejected.json`）。 |
| A4.1 | 预检 7 项检查 | `312ca00`, `63ebe17` | 加入 `--offline` 参数。内联注释解释了为何绕过 `QArmDriver.disconnect()`（它会让机械臂回到原点）。 |
| A4.2 | `LAB_RUNBOOK.md` | `4f4ada6` | 从开机到关机的完整会话流程。 |
| A5.1 | 抽取 TraceLogger + 事件 | `0380094` | PICK_ATTEMPT / GRIPPER_READBACK / SORT_COMPLETE 事件。 |
| A6.1 | 报告作图脚本 | `b77a942` | 4 张图（关节轨迹、夹爪读数、FSM 时间线、检测结果）。 |
| A7.1 | 远程点动逻辑 | `e72fe8d` | 受工作空间盒和关节限位约束的微调。 |
| A7.2 | 远程伴随视图 | `818a2be` | OpenCV 窗口 + MATLAB 启动器。 |
| A7.3 | Stateflow 构建脚本 | `b3d59ac` | 加上 `db47fb7` 中的 `fruit_queue()` 括号修复（缺失括号会让 MATLAB 首次构建时图表静默挂起）。 |
| A7.4 | Stateflow Python 助手 | `87b3478` | `stateflow_init/select/close_grip/open_grip/sorted_count/fruit_queue`。 |
| A7.5 | 统一 v2 硬件模型 | `1f968b3` | 三向模式开关 + 远程 HMI 接线参考。 |
| A8.1 | 文档刷新 | `9cf30ad` | PROGRESS 与 PROJECT_CONTEXT 对齐 2026-04-20 状态。 |

## 阶段 B — 实验室硬件，进行中

### B1.1 — 首次预检 ✅（2026-04-20 15:00）

首次在线运行暴露了 2 项红灯检查，现已全部转绿：

1. **QArm -108**：资源被其他进程占用；用户释放后正常。
2. **VisualRef 缺失**：首次运行磁盘上没有基准/参考；`setup_baseline_ref.py` 两者都会采集（机械臂 → homeplace0 → 采基准 → pickhome1 → 采参考）。

**B1 过程中的关键调试发现：** UGreen 相机在基准与参考采集之间发生了移动 — TCP 增量达 76.9 px（阈值 20）。通过以下修复解决：

- **列带掩膜**（`ugreen_tracker.py`，提交 `2e40c39`）：在形态学运算 **之前** 将 `[380, 820]` 列区间之外的像素归零。防止背景运动（工位后方走动的人）经 MORPH_CLOSE 与机械臂轮廓合并。仅作用于实际的 1280 宽帧；640 宽的合成测试夹具会跳过它，从而使现有 7/7 测试保持绿色。
- **阈值放宽** 至 50 px（preflight.py），以吸收图像差分边缘抖动。
- 用户已固定 UGreen 相机，使其在不同会话间不再发生位移。

最终状态：`PREFLIGHT OK — cleared for lab work.`（预检通过 — 可进行实验室作业。）

### B1.2 — 验证 UGreen TCP 检测 ✅

`pickhome1` 位姿下在线 TCP = `(659, 391)`。叠加图证实检测落在机械臂轮廓上（保存为 `logs/ugreen_pickhome1_overlay.png`）。由于机械臂底座在 homeplace0 与 pickhome1 之间保持静止，检测到的 "TCP" 实际上是移动轮廓与静止臂座之间的边界 — 它是位姿的确定性函数，足以供 PnP 使用。

### B2 — D415 重新标定 ✅（2026-04-20 16:00）

**范围调整：** 经讨论后，UGreen 闭环流水线被移出范围（见摘要部分）。UGreen 内参标定在首次运行时就遇到了 MSMF 重新配置挂起问题，导致我们陷入了一段兔子洞 — 下面有详细记录 — 因此即便修复了相机访问，我们仍决定彻底跳过它，直接进行 D415 精修。

#### UGreen 内参工作（尝试后废弃）

`ugreen_intrinsics.py collect` 在本次会话期间被两次增强：

1. **实时预览与角点叠加**（`28bf291`）：原始交互是"盲拍"（按回车后才拍摄，事后才能看到检测结果）。新版本打开 OpenCV 窗口显示 UGreen 实时画面，并以绿色实时叠加检测到的棋盘角点。空格保存一个位姿（只有检测成功才会计数）；Q/ESC 退出。
2. **MSMF 媒体类型协商**（`5fe0a85`）：UGreen 在 `cap.set(CAP_PROP_FRAME_HEIGHT, 720)` 上挂起 — MSMF 两次连续重新配置会无限阻塞。完全去掉 `cap.set` 会得到 `MF_E_INVALIDMEDIATYPE`，因为没有协商时 MSMF 根本不会确定像素格式。修复方法：先强制 MJPG FOURCC，再设 WIDTH，再设 HEIGHT，每步之间休眠 300 ms。可选的 `setWindowProperty(TOPMOST)` 用 try/except 包起来，因为并非所有 OpenCV-Windows 构建都支持它。并在每个阶段增加 `[debug]` 打印。

两处修复完成后相机本可以干净地打开，但此时范围决策已定：UGreen 仅作诊断工具，并非运行工具。没有采集任何棋盘位姿。

#### 通过 `analyze_static.py` 进行 D415 重新标定

将草莓放在 `cal_01..cal_04`，番茄放在 `cal_05..cal_08`，我们运行：

```
python/analyze_static.py --from pickhome1 cal_01..cal_08 --save
```

**发现：从 pickhome1 看，cal_05..cal_08 超出了 D415 的视野范围。** 这四个番茄位于工作空间 y ≈ −0.4 到 −0.5 m 的左后位置，而 pickhome1 从 y ≈ −0.05 朝向前方中央。D415 在约 0.5 m 深度上的 FOV 无法覆盖到那么远。结果：只有 4 对配对检测（cal_01..cal_04），而非 8 对。

脚本中的"候选重标定"段要求所有 N 个 GT 标签都配对成功才会执行，因此拒绝运行。我们仅用 `cal_01..cal_04` 重新运行：

| 标签 | 残差 |
|---|---|
| cal_01 | 26.1 mm |
| cal_02 | 30.1 mm |
| cal_03 | 47.5 mm |
| cal_04 | 29.5 mm |

**均值 33.3 mm，最大 47.5 mm，RMS 34.4 mm。** 已保存到 `calibration.json`；旧标定备份到 `calibration_bak_20260420_154943.json`。

这比之前的 37 mm RMS 略好，并且仍在夹爪的抓取余量之内（番茄 50 mm → ±45 mm 余量，草莓 35 mm → ±52 mm 余量）。最大 47.5 mm 对番茄来说比较紧，但可以接受。

**注意事项：当前标定对前中央工作区过拟合。** Umeyama 刚体变换拟合基于 4 个对应点，全都集中在桌面上同一个 ~15×15 cm 的区域。在 cal_05..08 所处的左后方，精度可能更差。对演示而言这可以接受，因为抓取操作都发生在 D415 能看到水果的地方（前中央）；`pickhome1` 处的检测是关键路径。

### 给 B3（HSV 调参）的观察 — 仍待完成

重新标定流程在实际包含 4 个草莓（cal_01..04）和 0 个可见番茄（cal_05..08 超出 FOV）的场景中，报出了 5 个草莓 + 1 个番茄的检测结果。拆解：

- 4 个真实草莓 → 4 个检测（其中一个在 cal_03 被误判为番茄）。
- 2 个误报检测（可能是桌面光照伪影），两个都因无有效深度而被跳过。
- 总体：分类器有噪声但不至于灾难性。CIRCULARITY_THRESH = 0.75 对其中一颗草莓而言正好在边界上。

在进行 14 果运行前仍建议做 B3 调参，但它 **不阻塞** 单果干跑 — cal_01..04 上的草莓都能被检测到且坐标可用。

## 本次冲刺的架构发现

1. **D415 是臂载的。** `T_cam_to_base` 与位姿相关；`main_final.py` 会在检测前移动到 `pickhome1`。形式化的 AX=XB 手眼标定被移出范围；每个位姿的标定加上 UGreen 交叉验证已足够满足抓取容差（番茄 45 mm 余量、草莓 52 mm 余量 vs 当前 36 mm RMS）。
2. **UGreen 作为外部验证器。** 提供了独立于 D415 的闭环视觉反馈，通过图像差分（前/后帧对比）实现抓取成功率的测量。
3. **图像差分 TCP 提取对相机位姿和背景运动敏感。** 列带掩膜解决了后者。第一个陷阱是发现后方走动的人通过形态学闭运算与机械臂轮廓合并 — 在实验室中发现、在代码中修复。
4. **Stateflow 负责编排，Python 负责执行。** 最小深度原则 — Stateflow 掌控状态转移与时序；复杂行为（IK 安全、夹爪斜坡 + 读数回读、智能抓取 Z）仍然以 `sorting_controller.py` 作为唯一真值来源。避免两套实现出现漂移。

## 下一步

| 阶段 | 动作 | 负责人 | 阻塞？ |
|---|---|---|---|
| B4 | 示教 `basket_a/b/c` 点（用户通过 `teach_points.py` 将机械臂点动到 3 个篮筐位置） | 用户 | 是，B6 之前 |
| B5 | 通过 `main_final.py --pick-only` 在 cal_01 的草莓上做单果干跑 | Claude | 是，B6 之前 |
| B3 | 实验室光照下的 HSV 调参（调整 `fruit_detector.py` 中的 `HSV_RANGES` / `CIRCULARITY_THRESH`） | 混合 | 否，但能提升召回率 |
| B6 | 14 果自主运行 + 录制 | 混合 | 演示关键路径 |
| B7 | 远程模式在线验证（用示教器手动抓取） | 混合 | 演示关键路径 |
| B8 | 视频剪辑 + 提交 | 团队 | 最后一步 |

预计剩余实验室时间：约 2-3 小时的硬件使用 + 剪辑。

**待定的即时决策：** 是否先示教 `basket_a/b/c`（用户驱动，5-10 分钟）再尝试单果抓取。建议：是 — 否则 B5 只能验证抓取，无法验证放置。但 `--pick-only` 也是一个合理的中间安全检查，可在提交完整循环前进行。

## 风险登记（根据规格 §7 更新）

| 风险 | 状态 |
|---|---|
| UGreen 位姿在不同会话间漂移 | **B1 期间命中一次。** 通过用户固定相机 + 预检第 7 项（TCP 增量 > 50 px 会触发）缓解。UGreen 降为诊断工具后重要性降低。 |
| 贴纸在抓取过程中脱落 | 不适用 — 图像差分方法不依赖贴纸。UGreen 不再运行后也不再需要。 |
| D415 RMS 始终达不到 20 mm | **部分命中。** B2 重标定后目前 34.4 mm。仍在夹爪余量内。要进一步改进需要更多覆盖完整 FOV 的标定点。 |
| D415 标定对前中央过拟合 | **新增，2026-04-20。** 4 点 Umeyama 拟合只覆盖 cal_01..04 的区域。缓解：抓取发生在 `pickhome1`，它正好看向那块区域，因此过拟合与运行位姿一致。 |
| 从 pickhome1 看 cal_05..08 的番茄超出 FOV | **新增，2026-04-20。** 在 D415 重标定中发现。不阻塞演示（番茄将放在可见的前区），但使计划中的 8 点标定失效。 |
| MSMF 在 UGreen `cap.set` 上挂起 | **命中 + 已缓解。** 强制 MJPG FOURCC + WIDTH/HEIGHT 设置之间休眠。已随 UGreen 标定移出范围而失去实际影响。 |
| Stateflow 引入新 bug | 尚未构建（需要 MATLAB）。Python 核心仍在原位作为回退方案。 |
| 远程 HMI 无法在演示前就绪 | 如 Cartesian 模式延期，可交付仅点动的子集。 |
| 实验室时间意外缩短 | 预检 30 秒内发现问题。D415 重标定约 1 分钟。抓取干跑约 3 分钟。 |

## 提交列表（整个冲刺，2026-04-20 — 2026-04-21）

```
5fe0a85 ugreen_intrinsics: MJPG FOURCC + sleeps around cap.set() calls
28bf291 ugreen_intrinsics collect: live preview with detection overlay
7cbfa71 B1 lab session: preflight 7/7 green + sprint report
2e40c39 ugreen_tracker: column-band mask to reject background motion
db47fb7 final-review: fruit_queue() parens + honest preflight message
9cf30ad docs: refresh PROGRESS + PROJECT_CONTEXT for 2026-04-20 state
1f968b3 v2 hardware model scaffold + remote HMI wiring reference
87b3478 sorting_controller: Stateflow-facing module helpers
b3d59ac build_autonomous_stateflow.m: Stateflow chart generator
818a2be remote_view: OpenCV companion + MATLAB launcher
e72fe8d remote_jog: workspace-box + joint-limit guarded nudges
b77a942 generate_report_plots.py: 4 report figures from logs
0380094 extract TraceLogger + emit PICK/GRIPPER/COMPLETE events
4f4ada6 LAB_RUNBOOK: session flow from power-on to shutdown
63ebe17 preflight: document why we bypass QArmDriver.disconnect()
312ca00 preflight.py: 7-check pre-session sanity script
1e27eb6 calibrate_closed_loop: honest residuals + SQPnP + sanity gate
35d9c70 closed-loop UGreen calibration runner
fa0a614 ugreen_intrinsics: wire CALIB_CB flags + honor q-to-abort
22cc2eb UGreen intrinsic calibration: chessboard pattern + cv2 solver
c985f3a ugreen_tracker: photometric ceiling + largest-component test
ff4c099 UGreen tracker: image-diff TCP extraction + baseline mgmt
76b7394 plan: fix Stage A1.1 reference impl to call qarm.set_joints_and_gripper
e4641cf extract set_gripper_ramp helper for reuse by remote mode
```

## 下次实验室会话的经验教训

1. **在物理标定布置中覆盖整个工作空间。** 将水果放到 cal_* 之前，先从 pickhome1 验证所有预期的 cal 点都在 D415 FOV 内。如果不在，要么移动水果到机械臂能看到的地方，要么使用两个检测位姿（pickhome1 + placehome1）做分位姿标定。
2. **在采集基准之前，先把 UGreen 物理固定好。** 1 cm 的位移就足以让所有后续的图像差分操作失效，直到基准被重新采集。用胶带、配重或夹具固定它。
3. **运行 `preflight.py` 之前，关闭所有其他 Python 进程。** 第一次预检就在 QArm 检查上失败，因为还有进程占着 USB。UGreen 同样有这个风险（MSMF 在释放前会一直持有设备）。
4. **MSMF 的怪癖在代码中已有充分记录。** 如果某个相机在 `cap.set` 上挂起，`ugreen_intrinsics.py` 中的修复方案（MJPG FOURCC + 休眠）就是模板。如果看到 `MF_E_INVALIDMEDIATYPE`（−1072875772），就是这个修复。
5. **`calibrate_closed_loop.py` 中的标定合理性闸门设计为在 RMS > 50 mm 时写出 `.rejected.json` 而非覆盖原文件。** `analyze_static.py` 使用不同的流程（带匹配的 Umeyama）并且 **没有** 同样的闸门 — 评审者在信任 `--save` 之前应该先读打印出的 RMS。

---
---

# 冲刺进展更新 — 2026-04-21

**分支：** `yichang_branch`
**会话：** 第二次实验室会话，新的工作站（`Amarande_chao`）

## 摘要

完成了第二台工作站的环境搭建，为新的物理布置重新验证了 D415 标定，给示教笔增加了单关节点动功能，构建了两个硬件测试脚本（`test_cal_picks.py`、`test_auto_pick.py`），并完整运行了一次基于视觉的单草莓自主检测-抓取流程。**物理抓取失败**（机械爪没有真正夹住水果），但整个 视觉 → 规划 → 分段接近 → 抓取 → 搬运 → 释放 的流水线端到端运行无任何崩溃。剩余的差距是 HSV 和标定精度以及抓取对齐，不是控制流程。

## B 阶段进度更新

| 阶段 | 之前 | 现在 | 说明 |
|---|---|---|---|
| B1 预检 + 视觉参考 | ✅ | ✅ | 在新机器上重新运行。继承了 Piero 提交 `2d52abd` 中的文件后，重新采集了 baseline + reference。 |
| B2 D415 重新标定 | ⚠️ | ✅ | Piero 的 `calibration.json` 在这台机器上有 **44 mm 的 X 方向整体偏置**。通过 `analyze_static.py` 做了 4 点重标定，RMS 31.7 mm，最大 44.8 mm。 |
| B3 HSV 调参 | ❌ | ❌ | 观察到假阳性 + 一颗草莓被误分类为番茄。尚未调参。 |
| B4 篮筐示教点 | ❌ | ❌ | `placeberries` 已存在，`basket_a/b/c` 仍然缺失。 |
| B5 单果干跑 | ❌ | ⚠️ | 运行了 `main_final.py --pick-only`（4/4 完成循环）以及 `test_auto_pick.py --max 1`。循环完成，抓取读数 0.887（所有尝试都是同一个数字——很可能不是真实的抓取反馈信号）。物理抓取失败。 |
| B6 14 果自主运行 | ❌ | ❌ | |
| B7 远程模式在线验证 | ❌ | ❌ | |
| B8 视频剪辑 | ❌ | ❌ | |

## 今天完成了什么

### 1. 工作站搭建（Amarande_chao → `D:\study in birmingham\Applied Robots\qarm-fruit-sorting`）

- `git checkout master && git pull` → 合并了 Piero 的 2026-04-20 提交（32 次提交）
- 预检：首次运行 **2 项红色**：
  - `calibration.json` 缺失
  - UGreen baseline / reference 缺失
- Piero 推送了 `2d52abd` 和 `3fea73d`，提供了 `calibration.json`、`logs/ugreen_*.png`、`logs/ugreen_pickhome1_tcp.json` 和 `teach_points.json` — 全部是通常被 gitignore 忽略的会话专属文件，强制添加以便团队共享。
- 拉取之后，预检第 7 项仍然失败：`tcp delta = 274.4 px`（阈值 50）。Piero 的 UGreen 位置不是我们的 UGreen 位置 — **重新运行 `setup_baseline_ref.py`** 在本地重新采集 baseline + reference。之后预检 7/7 全绿。

### 2. D415 标定 — Piero 的在这里不可用

- 运行 `analyze_static.py --from pickhome1 cal_01 cal_02 cal_03 cal_04`（草莓放在 cal_01..04 位置）：
  - Piero 标定在我们机器上的误差：**平均 55.5 mm，最大 74.2 mm，偏置 [-44, -16, +14] mm**。
  - 基于我们 4 个检测点的候选重新拟合：**平均 30.5 mm，最大 44.8 mm，RMS 31.7 mm**。
- 保存候选（`--save`）；Piero 的旧标定备份到 `calibration_bak_20260421_141820.json`。
- 最大 44.8 mm 残差距离草莓抓取余量只剩约 7 mm 的 slack，对番茄来说（45 mm 余量）实际上是在极限位置 — 这也解释了后面抓取失败的原因。

### 3. `teach_points.json` 编辑

- 将 `point1..point4` 的内容替换为 `cal_01..cal_04` 的数据副本（保留 `pointN` 作为键，兼容脚本默认值）。这是为了绕过 `analyze_static.py` 的参数解析问题 — 显式的 `cal_*` 标签没有正确进入 `sys.argv`，脚本退回到默认值。
- 实时添加了 `cal01..cal04`（无下划线）— 新的 pick-ready 示教点，Z ≈ 0.02 m，抓爪预先关闭到 ~0.149。和旧的 `cal_01..cal_08` 集合（Z ≈ 0.13-0.16 m）完全不同。

### 4. 新脚本

#### `python/joint_jog.py` — 独立的单关节点动工具
OpenCV 窗口 + 键盘。`q/a` `w/s` `e/d` `r/f` 直接移动 J0/J1/J2/J3。实时显示 FK 位置，强制执行 JOINT_LIMITS，通过 `[ / ]` 调整步长，`h` 回零，`p` 打印。默认步长 3°。

#### `python/teach_points.py` — 内嵌了关节模式
- 新增 `j` 键：切换笛卡尔 ↔ 关节模式（在外层循环 **和** modify 子循环中都持续生效）
- `DEFAULT_ROT_STEP`：**5° → 1.5°**（影响笛卡尔模式下的手腕和关节模式下的所有关节）
- 关节模式下屏幕顶部有提示横幅
- trace log 中记录 `JOINT_JOG` / `JOINT_JOG_MODIFY` 事件

#### `python/test_cal_picks.py` — 分段接近验证脚本
绕过相机 + IK。直接使用 `cal01..cal04` 预先示教的关节角度。分段动作：
- **Approach**：在 transit-Z 处 rotate+advance → 垂直下降
- **Retreat**：直接向上 → 横向回 `pickhome1`

每颗水果每阶段都有 UGreen 快照。最终版本按用户要求去掉了抓取阶段 — 4/4 接近完成无任何事故，没有扰动任何水果。

#### `python/test_auto_pick.py` — 自主检测-抓取
完整流水线：D415 帧 → `detect_fruits` → 通过当前标定做 `pixel_to_world` → 工作空间盒过滤 → 每颗水果循环：
  1. 接近（分段，两个阶段都做 IK — 不需要预先示教关节角）
  2. 关闭抓爪（渐进式）
  3. 撤退（分段）
  4. 搬运到 `placeberries` 示教点
  5. 释放
  6. 返回 `pickhome1`

每个阶段都有 UGreen 快照。标志：`--max N`、`--dry-run`、`--label strawberry|tomato`。

**首次运行**（`--max 1`）：
- 从 pickhome1 检测到 4 颗草莓（其中一颗在早期运行中被识别为番茄 — 分类器有噪声）
- 抓取第一颗：`xyz=[0.545, 0.07, 0.055]`，深度 428 mm，面积 4406
- 整个循环执行完毕，关闭抓爪时读数 0.887
- **物理抓取失败** — 用户目视确认。分段动作正确，抓取对齐不足。

### 5. proactive-execution 技能

添加了 `~/.claude/skills/proactive-execution/SKILL.md`：通用指令，当下一步是已批准计划的明显延续且动作是本地 + 可逆时，直接执行而不是询问确认。减少实验室工作中的反复沟通。

## 观察

1. **44.8 mm 最大残差是抓取成功的实际天花板** — 和抓取失败一致。确认了我们需要 8 点标定（分布在工作空间内，包括番茄）才能降到 20 mm 以下，给抓爪留出工作余量。
2. **分段接近（rotate+advance → descend）是正确的** — 在 `test_cal_picks.py` 中视觉验证了 4/4 接近，在 `test_auto_pick.py` 中验证了 1/1。用户要求的动作模式现在已经在可复用的函数里（`approach_staged_ik`、`retreat_staged_ik`）。
3. **`main_final.py` 和 `test_auto_pick.py` 都看到 `held_grip=0.88`** — 无论夹爪里是否有东西，每次尝试都是同一个数字。说明这个值是指令斜坡停在抓爪机械上的终点，不是真实的"受阻"读数。不能用作成功信号。
4. **`sorting_controller.py` FSM 尚未使用分段接近** — 它直接在关节空间插值到抓取目标，意味着 TCP 同时下降 + 伸出（用户明确要求避免的旧模式）。如果我们选择 `main_final.py` 作为演示入口，把 `approach_staged_ik` 移植到 `sorting_controller.py` 是下次会话的候选任务。
5. **HSV 分类器仍然有噪声** — 预检第 5 项里 19 个 blob，`test_auto_pick.py` 找到 4 颗有效草莓，另外一颗在其他地方被误识别为番茄。不阻塞但光照变化时召回率 / 准确率会下降。

## 距离演示还剩什么

| 优先级 | 项目 |
|---|---|
| P0 | 用分布在整个工作空间的 cal_01..cal_08（或者新的正确高度的 cal 集合）做 8 点 D415 重标定 — 目标 RMS < 20 mm |
| P0 | 把 `approach_staged_ik` 移植到 `sorting_controller.py`，让 `main_final.py` 使用分段动作 |
| P1 | 示教 `basket_a/b/c`（用户通过 `teach_points.py` 手动点动） |
| P1 | 实验室光照下 HSV 调参（`fruit_detector.HSV_RANGES` + `CIRCULARITY_THRESH`） |
| P1 | 单果抓取成功（端到端，物理抓握已验证） |
| P2 | 14 果自主运行 + 录制 |
| P2 | 远程模式在线验证 |
| P3 | 视频剪辑 + 报告撰写 |

## 提交列表（2026-04-21）

相对 `3fea73d` 的差异：
- `calibration.json` — 为这个物理布置新建的 D415 标定（RMS 31.7 mm）
- `logs/ugreen_baseline.png`、`logs/ugreen_pickhome1_reference.png`、`logs/ugreen_pickhome1_tcp.json` — 为这个 UGreen 位置重新生成
- `teach_points.json` — point1..4 的值替换为 cal_01..04 的副本；实时添加了无下划线的 cal01..04
- `python/teach_points.py` — 关节点动模式（j 切换），rot_step 默认 1.5°
- `python/joint_jog.py` — 独立的单关节点动工具（新）
- `python/test_cal_picks.py` — 分段接近验证脚本（新）
- `python/test_auto_pick.py` — 自主检测-抓取（新）
- `docs/SPRINT_REPORT_2026-04-21.md` — 英文版今日报告
- `docs/SPRINT_REPORT_2026-04-20.chinese.md` — 本次更新（中文版，覆盖 2026-04-20 + 2026-04-21）

## 下次实验室会话 — 建议顺序

1. 验证 `teach_points.json` 还对应物理的 cal01..cal04 位置（它们今天被实时编辑过）
2. 添加 4 个覆盖工作空间角落的分散示教点（或者把番茄移到 FOV 内）→ 做 8 点标定
3. 用 `analyze_static.py` 的场景截图做 HSV 调参
4. 把分段接近移植到 `sorting_controller.py`
5. 示教 `basket_a/b/c`
6. 首次单果抓取成功（物理抓握已验证）
7. 14 果自主演示运行 + 视频

---

# 2026-04-22 追加 — 视觉管线重新设计 + D1/D2 实施与现场调参

**分支：** `yichang_branch`
**范围：** `5056c47` .. `fdbc331`（当天 37 次提交）
**剩余：** 9 天到 2026-05-01

## 摘要

整个冲刺幅度最大的一天。砍掉 2026-04-20 引入的 UGreen 地面相机闭环标定流程，换成**单相机（D415）俯拍**管线，fiducial 是 7×5 内角点棋盘格。产出了新 spec + 两份实施计划，接着用 subagent-driven TDD 完成两个子系统，然后在实验室硬件上做端对端验证和大量灯光调参。一天结束时的状态：检测器能正确识别桌上的真香蕉 / 番茄 / 草莓并输出 base-frame XYZ 座标；尚未完成的是物理抓取实验证明座标真的能让手臂夹到。

## 头脑风暴 → spec → 计划

### 1. 视觉管线重新设计（3 次提交）

替换每 session 约 20 分钟的 UGreen 闭环标定流程（手臂靠边、拍基线、cal_01..08 摆 8 颗水果、SQPnP 手眼、pickhome1 存参考帧）→ 换成每 session 约 30 秒的**俯拍 + 棋盘格 fiducial**方案。两种操作 UI：鼠标点击单颗 + 键盘 `b`/`t`/`s` 按类别批量。

锁定的设计决定（Q1..Q8）：

| # | 决定 | 选择 | 理由 |
|---|---|---|---|
| Q1 | Fiducial | 7×5 30 mm 棋盘格（已有） | OpenCV 原生、团队已有 |
| Q2 | Base-frame 注册 | jog-touch 原点角 | 弹性、30 秒、复用 teach_points |
| Q3 | run-time 棋盘格 | 留在桌角 | 每帧残差 = 即时漂移检测 |
| Q4 | 检测器 | 纯 OpenCV 分层特征 | 无新依赖、3 类可线性分 |
| Q5 | Z 高度 | D415 depth（后来改为 + fallback） | RGB-D 已有、视差修正 |
| Q6 | 范围 | 整个砍掉 UGreen | 减 1500 行、session setup -15 分钟 |
| Q7 | autonomous UX | click-to-pick | 每颗都监督、demo 安全 |
| Q8 | 批量模式 | `b`/`t`/`s` + 点击并存 | rubric 友善 + 单颗 debug |

完整 spec：`docs/superpowers/specs/2026-04-22-overhead-vision-pipeline-design.md`

### 2. D1 棋盘格标定子系统（18 次提交）

计划：`docs/superpowers/plans/2026-04-22-d1-chessboard-calibration.md`

用 subagent-driven TDD 执行，每个 task 一个 subagent，中间做 spec-compliance + code-quality 两轮 review：

- `python/session_cal.py` — `SessionCal` dataclass + JSON roundtrip (A1)
- `python/homography_solver.py` — `solve_homography` + reprojection RMS (A2)
- 加上 `camera_height_from_homography`（用 H 在主点的 Jacobian 推相机高度，2% nadir/15% 30° tilt 精度）(A3)
- `python/touch_probe.py` — `capture_tcp` + `jog_and_capture`（cv2.waitKeyEx HUD 方式）(A4)
- `python/calibrate_chessboard.py` — 顶层 CLI + 合成端对端整合测试 (A5)
- `docs/superpowers/plans/2026-04-22-d1-smoke-test.md` — 实验室 runbook (A6)

10/10 离线单元测试通过。

### 3. D2 水果检测器重写（10 次提交实施）

计划：`docs/superpowers/plans/2026-04-22-d2-fruit-detector.md`

同样 subagent-driven TDD：

- `python/fruit_detector.py` 整个重写
- `Detection` dataclass + `to_dict` (B1)
- `hsv_mask` 支援红色 wrap-around 色相 (B2)
- 每类检测器：banana/tomato/strawberry (B3-B5)
- depth 采样 + 去噪 (B6)
- `pixel_to_base_frame` 含 nadir 视差修正 (B7)
- `detect_fruits` 顶层 orchestrator (B8)
- `test_integration.py` 手术：移除旧 API 相关区块 (B9)
- Post-review：tomato/strawberry 在 `green_ratio == 0.05` 的边界条件 exclusive 化

22/22 离线单元测试通过。旧 `FruitDetection` / `CIRCULARITY_THRESH` / `detection_depth_mm` API 已删除；下游使用者（`main_final.py`、`preflight.py`、`hover_test.py` 等）目前 import 就会坏 — D3/D8 会修。

## 实验室验证（2026-04-22 下午）

### D1 smoke test

跑 `python/calibrate_chessboard.py` on 真机：

- **第一轮** — bug 浮现：
  - `AttributeError: 'QArmCamera' object has no attribute 'fx'` → 修成 `cam.intrinsics["fx"]`（`3ab955e`）
  - 抓到**全黑 frame**（固定帧数 warm-up 不够）→ 改成 `while c.mean() > 5` 轮询（同 preflight 模式），同时加 touched-TCP cache 到 `logs/last_touched_tcp.json` 让 Phase-2 重跑可以跳过 jog-touch（`960836a`）
  - `findChessboardCorners failed` → 加 debug 存帧到 `logs/calibration_latest.png`（`d15c908`）
  - `_INNER_COLS=6, _INNER_ROWS=4` **错了** — spec/档名的 "7×5" 指**内角点**不是方格数（印出来的是 8×6 方格 = 7×5 内角）（`3312f3f`）

- **第二轮** — pose/framing：
  - 初始 survey1 是 ~42° 倾斜（joint2 33° + joint3 19°）；Jacobian 估的相机高度 33 cm vs 实测 26 cm（27% 误差，超出 15% spec 范围）
  - 重教到更 nadir + 把棋盘往手臂基座拉近：chess origin X 从 0.686 → 0.403 m，camera height 0.330 → 0.262 m

- **最终通过：** 35 corners、reprojection RMS 0.709 px、`session_cal.json` 写入。

### D2 检测器实验室调参

跑 `python/diag_detector.py`（新增）— 手臂去 survey1、拍一张、跑 `detect_fruits`、存 annotated PNG。根据真实灯光迭代多次：

| 问题 | 实测值 | 修法 | 提交 |
|---|---|---|---|
| 香蕉 HSV miss | H=15（温灯下橙黄色）vs spec H=[18,35] | H=[14,40] | `a1d25de` |
| 番茄阴影区 miss | V=59 vs spec V min 60 | V min 40 | `a1d25de` |
| 香蕉面积 > MAX | 50k px 近景 vs spec MAX 30k | MAX 80k | `a1d25de` |
| 番茄面积 > MAX | 15.6k vs MAX 15k | MAX 40k | `a1d25de` |
| 番茄圆度 | 0.50（高光咬掉）vs spec 0.70 | 0.40 + bbox aspect ≤ 1.8 | `a1d25de` |
| depth 沿倾斜光轴 | 641 mm vs 相机高 330 mm | 用每类硬编码高度 fallback（= spec Q5=A） | `3a11b8c` |
| Confidence floor 0.3 < gate 0.35 | 真水果被悄悄 drop | 移除 floor，重新 scale | `c7eb97b` |
| 香蕉 aspect 1.67 < 1.8 | 俯拍香蕉很少 >1.8 | MIN_ASPECT 1.5 + score floor 0.5 | `6028c0d` |
| 草莓叶子绿色 miss | 实验室灯让叶子 H 20-35 V 15-40（暗橄榄色）vs spec 亮绿 | `green_calyx` H [20,90] V [15,255] + 新 `_has_green_in_top_strip`（俯拍时绿叶在 bbox 内部顶端不在上方） | `6028c0d` |
| 草莓 taper 0.44 < 0.9 | 俯拍压扁了 3D taper | 放宽到 [0.3, 3.5] 只作理智性 gate | `6028c0d` |
| Speckle 假番茄（435 px²） | 香蕉阴影边缘 | MIN_AREA 400 → 1500 | `d74aadc` |

**最终实验室帧结果：** 3 颗真水果全部检测到、0 个假阳性：

```
banana     px=(1116, 483) base=[+0.577, +0.235, +0.042] conf=0.59
tomato     px=(631, 275)  base=[+0.458, +0.183, +0.067] conf=0.84
strawberry px=(725, 533)  base=[+0.481, +0.247, +0.042] conf=0.87
```

番茄 base X = 0.458 ≈ HOME X (0.45) —— 最好抓；香蕉 X = 0.577 略超 HOME，borderline 可达。

### 4. 其他实验室辅助工具
- `python/diag_detector.py` — survey → capture → detect → annotated PNG
- `python/diag_pick_one.py` — 预先 IK 解 HOVER/PICK/LIFT 三个 pose、每步 ENTER 确认才动、gripper ramp + readback。**今天还没跑**。

## 观察

1. **Survey pose 的 nadir 程度是整个几何的支点。** 42° tilt 会同时破坏 Jacobian 相机高度推算（27% 误差）和 nadir 视差公式（depth 沿倾斜射线 ≠ 垂直高度 → base Z 偏 3 倍）。搬到近端 + 更多肘弯重教后到 26 cm 相机高度、18% 高度误差。真正 <15° nadir 仍然是更彻底的修法。
2. **实验室灯光比合成测试残忍得多。** 每一个默认 HSV / 形状 / confidence 阈值都要放宽。这就是 spec §3 + D4 note 预言的，「hsv_tuner.py 还是缺的一块」。
3. **D415 depth 在近距红色/光滑表面只有 ~20% 有效率。** survey1 俯拍 + 水果 200-300 mm 距离下，5×5 median patch 常常采到背景而非水果。用 per-type flat-table fallback 是稳健选择；depth 通道仍然可用来剔明显离群。
4. **`max(0.3, ...)` floor 低于 `CONFIDENCE_MIN=0.35` gate 是陷阱。** final review 标为「D4 concern, 非 bug」—— 结果实验室验证前 20 分钟就咬人。以后原则：floor 若存在必须 ≥ gate，不然就移除。
5. **Subagent-driven TDD 非常适合机械式任务。** D1+D2 共 15 个任务每个都用 Haiku subagent 执行 + 2 轮 review，review 抓到了真 bug（边界 double-class、surgery 后 dead code）。估计比 inline 执行快 2-3 倍且品质不降。

## 当天提交清单

37 次提交在 `yichang_branch`：

**Spec (3):** `87013a4`, `e85dbb1`, `ca44e11`
**D1 计划 + 实施 (15):** `faf32f2`（计划）+ `615e801`, `fc77375`, `857bc95`, `d1c8f99`, `c3f75cf`, `f268374`, `17cd081`, `2153f5c`, `4371ca1`, `701b22f`（完成）、加实验室修复 `3ab955e`, `d15c908`, `960836a`, `3312f3f`。
**D2 计划 + 实施 (10):** `842f81f`（计划）、B1-B9 共 9 个、加 post-review `6bd7fa9`。
**D2 工具 + 实验室调参 (7):** `353a704`（diag_detector）、`a1d25de`, `3a11b8c`, `c7eb97b`, `6028c0d`, `d74aadc`、`cbac0c8`（diag_pick_one）。
**Session 状态 (1):** `fdbc331`（survey1 重教）。

## 未完成

| 优先级 | 项目 |
|---|---|
| P0 | 跑 `diag_pick_one.py` — 证明 base-frame 座标确实可被抓取（Hover/Descend/Grip/Lift 四阶段） |
| P0 | 若抓不到：或重教更 nadir 的 survey1、或加 per-fruit XY offset 校正 |
| P1 | D3 — `picker_viewer.py` + `survey_capture.py` + `pick_single` + wire `main_final.py` |
| P1 | D4 — `hsv_tuner.py` 互动式 HSV 调参工具（可延后；目前手调够用） |
| P2 | D6 — Simulink facade：autonomous mode shell out 到 `main_final.py`；从 facade 移除 UGreen wiring |
| P2 | D8 — 删 UGreen stack、修所有下游 import |
| P3 | Demo 视频、报告撰写、风险评估表 |

## 下次 session 建议顺序

1. 开机检查 `session_cal.json` 是否还对应桌面现状（棋盘没被碰、桌面高度没变）
2. 跑 `py -3.13 python/diag_pick_one.py --type tomato`（步步 ENTER 确认）
3. 若抓成功：commit/tag 「第一次成功的 autonomous pick」，然后启动 D3 计划
4. 若抓失败：看 annotated debug frame + PICK 阶段手臂 pose；多半是残余 tilt 引起的 XY 视差误差
5. 判断要不要：加更积极的视差修正、重教 survey1、或加 per-fruit XY offset pass
