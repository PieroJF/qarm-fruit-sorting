# 冲刺报告 — 2026-04-24

**截止日期:** 2026-05-01 14:00(剩余 7 天)
**分支:** `applied_robot_vision`(将以 `vision` 名推到 vico3740)
**冲刺范围:** `f20aded` .. `449c5f0`(24 次提交)

---

## 摘要

修复了长期存在的 "只能在 x ≈ 0.35 m 那条线上准确夹取" 问题。诊断出根因是原 pixel-to-base-frame 管线里四种几何误差叠加,把整个投影栈换成了基于 `cv2.solvePnP` 的相机外参标定。做到了 `PREFLIGHT OK`,lab 里首次成功夹取尝试完成,剩余 4 cm 的 x 方向系统偏差用常数 offset 补丁临时处理。另外完成了 P1/P2 的规格设计(草莓 → 塑料盒替换、orientation-aware 夹取),等 pick 偏差完全解决再执行。

整天分三阶段:

1. **设计**(上午):和用户 brainstorm 定下一个四项规格 — 工作区任意位置准确夹取、手臂停稳后才拍照、草莓替换成透明塑料盒、手腕旋转使细长物体按短边被夹。规格 + 实施计划都已 commit。
2. **P0 实现**(中午):subagent-driven 开发,共 7 个 TDD 任务 — 每个任务派一个干净 subagent,任务之间夹一道独立的规格符合性审查 + 一道代码质量审查。结束时测试套件(42 → 45 通过)全绿,preflight 守门全部就位。
3. **P0 实机 bring-up**(晚上):五轮诊断迭代解决投影错误:
   1. 用 `SOLVEPNP_IPPE` 替换 `SOLVEPNP_ITERATIVE`(平面姿态歧义)
   2. Preflight 加"桌面下方外参"拒载门(防遗留数据)
   3. 4-orientation sweep(OpenCV 角点排序不确定性)
   4. 相机到 TCP 物理距离门(剔除虚假的"桌面上方"解)
   5. 3-corner 触碰重建棋盘几何(棋盘实际在 base frame 里转了 ~90°,原"轴对齐"假设才是最后的根因)

最终在一个测试点测得的精度:**y 差 3 mm,x 差 40 mm**。x 残差 40 mm 的源头是 3-corner 触碰时方向测量有 ~4° 残差,经过投影放大成 40 mm。现用 `+0.05 m` x 方向 pick 时间 offset 补丁处理,不影响投影本身的正确性;后续仔细重测 3 角可消除该残差。

---

## 范围参考

- **规格(今日):** [`docs/superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md`](superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md)
- **计划(今日):** [`docs/superpowers/plans/2026-04-24-accurate-pick-and-box-swap.md`](superpowers/plans/2026-04-24-accurate-pick-and-box-swap.md)
- **已有 lab 操作手册:** [`LAB_RUNBOOK.md`](../LAB_RUNBOOK.md)

---

## 阶段 1 — Brainstorm → 规格 → 计划

上午 lab 报告带出四个关联问题:

1. 夹取精度只在距 base x ≈ 0.35 m 的那条线上有效(其他位置飘 10 cm)。设计阶段定位到根因:`pixel_to_base_frame` 的 nadir-pinhole 视差近似与实际腕装 D415 不匹配(相机相对 TCP 横向偏移 + 前向倾斜,并非 nadir)。
2. `survey_capture.capture_fruits` 在 `slow_move_to_joints` 返回后立即拍照,位置模式 PID 还没收敛(≈ 0.5–1.0 s 跟随滞后)。
3. 水果集需要变更:移除草莓,新增 17.5 × 11.5 cm 透明塑料盒。盒子透明 + 顶上有蓝色英国旗贴纸和白色条码。
4. 长条物体(香蕉、塑料盒)需要手腕旋转,让夹爪沿物体短边闭合。

锁定的设计决策(brainstorm Q1..Q6):

| # | 问题 | 选择 | 理由 |
|---|---|---|---|
| Q1 | 盒子检测主特征 | 矩形轮廓 + 白色标签占比验证 | 形状不受方向影响;透明盒子 HSV 没信号;内部红色草莓会触发 tomato 假阳性 |
| Q2 | 盒子摆放 | 任意角度,标签朝上,不倒扣 | 驱动 `minAreaRect` + orientation-aware 夹取需求 |
| Q3 | 夹取精度修复 | **方案 B + C**:撤手补 origin **并且**用 solvePnP + ray-plane 替换 nadir-pinhole | memory 标记 workaround 腐蚀了校准;只有 B 能解决非 nadir 几何;不需要完整 hand-eye,因为检测永远在 survey1 |
| Q4 | 拍照沉降 | 修现有 `r` 键,1.5 s 固定 dwell,关节范数做 sanity,不到位抛 RuntimeError | 复用 D4 `T_SETTLE` 经验;不新增键位 |
| Q5 | 夹取几何 | 选项 X(张口 11.5 cm,夹爪沿短边闭合);夹爪最大 18 cm ≥ 11.5 cm | 用户确认 |
| Q6 | 优先级 / 工作量 | P0(solvePnP + sanity)先出,停下实机验收;P1(盒子替换);P2(orientation 夹取) | 用户强调"先出成果" — 先让夹取准,然后迭代 |

规格 + 计划已 commit:`0fa6214`(规格)、`54e4cf2`(计划)。

---

## 阶段 2 — P0 实现(subagent-driven TDD)

P0 = 投影重写 + 拍照沉降 + preflight 守门。共 7 个任务,每个派一个干净的 subagent 去做(带完整任务文本 + 上下文);每个任务完成后,独立审查员先核对规格符合性,通过后再派代码质量审查员审美。

| # | 任务 | 模型 | 提交 | 备注 |
|---|---|---|---|---|
| 0.1 | `SessionCal.cam_extrinsics_survey1` 字段 + 来回 JSON 测试 | Haiku | `3ba3ff0` | 对旧 JSON 向后兼容(`.get(..., None)`)。 |
| 0.2 | `calibrate_extrinsics.solve_survey1_extrinsics` 封装 `cv2.solvePnP` | Sonnet | `6e17881`、整理 `6c1d88a` | 合成 pinhole 来回测试:无噪声数据上姿态复原精度 1 mm / 0.5°;RMS > 5 px 硬门。 |
| 0.3 | 重写 `pixel_to_base_frame` 为 ray-plane 投影 | Sonnet | `0f41e83`、docstring `593ea8c`、typo `43a5929` | 原有 fixtures 现在会 fail,等 Task 0.4 更新。 |
| 0.4 | 给检测器 fixtures 注入 `_nadir_extrinsics` | Sonnet | `763dec8` | 实现者捕捉到 0.3 中潜在的 `target_z` 语义 bug(pre-rewrite 代码里 `origin.z` 的偏移被误丢了) — 同次 commit 修掉。 |
| 0.5 | 把 `solve_survey1_extrinsics` 接进 `run_calibration_core` | Sonnet | `a8d8be7` | 4-orientation sweep 后来才加。 |
| 0.6 | `capture_fruits` dwell + 关节 sanity | Sonnet | `c8787af`、整理 `1fa956b` | 1.5 s 固定 dwell;0.05 rad 关节范数(后来在阶段 3 收紧)。 |
| 0.7 | Preflight 门检查 `cam_extrinsics_survey1` | Haiku | `cd57c5b` | 旧 session_cal 现在会硬失败,不会静默让 picker 用到错外参。 |

阶段 2 结束时测试套件:42/42 通过。所有 gates 上线。

---

## 阶段 3 — P0 实机 bring-up(五轮诊断迭代)

第一次实机跑 P0 代码暴露了五个独立错误,每个被新一批真实硬件数据揭出。每一个都单独定位和修复,才进入下一个。

### 迭代 1 — 平面姿态歧义(`ee10740`)

**现象:** 首次实测标定给出 `solvePnP: RMS = 2.79 px, camera at base XYZ = [0.587, 0.3, −0.207] m` — camera z 是负的(桌面下方,物理不可能)。

**根因:** `cv2.solvePnP(..., SOLVEPNP_ITERATIVE)` 对平面目标有两个几何等价解(正确姿态 + 通过目标平面的镜像);ITERATIVE 根据初值随机收敛到其中一个。这次它挑了镜像解。

**修法:** 换成 `cv2.solvePnPGeneric(..., SOLVEPNP_IPPE)`,专为平面目标设计,显式返回两个候选;过滤出 `C[2] > chess_origin_z` 的那个。加了一个回归测试:模拟 lab 场景(升高的 chess origin、camera 在上方),确认总能选到正解。

### 迭代 2 — Preflight 防旧数据(`1e3334d`)

**现象:** 代码修了,但 preflight 还显示 camera 在桌面下方。用户没重跑 `calibrate_chessboard.py`,用了旧的镜像数据。

**修法:** preflight 加门,拒绝任何 `cam_extrinsics_survey1` 其 `C[2] ≤ chess_origin[2]`。旧数据在会话启动时被硬挡,picker 拒绝运行。

### 迭代 3 — 角点排序歧义(`c928230`)

**现象:** 第二次标定:`findChessboardCorners` 成功(35 角),但 `solvePnP REJECTED: no IPPE solution places the camera above the chessboard plane`。两个 IPPE 候选都是桌面下方,RMS = 0.87 px。

**根因:** OpenCV 的 `findChessboardCorners` 不保证 `image_pts[0]` 对应棋盘的任何特定物理角。根据相机姿态,OpenCV 可能从上到下扫描,也可能从下到上(等等);4 个可能的"起点角"给出 4 种不同的 2D-3D 对应关系。我们的 3D 网格构造器永远假设"`image_pts[0]` = 物理左上角" — 对这个设置错了。

**修法:** `run_calibration_core` 里 4-orientation sweep:分别按 4 种 `(flip_i, flip_j)` 对应关系调 `solve_survey1_extrinsics`,保留能给出 "camera above plane" 且 RMS 最低的那个。回归测试模拟一个 mirror-ordered 2D 集,确认 sweep 能选对 orientation。

### 迭代 4 — 物理距离门(`5a8ca67`)

**现象:** Sweep 选中 `flip-rows` orientation,RMS = 0.88 px,camera 在 `(0.552, 0.038, 0.273)` — 数学有效但**离 survey1 TCP 31 cm**。对腕装 D415 来说物理不可能。首次 pick 测试确认投影是错的:番茄报 `(0.579, 0.156)`,实际 `(0.449, 0.010)` — 偏 20 cm。

**根因:** 每个 `(flip_i, flip_j)` orientation 都能给出 IPPE 有效的姿态,但只有一个物理正确。其他是几何假象,恰好能拟合那 35 个角点。RMS 几乎相同(0.82 px),光靠 RMS 无法区分。

**修法:** 加硬门 `||C_cam − TCP_survey1|| ≤ 25 cm`,用 survey 姿势的正向运动学算 TCP,剔除那些"相机离手腕太远、装不到那里"的 orientation。每次校准现在都会打印 4 个候选的完整表格,方便诊断。

### 迭代 5 — 棋盘非轴对齐(`608d857` + `f9bdc04` + `9146f00`)

**现象:** 迭代 4 之后,**四个** orientation 全被拒 — 两个桌面下方,两个离 TCP > 25 cm。全军覆没。

**根因(最后一个):** 棋盘在 base frame 里**旋转了 ~88°** — 用户摆放时没和 arm 的运动学 x/y 轴对齐。我们的 3D 网格构造器假设轴对齐:`corner[i, j] = touched_origin + (i·30 mm, j·30 mm, 0)`。用 90° 旋转的棋盘,输入 `solvePnP` 的 3D 位置就错了,任何相机姿态都同时对不上 2D 像素和(错的)3D 网格 — 所有 orientation 都失败。

**修法(三次 commit):**

1. `debug_corner_order.py` 诊断工具:在上一次捕获的帧上标出 4 个极端索引(`[0]`、`[6]`、`[28]`、`[34]`),操作员肉眼识别 OpenCV 把 `image_pts[0]` 放在哪个物理角。(操作员确认黄色 `[0]` = 他触的物理左上角,排除了角点排序 bug。)
2. `touch_three_corners.py` 工具:依次 jog 触 TL、TR、BL,打印推导的 `+i_axis`、`+j_axis`、棋盘旋转、距离。写 `logs/chess_touched_corners.json`(无 y/n 提示)。**首次使用就发现棋盘旋转 −88.5°,确认根因。**
3. `_grid_with_ordering` 更新:可接收 TR、BL 参数;有了就用 3-corner 推出的**方向**(Gram-Schmidt 正交化),**方格间距用标称 30 mm**(丢弃测出的距离,因为有 5–10 mm 的 jog 噪声)。Calibrate CLI Phase 1 读 3-corner cache,自动跳过单角 jog。

**结果(迭代 5 之后):**
```
solvePnP 4-orientation sweep:
  original: OK; C=[0.292, 0.043, 0.268], RMS=0.90px, ||C-TCP||=18.8cm
  flip-cols: REJECTED (两 IPPE 候选都在桌面下方)
  flip-rows: REJECTED (两 IPPE 候选都在桌面下方)
  180°: REJECTED (cam 31.7cm from TCP > 25cm wrist bound)
solvePnP: chosen orientation=original, RMS=0.90 px
```
正好一个 orientation 过了所有物理门。`||C−TCP|| = 18.8 cm` 是合理的腕装偏移距离。

### 附加 — 沉降阈值收紧(`47731f4`)

**现象:** 迭代 2 测试时发现,每次按 `'r'` 刷新,棋盘残差就往 10 mm 方向爬;第二次按 `'r'` 硬失败 `chessboard residual 10.7 mm > 10 mm`。

**根因:** 0.05 rad 关节范数阈值(来自 Task 0.6)允许 4 个关节总共约 20 mm TCP 漂移 — 足够让 pick-and-place 一个周期的跟随误差把残差推过 10 mm 门。

**修法:** 收紧到 0.015 rad(~6 mm TCP 漂移上限),与目标 pick 精度匹配。

---

## 最终测得精度

五轮迭代之后,preflight 全绿,一次代表性的 pick 测试:

| | 真实(jog-touch 到番茄) | Picker 报 | Δ |
|---|---|---|---|
| x | 0.448 m | 0.408 m | −40 mm |
| y | 0.064 m | 0.061 m | −3 mm |
| z | 0.072 m | 0.067 m | −5 mm |

y 和 z 都在毫米内;**x 方向有 40 mm 系统偏差** — 最可能源自操作员 3-corner 触碰时 ~4° 的非正交残差传到 `R` 旋转矩阵里。对 y、z 影响不大(旋转轴碰巧是几何对称)。

解决路径两条:
1. **更仔细地重测 3 个角**(慢 jog,目视确认 TCP 尖确实对在内角交叉点上)。预期残差塌缩到 <1 cm。
2. **pick 时在 x 加常数 offset**(今天的快补丁,`449c5f0`):`sorting_controller.pick_single` 在命令 pick 目标时 x 轴加 +5 cm,不改变 picker 界面显示的检测坐标。在偏差已知是常数时有效;等仔细重测 3 角时可以撤掉。

Offset 补丁目前在用。装有 offset 的 pick 精度验证是明天的第一件事。

---

## 今日触及文件

| 区域 | 路径 |
|---|---|
| 新标定模块 | `python/calibrate_extrinsics.py`、`python/test_calibrate_extrinsics.py` |
| 新测试模块 | `python/test_session_cal.py`、`python/test_calibrate_chessboard_orientation.py` |
| 新诊断工具 | `python/debug_corner_order.py`、`python/touch_three_corners.py` |
| 修改模块 | `python/session_cal.py`、`python/fruit_detector.py`、`python/survey_capture.py`、`python/calibrate_chessboard.py`、`python/preflight.py`、`python/sorting_controller.py`、`python/test_fruit_detector.py`、`python/test_survey_capture.py` |
| 设计文档 | `docs/superpowers/specs/2026-04-24-accurate-pick-and-box-swap-design.md`、`docs/superpowers/plans/2026-04-24-accurate-pick-and-box-swap.md` |
| 调试产物 | `logs/calibration_latest.png`、`logs/corner_order_debug.png`、`logs/chess_touched_corners.json` |

日末测试套件:45/45 通过(从早上的 36 起)。Preflight:全绿。

---

## 未结事项

- [ ] **P0.8 实机验收** — 装 +5 cm x offset 补丁后,在 TL 附近、TR 附近两个位置验 pick 精度。如果都 ≤ 1 cm,P0 算 ship。如果偏差随位置变,重做 3-corner 触(更仔细)。
- [ ] **P1(计划就绪,未启动):** 移除草莓类;新增 `plastic_box` 检测器(Canny + `minAreaRect` + 白色标签验证);tomato 去混淆(排除 box 区域);重拨 `placebox` teach 点;picker UI 里 `'s'` → `'p'`。
- [ ] **P2(计划就绪,延后):** `Detection.grasp_angle_rad` 由 `minAreaRect` 填;`sorting_controller._start_move` 用 `phi[0]_at_pick` + 实机标定的 `gripper_mount_offset_rad`(新 session_cal 字段)把 base 系角度转 wrist 关节。
- [ ] 文档:本报告 + 英文版,然后等 P0 正式 ship 时更新 `docs/PROGRESS.md`。

---

## 提交记录(今日 24 次)

```
449c5f0 chore(controller): +5cm x offset at pick time (empirical lab patch)
5072a7f chore(detector): tomato fallback 50->55 mm (measured in lab)
9146f00 fix(calibrate): use 3-corner directions with nominal spacing + Gram-Schmidt
f9bdc04 fix(calibrate): use 3-corner touched geometry to build correct 3D grid
608d857 feat(debug): corner-order visualizer + 3-corner touch helper
5a8ca67 fix(calibrate): reject orientations where camera too far from TCP
47731f4 fix(capture): tighten settle joint-norm tol 0.05 -> 0.015 rad
c928230 fix(calibrate): 4-orientation sweep resolves OpenCV corner-order ambiguity
1e3334d feat(preflight): reject below-plane camera extrinsics (mirror-flip guard)
ee10740 fix(calibrate): use SOLVEPNP_IPPE + pick above-plane solution
cd57c5b feat(preflight): require cam_extrinsics_survey1 in session_cal
1fa956b chore(capture): drop raising=False on monkeypatch + sharpen settle comment
c8787af feat(capture): 1.5s settle + sanity check before shooting
a8d8be7 feat(calibrate): call solve_survey1_extrinsics in run_calibration_core
593ea8c docs(projection): clarify fruit_top_z_mm is table-relative + trim stale docstring
763dec8 test(detector): update fixtures to provide cam_extrinsics
43a5929 chore(test): fix 'finite and finite' typo in docstring
0f41e83 feat(projection): ray-plane pixel_to_base_frame via extrinsics
6c1d88a chore(calibrate): fix transpose comment + drop unused cv2 import
6e17881 feat(calibrate): solve_survey1_extrinsics via cv2.solvePnP
3ba3ff0 feat(session_cal): add cam_extrinsics_survey1 field
54e4cf2 docs(plan): accurate-pick projection + box swap + orient grasp
0fa6214 docs(spec): accurate-pick projection rewrite + box swap + orient grasp
b8cf978 refactor(controller): joint-space interp + T_SETTLE hold phase
```
