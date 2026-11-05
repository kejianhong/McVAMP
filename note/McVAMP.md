# McVAMP 论文精读：向量化投影与流形约束运动规划

> 论文：*Vectorizing Projection in Manifold-Constrained Motion Planning for Real-Time Whole-Body Control*
> Shrutheesh R. Iyer, I-Chia Chang, Andrew Z. Liu, Yan Gu, Zachary Kingston (Purdue University), arXiv:2604.13323
>
> 代码：本仓库 `McVAMP/`，其中 `vamp/`（分支 `stack-4-mcvamp`）是规划器与约束内核，`cricket/` 是生成机器人运动学/约束内核的代码生成器，`experiments/` 是论文实验的 C++/Python 复现代码。
>
> 本文所有公式均为 LaTeX；所有代码引用都给出仓库内相对路径与行号（以当前 checkout 为准）。

---

## 目录

- [McVAMP 论文精读：向量化投影与流形约束运动规划](#mcvamp-论文精读向量化投影与流形约束运动规划)
  - [目录](#目录)
  - [0. 一页速览](#0-一页速览)
  - [1. 问题定义与记号](#1-问题定义与记号)
    - [1.1 配置空间、约束与流形](#11-配置空间约束与流形)
    - [1.2 TSR 约束（论文式 (1)）](#12-tsr-约束论文式-1)
    - [1.3 代码里的数据结构](#13-代码里的数据结构)
  - [2. Methodology：数学原理、物理含义与代码实现](#2-methodology数学原理物理含义与代码实现)
    - [2.1 系统结构：约束是"局部规划器"](#21-系统结构约束是局部规划器)
    - [2.2 约束函数：TSR 位姿误差与铰链（hinge）](#22-约束函数tsr-位姿误差与铰链hinge)
      - [数学](#数学)
      - [物理含义](#物理含义)
      - [代码](#代码)
    - [2.3 投影 = 阻尼高斯–牛顿（Levenberg–Marquardt）](#23-投影--阻尼高斯牛顿levenbergmarquardt)
      - [数学](#数学-1)
      - [物理含义](#物理含义-1)
      - [收敛判据（代码）](#收敛判据代码)
      - [代码](#代码-1)
    - [2.4 多约束复合：循环投影](#24-多约束复合循环投影)
      - [数学](#数学-2)
      - [物理含义](#物理含义-2)
      - [代码](#代码-2)
    - [2.5 向量化：SIMD、SoA 与 rake](#25-向量化simdsoa-与-rake)
      - [数学](#数学-3)
      - [物理含义](#物理含义-3)
    - [2.6 两阶段约束扩展（Alg. 2 逐行对照）](#26-两阶段约束扩展alg-2-逐行对照)
      - [第一阶段：粒子投影（找落脚点）](#第一阶段粒子投影找落脚点)
      - [第二阶段：弦离散 + 并行投影 + 连续性检查](#第二阶段弦离散--并行投影--连续性检查)
      - [第二阶段（续）：递归加密到分辨率 $\\delta$](#第二阶段续递归加密到分辨率-delta)
      - [收尾](#收尾)
    - [2.7 追踪编译、可微 log 映射与手写 Cholesky](#27-追踪编译可微-log-映射与手写-cholesky)
      - [追踪编译（tracing compiler）](#追踪编译tracing-compiler)
      - [可微的 $\\log(R)^\\vee$](#可微的-logrvee)
      - [手写 Cholesky](#手写-cholesky)
    - [2.8 四个具体约束的数学（Digit 全身例子）](#28-四个具体约束的数学digit-全身例子)
      - [质心约束：数学与实现的一处差异](#质心约束数学与实现的一处差异)
      - [闭环连杆：为什么只用 1 行](#闭环连杆为什么只用-1-行)
      - [双手刚体约束的左右乘顺序](#双手刚体约束的左右乘顺序)
      - [维度账本](#维度账本)
    - [2.9 与采样规划器的集成（Alg. 1）](#29-与采样规划器的集成alg-1)
      - [数学 / 算法](#数学--算法)
      - [代码](#代码-3)
    - [2.10 复杂度与"为什么是 100×而不是 8×"](#210-复杂度与为什么是-100而不是-8)
    - [2.11 设计取舍一览](#211-设计取舍一览)
  - [3. Experimental evaluation：如何验证 Methodology](#3-experimental-evaluation如何验证-methodology)
    - [3.1 验证逻辑总表](#31-验证逻辑总表)
    - [3.2 度量口径与统计方式（代码级）](#32-度量口径与统计方式代码级)
    - [3.3 公平性控制与超参扫描](#33-公平性控制与超参扫描)
    - [3.4 实验 A：单臂线/平面四类约束（Fig. 5）](#34-实验-a单臂线平面四类约束fig-5)
    - [3.5 实验 B：迷宫（Table I）与 OMPL(comp.) 消融](#35-实验-b迷宫table-i与-omplcomp-消融)
    - [3.6 实验 C：双臂 14-DoF（Table II）与 1-step 消融](#36-实验-c双臂-14-doftable-ii与-1-step-消融)
    - [3.7 实验 D：Digit 全身 28-DoF（Fig. 7/8、Table III、真机）](#37-实验-ddigit-全身-28-doffig-78table-iii真机)
    - [3.8 实验 E：动态环境重规划](#38-实验-e动态环境重规划)
    - [3.9 主张 ↔ 证据对照](#39-主张--证据对照)
    - [3.10 本机复核测量](#310-本机复核测量)
    - [3.11 效度威胁与批判性阅读](#311-效度威胁与批判性阅读)
  - [4. 代码地图](#4-代码地图)
    - [4.1 规划器与约束（`vamp/`）](#41-规划器与约束vamp)
    - [4.2 代码生成（`cricket/`）](#42-代码生成cricket)
    - [4.3 实验（`experiments/`）](#43-实验experiments)
  - [5. 复现命令](#5-复现命令)
  - [6. 论文与代码的差异、阅读注记](#6-论文与代码的差异阅读注记)
  - [7. 一页公式汇总](#7-一页公式汇总)

---

## 0. 一页速览

**问题**：流形约束运动规划要求在整条轨迹上满足若干几何约束（末端贴在平面/直线上、双手保持相对位姿、人形保持平衡且双脚不滑动）。当前最优（state-of-the-art）的投影式规划器（OMPL / CBiRRT²）在 30 自由度人形上需要数十秒，无法用于实时场景。

**观察**：这类算法的瓶颈不在碰撞检测，而在**约束扩展步**——把一条几何测地线（geodesic）的离散点逐个投影回约束流形。VAMP 已经证明碰撞检测可以用 CPU SIMD 做到微秒级，但"投影"仍然是串行 Jacobian 迭代。

**核心思想**：把投影从"一次处理一个配置"改造成"一次处理 $n$ 个配置"的**批量算子**，并让"投影"与"校验"在同一批 lane 上融合：

$$
\underbrace{q \mapsto P_{\mathcal M}(q)}_{\text{串行迭代}}
\;\Longrightarrow\;
\underbrace{Q=[q^{(1)},\dots,q^{(n)}] \mapsto [P_{\mathcal M}(q^{(1)}),\dots,P_{\mathcal M}(q^{(n)})]}_{\text{SIMD 一条指令流，}n=8}
$$

配合三项工程：(i) 用**追踪编译**（Pinocchio + CppAD + CppADCodeGen）把约束函数、Jacobian 和一步 LM 求解都展开成无分支直线代码；(ii) **两阶段扩展**（先绕 $q_{steer}$ 撒粒子投影，再沿弦插值并行投影并递归加密）；(iii) **早退**（任一 lane 投影失败/漂移过大/碰撞，整条边立刻作废）。

**结果**：迷宫问题中位时间 $0.006\,$s（OMPL $17.3\,$s，约 $2900\times$），双臂 14-DoF 约 $1.4$–$3.5\,$ms，Digit 人形全身搬运 $5$–$16\,$ms，可在真机上以 20 Hz 重规划。

---

## 1. 问题定义与记号

### 1.1 配置空间、约束与流形

设机器人构型空间 $\mathcal C \subset \mathbb R^d$（$d$ 为驱动关节数加浮动基），$q\in\mathcal C$。第 $i$ 个约束是光滑映射

$$
F_i:\mathcal C\to\mathbb R^{k_i},\qquad
\mathcal M_i \;=\; F_i^{-1}(0)\;=\;\{\,q\in\mathcal C: F_i(q)=0\,\}.
$$

工程上放宽为容差形式 $\|F_i(q)\|\le \epsilon_i$，于是每个约束在 $\mathcal C$ 中切出一个 $(d-k_i)$ 维子流形。多个约束取交：

$$
\mathcal M=\bigcap_{i=1}^{m}\mathcal M_i,\qquad
\mathcal M_{free}=\mathcal M\cap \mathcal C_{free}.
$$

**规划问题**：给定 $q_a,q_b\in\mathcal M_{free}$，求连续映射

$$
\sigma:[0,1]\to\mathcal M_{free},\qquad \sigma(0)=q_a,\ \sigma(1)=q_b .
$$

**物理含义**：$\mathcal C$ 是"关节怎么动"的空间；$F_i$ 是"任务要求什么"的度量；$\mathcal M$ 是"既满足任务又不违反机构约束"的曲面。规划器要做的是在这个曲面上"走"，而不是在 $\mathbb R^d$ 里走。

### 1.2 TSR 约束（论文式 (1)）

论文采用 Berenson 等人的 Task Space Region 参数化。三个量：

- $T^r_w\in SE(3)$：参考系（物体系）在世界系中的位姿；
- $T^r_e\in SE(3)$：末端执行器上的**偏置系** $e$ 在末端系中的位姿（抓取位姿）；
- $B=[l,u]\subset\mathbb R^6$：六个位姿分量的允许区间（平移 3 + 旋转 3）。

给定 $q$，正运动学给出 $T^e_w(q)$，于是偏置系在参考系中的实际位姿为

$$
\boxed{\;T^{r}_{obj}(q)=\left(T^{r}_{w}\right)^{-1}\,T^{e}_{w}(q)\,\left(T^{r}_{e}\right)^{-1}\;}
\qquad\text{（论文式 (1) 的 }T^{w'}_s\text{）}
$$

误差取 $SE(3)$ 的对数坐标（平移 + 旋转轴角）：

$$
d(q)=\begin{bmatrix} t\!\left(T^{r}_{obj}\right)\\[2pt] \log\!\left(R^{r}_{obj}\right)^{\!\vee}\end{bmatrix}\in\mathbb R^{6},
\qquad
\log(R)^{\vee}=\frac{\theta}{2\sin\theta}\left(R-R^{\top}\right)^{\vee},\ \ \theta=\arccos\!\frac{\operatorname{tr}R-1}{2}.
$$

论文写作 $d_w=d(q)-B^w$ 并要求 $\|d_w\|\le\epsilon$；代码则采用更一般的**铰链（hinge）**写法，见 §2.2：

$$
e_j(q)=\min\!\big(d_j(q)-l_j,\,0\big)+\max\!\big(d_j(q)-u_j,\,0\big),\qquad
F(q)=\sum_{j=1}^{6}e_j(q)^2 .
$$

**物理含义**：$d$ 的前三维单位是米，后三维单位是弧度。所以区间 $[-0.001,0.001]$ 表示"位置误差不超过 1 mm"，$[-0.01,0.01]$ 表示"姿态误差不超过 0.01 rad ≈ 0.57°"。**区间宽度就是"松紧程度"**：宽度很大的分量是"自由方向"（slab），宽度接近 0 的分量是"硬约束"（等式约束）。这一句话直接解释了论文中所有实验的物理设定：

| 实验                      | 参数来源                     | 区间设置（$[l,u]$ 的宽度）                            | 等式行数 $k$ | 流形维数 $d-k$ |
| ------------------------- | ---------------------------- | ----------------------------------------------------- | ------------ | -------------- |
| 直线 LP（position only）  | `line_problems.json`         | $x,z$ ±2.5 mm；$y$ 自由 ±10.0；姿态被脚本放开到 ±1000 | 2            | $7-2=5$        |
| 直线 LPO（+ orientation） | `crrtc_spheres.cc`           | $x,z$ ±1 mm；$y$ 自由 ±10.01；姿态 ±0.01 rad          | 5            | $7-5=2$        |
| 平面 PP（position only）  | `plane_problems.json`        | $z$ ±0.1 mm；$x,y$ 自由 ±10.0；姿态被脚本放开到 ±1000 | 1            | $7-1=6$        |
| 平面 PPO（+ orientation） | 同上 + `--keep_orientation`  | $z$ ±0.1 mm；$x,y$ 自由；姿态 ±0.1 mrad               | 4            | $7-4=3$        |
| 迷宫（$x,y$ + yaw 自由）  | `benchmark_maze.cc`          | $z$ ±1 cm；roll、pitch ±0.01 rad；$x,y$、yaw 自由     | 3            | $7-3=4$        |
| 双臂保持刚体              | `bimanual_panda_planning.cc` | 6 个分量全部 ±1 mm / ±0.001 rad                       | 6            | $14-6=8$       |

> 说明：`benchmark_line_plane` 默认把姿态上下界改成 $\pm1000$，所以 Fig. 5 的 LP / PP 只约束位置（"position only"），带姿态的 LPO / PPO 需要 `--keep_orientation` 或像 `crrtc_spheres` 那样直接给窄区间。
>
> 代码位置：`vamp/src/impl/vamp/planning/constraints/manifold/task_space_constraint.hh:30-51`（`rTe`、`wTr`、`lower`、`upper` 按末端执行器逐个给出），`.../bimanual_task_space_constraint.hh:29-45`。实验侧的具体数值见 `experiments/cpp/crrtc_spheres.cc:64-72`、`experiments/cpp/benchmark_maze.cc:59-61`、`experiments/cpp/bimanual_panda_planning.cc:66-73`、`experiments/cpp/digit_transport.cc:50-56`，以及 `experiments/assets/problems/benchmark/{line,plane}_problems.json` 里的 `tsr_lower_bound` / `tsr_upper_bound`。

### 1.3 代码里的数据结构

VAMP 用"结构体数组（SoA）+ SIMD 行"的方式表示一批配置：

```cpp
// vamp/src/impl/vamp/vector.hh:30
static constexpr std::size_t FloatVectorWidth = SIMDVector<SimdFloatT>::VectorWidth;  // AVX2 → 8
// vamp/src/impl/vamp/robots/panda.hh:47
using ConfigurationBlock = FloatVector<rake, dimension>;   // dimension 行，每行 rake 个 lane
```

即 `Block` 的第 $j$ 个元素是一个 8 lane 的向量，装着 $q^{(1)}_j,\dots,q^{(8)}_j$。同一个内核（如 `tsr_error`）对 8 个配置同时求值，完全无分支。**这就是论文 "vectorizing projection" 的物理载体**：$n=8$（论文图 2 为了画得下用 $n=4$，Alg. 2 注明 "e.g., 8 for Intel AVX"）。

另一个关键约定：`Configuration` 存的是**关节角原始值（rad / m）**，采样器在单位立方体 $[0,1]^d$ 里采样，再由 `scale_configuration_block` 仿射映射到关节限位（`panda.hh:81-92`，`digit.hh` 同）。所以投影迭代后要 `descale → clamp(0,1) → scale` 才能保证不违反关节限位（§2.3）。

---

## 2. Methodology：数学原理、物理含义与代码实现

### 2.1 系统结构：约束是"局部规划器"

论文把约束扩展写成一个独立的 `ParallelConstrainedExtend` 子程序，挂在 RRT-Connect 上。代码里这个关系被做成**面向对象的组合**：约束集 + 约束局部规划器作为参数传给任意现有规划器。这条设计线非常重要，因为论文的 Alg. 1 与 Alg. 2 在代码里是两个层次：

```
experiments/cpp/<problem>.cc            问题定义：障碍、起终点、约束、参数网格
  └─ experiments::run(problem, args)    experiments/cpp/common.hh:592
      ├─ problem.make_constraints()     构造 TaskSpace/CoM/ClosedLoop/Bimanual 约束
      ├─ seed_set.project(start/goal)   把起点终点先投影到流形（严格种子策略）
      ├─ RRTC::solve(..., lp)           vamp/src/impl/vamp/planning/planners/rrtc.hh:33
      │   └─ lp.steer / lp.connect_within
      │       └─ ConstrainedLocalPlanner
      │           ├─ ConstraintSet::project_any   ← Alg.2 第一阶段（粒子）
      │           ├─ fkcc_block                   ← 并行碰撞检测
      │           └─ trace()                      ← Alg.2 第二阶段（弦 + 递归加密）
      │               ├─ ConstraintSet::project_all
      │               └─ fkcc_block（细分扫描）
      └─ simplify(..., lp)              在流形上做 shortcut
```

对应关系：

| 论文                                                 | 代码                                                                                                              |
| ---------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| Alg. 1 `Vectorized Manifold-Constrained RRT-Connect` | `vamp/src/impl/vamp/planning/planners/rrtc.hh`（`RRTC::solve`）                                                   |
| Alg. 2 `ParallelConstrainedExtend`                   | `vamp/src/impl/vamp/planning/constraints/local_planner.hh`（`steer` + `trace`）                                   |
| Alg. 2 `ParallelProject`                             | `vamp/src/impl/vamp/planning/constraints/manifold/constraint_set.hh`（`project_all` / `project_any` / `descend`） |
| Alg. 2 `getDescentStep`                              | 生成内核 `solve_*_lm_inner/outer/gradient_descent` + `integrate_step`                                             |
| 约束函数 $F_i$ 与 Jacobian $J_i$                     | `cricket/src/tracing/constraints.cc` 追踪 → `vamp/src/impl/vamp/robots/*.hh` 中的 `tsr_error` 等                  |

### 2.2 约束函数：TSR 位姿误差与铰链（hinge）

#### 数学

对每个末端执行器 $e$，原始位姿误差由 §1.2 的 $d(q)\in\mathbb R^6$ 给出。把它"夹"进区间 $[l,u]$：

$$
e_j=\min(d_j-l_j,0)+\max(d_j-u_j,0)
=\begin{cases}
d_j-u_j, & d_j>u_j \quad(\text{超出上界})\\
0, & l_j\le d_j\le u_j \quad(\text{盒内，自由})\\
d_j-l_j, & d_j<l_j \quad(\text{超出下界})
\end{cases}
$$

总违反量与非负代价：

$$
F(q)=\sum_{i\in\text{约束}}\ \sum_{j} e_{i,j}(q)^2,\qquad
\text{converged}\iff F(q)\le \epsilon \quad(\text{代码默认 }\epsilon=10^{-6}).
$$

注意 $\epsilon$ 是**平方**误差阈值，所以默认对应的实际误差是 $\|e\|\le 10^{-3}$（1 mm / 0.001 rad）；README 也说明 `--tolerance=1e-4` 约等于"1 cm 与 0.01 rad 的松弛"。

#### 物理含义

- 区间是**不等式约束**：盒内不罚、盒外才推。这就是"手可以在平面里自由滑动，但一旦离开平面就被拉回来"。
- $F$ 在边界处 $C^1$（$e$ 本身在边界不可微，但 $e^2$ 的导数连续且为 0），所以高斯–牛顿类方法可用。
- 单位是物理单位（m / rad），因此容差可以在不同机器人、不同任务之间直接比较。

#### 代码

```cpp
// vamp/src/impl/vamp/planning/constraints/manifold/constraint.hh:151-183
auto squared_error(const Block &q) const noexcept -> Row final
{
    derived().input.q = q;
    derived().run_kernel();                      // 一次内核：d(q) 与 J(q)，8 个 lane 同时算

    for (auto i = 0U; i < err_size; ++i)
    {
        solve.err[i] = (solve.err[i] - derived().input.lb[i]).min(0.F) +
                       (solve.err[i] - derived().input.ub[i]).max(0.F);
        // hold_satisfied_rows=false 时，把已满足行的 Jacobian 清零，允许沿盒内方向滑动
    }
    auto d = solve.err[0] * solve.err[0];
    for (auto i = 1U; i < err_size; ++i) d = d + solve.err[i] * solve.err[i];
    return d;                                    // 逐 lane 的平方违反量
}
```

几个值得注意的实现细节（都直接影响数值行为）：

1. **误差与 Jacobian 被缓存**。`squared_error(q)` 把 $d$ 和 $J$ 存在 `solve` 缓冲里，`step(q,...)` 直接消费缓存，省掉一次完整求值（每个投影迭代只算一次内核）。代价是"必须先 `squared_error` 再 `step`"的顺序契约（`constraint.hh:16-22` 明确写了）。
2. **`hold_satisfied_rows`**（默认 `true`）：已满足行的 Jacobian 行**保留**，LM 求解会把"误差 0"当作"保持该位姿值"的等式，于是投影只修正被违反的分量，其余分量原地不动；设为 `false` 则把满足行清零，投影可以沿盒内方向"滑动"，找到更近的流形点但会缩短扩展步。README 记录了两者在不同问题上的速度差异（迷宫 / G1 上保留更快，Digit 上混合，脚相对 CoM 时清零更快）。这不是论文正文的内容，但是理解实测曲线的关键旋钮。
3. **NaN 的妙用**：当旋转误差恰为 0（在流形上）时，$\arccos(1)$ 使 `log` 映射产出 NaN。x86 的 `minps/maxps` 在遇到 NaN 时返回第二个操作数，于是 `.min(0.F)` 与 `.max(0.F)` 都把 NaN 夹成 0——**"已在流形上"被自动编码为"误差 0"**。这也是 README 警告不要用 `-ffinite-math-only` 编译的原因。
4. **`equality_row_width = 0.5`**（`constraint.hh:14`）只用于 `active_rows()`，即只给**切空间/chart** 构造分类"这一行是不是等式约束"，**不参与投影本身**。投影对 slab 行同样使用 hinge。

### 2.3 投影 = 阻尼高斯–牛顿（Levenberg–Marquardt）

#### 数学

给定不在流形上的 $q$（$e=e(q)\neq 0$），投影要解

$$
q^\star=\arg\min_{q'}\ \|q'-q\|^2\quad \text{s.t.}\quad e(q')=0 .
$$

一步高斯–牛顿：线性化 $e(q+\Delta q)\approx e(q)+J(q)\Delta q$，令其为零得**欠定/超定**线性系统

$$
J\,\Delta q=-e .
$$

代码给出三种解法（`vamp/src/impl/vamp/planning/constraints/settings.hh:8-13`）：

$$
\boxed{
\begin{aligned}
\text{InnerLM: } & \Delta q=-J^{\top}\!\left(JJ^{\top}+\lambda I_{k}\right)^{-1}e
&&\text{（任务空间 }k\times k\text{）}\\[2pt]
\text{OuterLM: } & \Delta q=-\left(J^{\top}J+\lambda I_{d}\right)^{-1}J^{\top}e
&&\text{（构型空间 }d\times d\text{）}\\[2pt]
\text{GradDesc: } & \Delta q=-J^{\top}e
&&\text{（纯梯度）}
\end{aligned}}
$$

然后

$$
q\leftarrow q+\alpha\,\Delta q,\qquad \alpha=\texttt{descend\_rate}\in\{0.5,0.75,1\},
$$

并做关节限位夹取与四元数重归一化。阻尼系数在代码中是 $\lambda=10^{-6}$（`cricket/src/tracing/constraints.cc:211`：`constexpr double damp = 1e-6;`）。

#### 物理含义

- **$J$ 是什么**：微分运动学。$J\in\mathbb R^{k\times d}$ 把关节速度映射成任务空间速度；$\ker J$ 是"动了也不影响约束"的方向（自运动 / 零空间），$\operatorname{range}(J^\top)$ 是"能修正约束"的方向。
- **InnerLM（最小范数解）**：在 $JJ^\top$ 可逆时，它给出满足 $J\Delta q=-e$ 的**最小关节位移**解，即经典阻尼伪逆（damped least squares / Levenberg 形式）。物理上就是"用最小的关节运动把末端推回约束面"——这正是操作空间控制（operational space control）里的做法。
- **OuterLM（最小二乘解）**：等价于最小化 $\|J\Delta q+e\|^2+\lambda\|\Delta q\|^2$。当 $J$ 行满秩且 $\lambda\to0$ 时与 InnerLM 等价；当约束**不可达**（$J$ 降秩、末端顶到奇异位形）时，InnerLM 的 $JJ^\top$ 奇异、解爆炸，而 OuterLM 退化为"最接近的折中"，更稳。
- **$\lambda$ 是什么**：数值阻尼，物理上类似"黏性/柔顺"——限制单步关节位移，避免在奇异点附近一步跳到很远。$10^{-6}$ 很小，说明他们更信任精确牛顿步（配合 $\alpha$ 与早退来兜底）。
- **为什么两种都要**：InnerLM 的矩阵是 $k\times k$（TSR 是 6 或 $6n_{eef}$，CoM 是 2，闭环是 2），**远小于** $d\times d$（7 / 14 / 28 / 31）；OuterLM 在 $k\lesssim d$ 时反而更贵，但对降秩鲁棒。所以论文把 "inner/outer" 作为超参扫描维度，并在实验里发现不同问题偏好不同（迷宫默认 OuterLM，球笼默认 InnerLM）。
- **GradDesc 的意义**：只用于对照，证明"不做二阶求解也能收敛，只是慢"，从而说明 LM 的加速来自二阶信息而非别的因素。

#### 收敛判据（代码）

`ConstraintSet::descend`（`constraint_set.hh:273-333`）同时检查三件事：

$$
F(q_t)\le\epsilon\ \ (\text{收敛}),\qquad
\|\Delta q_t\|^2\le 10^{-12}\ \ (\text{停滞，整批放弃}),\qquad
\|q_t-q_0\|^2\ge 4\,\Delta_{\max}^2\ \text{或}\ \|\Delta q_t\|^2\ge4\Delta_{\max}^2\ \ (\text{发散，放弃})
$$

其中 $\Delta_{\max}$ 由调用者给出（扩展步长或弦长），"漂移超过 $2\Delta_{\max}$ 就判失败"对应论文 Alg. 2 第 9 行的 `‖q_proj − q_s‖ > 2·dist`。

#### 代码

追踪阶段（生成内核）把"一步 LM"直接编译进机器人头文件：

```cpp
// cricket/src/tracing/constraints.cc:205-266（节选）
constexpr double damp = 1e-6;
// 输入布局：J (err_size x nq, 行主序)，然后 err (err_size)
ADMatrixXs J(err_size, nq); ... ADVectorXs err(err_size); ...

switch (method) {
case ProjMethod::InnerLM: {
    ADMatrixXs identity = ADMatrixXs::Identity(err_size, err_size);
    const auto factor =
        cholesky_factor<ADMatrixXs, ADCG>(J * J.transpose() + identity * damp);
    grad = J.transpose() * cholesky_solve<ADMatrixXs, ADVectorXs, ADCG>(factor, err);
    break; }
case ProjMethod::OuterLM: {
    ADMatrixXs identity = ADMatrixXs::Identity(nq, nq);
    const auto factor =
        cholesky_factor<ADMatrixXs, ADCG>(J.transpose() * J + identity * damp);
    grad = cholesky_solve<ADMatrixXs, ADVectorXs, ADCG>(factor, J.transpose() * err);
    break; }
case ProjMethod::GradDesc:
    grad = J.transpose() * err;
}
```

生成出来的代码就是一段**完全展开、无循环、无分支**的标量表达式，每个临时量是 `FloatVector<rake,1>`（即 8 lane 的向量）。例如 Panda 的 7 维构型、6 行 TSR：

```cpp
// vamp/src/impl/vamp/robots/panda.hh:41993（生成代码）
static inline auto solve_tsr_error_lm_inner(const InputVector &x, ConfigurationBlock<rake> &y) noexcept
{
    std::array<FloatVector<rake, 1>, 26> v;
    v[0] = sqrt(1e-06 + x[0]*x[0] + ... + x[6]*x[6]);              // Cholesky 的第 1 个对角元
    v[1] = (x[35]*x[0] + ... + x[41]*x[6]) / v[0];                  // 前代
    ...
    y[0] = x[35]*v[25] + x[28]*v[24] + ... + x[0]*v[18];            // J^T z
}
```

手写的 Cholesky（Eigen 的 `.llt()` 无法被 CppAD 追踪）在 `cricket/src/tracing/cholesky.hh:10-88`，由于维度在追踪期是常量，循环被完全展开——正好符合论文所说 "loop-unrolled Cholesky solve expressions"。

调用侧（运行时）：

```cpp
// vamp/src/impl/vamp/planning/constraints/manifold/constraint.hh:185-190
void step(Block &q, ProjMethod method, float alpha) const noexcept final
{
    Block gradient;
    derived().solve_step(gradient, method);   // 生成内核：J^T(JJ^T+λI)^{-1}e 或 (J^TJ+λI)^{-1}J^Te
    integrate_step<Robot, rake>(q, gradient, alpha);   // q -= α·gradient，夹限位，四元数归一化
}
// vamp/src/impl/vamp/planning/constraints/utils.hh:30-49
q[i] = q[i] - gradient[i] * alpha;
Robot::descale_configuration_block(q);
q[i] = q[i].clamp(0.F, 1.F);        // 关节限位
Robot::scale_configuration_block(q);
renormalize_so3<Robot, rake>(q);    // 四元数增量更新后投影回 S^3（nlerp）
```

> 关于四元数：加法更新会离开单位球面 $S^3$，除以范数是合法的 retraction，所以"线性更新 + 归一化"= nlerp（`utils.hh:6-26`）。这是把 $SE(3)$ 的浮动基塞进欧氏投影框架的必要补丁。

### 2.4 多约束复合：循环投影

#### 数学

同时满足 $m$ 个约束需要投影到交集 $\mathcal M=\bigcap_i\mathcal M_i$。论文放弃"合并成一个大方程组"（规模与条件数都差），改用**循环投影**（cyclic projection / POCS，Combettes 的 set-theoretic estimation）：

$$
q^{(t+1)}=\big(P_{m}\circ P_{m-1}\circ\cdots\circ P_{1}\big)\big(q^{(t)}\big),\qquad
P_i=\text{对约束 }i\text{ 的一步阻尼 GN},
$$

迭代到 $\sum_i F_i(q)\le\epsilon$。

**收敛性**：若各 $\mathcal M_i$ 是闭凸集且 $P_i$ 是精确投影，von Neumann 交替投影定理保证收敛到交集中的一点；这里 $\mathcal M_i$ 是非凸流形、$P_i$ 只是**一步** GN，因此只有局部收敛保证——这也正是论文需要用"粒子 + 早退"兜底的原因。

#### 物理含义

每个约束像一只"夹子"：先把脚按回地面，再把质心推回支撑多边形，再把腿的四连杆闭合，再让双手保持相对位姿。一次循环 = 每个夹子各夹一下；重复若干次，配置被挤到所有夹子的公共交集上。**顺序敏感**（先满足谁）但在实践中"够用"，而且**顺序化让并行零成本**——因为每个夹子内部都是 SIMD 批处理。

#### 代码

```cpp
// vamp/src/impl/vamp/planning/constraints/manifold/constraint_set.hh:151-162
void step_in_place(Block &q) const noexcept
{
    for (const auto &c : constraints_)          // 预定顺序：feet → CoM → closed loop → bimanual
    {
        if (has_pins_) c->mask_jacobian(pinned_.data());
        c->step(q, settings_.method, settings_.descend_rate);   // 消费上一次 squared_error 的缓存
    }
}
```

`descend` 的一次外层迭代是：

```cpp
auto dist = squared_error(q);              // ① 在当前 q 上一次性求所有约束的 e_i, J_i 并缓存
for (i < max_iterations and not converged(dist); ++i) {
    prev = q;
    step_in_place(q);                      // ② 依次施加每个约束的一步（各自用①的线性化点）
    dist = squared_error(q);               // ③ 在新 q 上重新求值，检查收敛/停滞/漂移
    ...
}
```

> **论文与代码的一处细微差别**：Alg. 2 第 30–34 行写的是 `for F_i: d ← F_i(q); J_i ← J_i(q); q ← q − Δq`，即每一步都用**更新后的** $q$ 重新线性化（Gauss–Seidel）；而 `descend` 在**同一个** $q$ 上把整栈 $e,J$ 都算好再依次施加（Jacobi 式线性化点 + Gauss–Seidel 式累积）。两者都是"循环投影"，但在强耦合约束（如 Digit 的脚 + CoM + 闭环）上行为会有差别：Jacobi 版本每轮只做一次全栈求值，省掉 $m-1$ 次内核调用，代价是耦合强时需要更多轮。这是阅读代码时值得记住的实现选择。

另外两个与约束复合相关的机制：

- **Pinning**（`constraint_set.hh:49-61` + `constraint.hh:116-133`）：把某些维度的 Jacobian 列清零，投影就只能在"活动子空间"里解，被钉住的关节严格不动。Python API 通过 `PinnedRNG` 暴露（`vamp/src/impl/vamp/bindings/python/robot_helper.hh:340-346`），用于"只规划一部分关节"的多查询场景。
- **Pfaffian 行**（`pfaffian_constraint.hh`）：只约束速度方向、不产生位置误差的行（如丝杠、刀口、无滑滚动）。它们对投影循环透明（`squared_error` 恒 0），但会进入切空间（`active_rows` / `pfaffian_rows` 都为真），从而把"允许的运动方向"进一步收紧。论文没有使用这一类，属于代码库的扩展能力。

### 2.5 向量化：SIMD、SoA 与 rake

#### 数学

把一批配置写成矩阵

$$
Q=\begin{bmatrix}q^{(1)} & q^{(2)} & \cdots & q^{(n)}\end{bmatrix}\in\mathbb R^{d\times n},
$$

批量投影算子

$$
\mathcal P(Q)=\begin{bmatrix}P(q^{(1)}) & \cdots & P(q^{(n)})\end{bmatrix},
$$

由于 $P$ 逐点定义、lane 之间**没有数据依赖**，$\mathcal P$ 可以完全并行。代码用两个不同的"批量语义"：

- **`project_all`**：要求**所有** lane 都收敛（用于弦上的插值点，因为整条路径都得在流形上）；
- **`project_any`**：只要有**一个** lane 收敛就返回它的编号（用于"在 $q_{steer}$ 附近找一个能投影成功的落点"）。

```cpp
// constraint_set.hh:167-197
auto project_all(Block &q, float max_distance = no_max_distance) const noexcept -> bool
{
    if (empty()) return true;
    return descend<true>(q, max_distance).test_all_less_equal(Row::fill(settings_.tolerance));
}
auto project_any(Block &q, float max_distance = no_max_distance) const noexcept -> int
{
    if (empty()) return 0;
    const auto dist = descend<false>(q, max_distance);
    for (auto lane = 0U; lane < rake; ++lane)
        if (dist[{0, lane}] <= settings_.tolerance) return static_cast<int>(lane);
    return -1;
}
```

注意 `test_all_less_equal` / `test_any_less_equal` 是**一条 SIMD 比较 + 掩码**，没有 per-lane 分支——这就是论文反复强调的 branchless control flow：8 个 lane 可以处于完全不同的收敛阶段，而指令流只有一条。

#### 物理含义

- 向量化的第一层含义是**吞吐**：一个内核调用处理 8 个配置，理想情况 $8\times$。
- 第二层、也是更重要的含义：8 个 lane 是 **8 个并行假设**。在约束梯度地形复杂（多解、局部极小、奇异性）时，"从稍微不同的位置出发的 8 次投影"比"从同一个位置出发的 1 次长迭代"更容易成功——这既是速度也是鲁棒性。
- 第三层：**早退**。因为 8 个点是一起校验的，只要有一个点碰撞或投影失败，整条边立即作废，不用等到把 8 个点都算完。这把"验证"从"逐点串行 + 后验"变成"批内即时否决"，是论文所说的 `quickly invalidate infeasible configurations`。

### 2.6 两阶段约束扩展（Alg. 2 逐行对照）

这是论文的**核心贡献**，也是 Fig. 2 的四格图。直觉是：**先找到流形上的"落脚点"，再保证从当前点到落脚点之间有一条连续的流形路径。**

| 图                                  | 做什么                                                           | 代码                                   |
| ----------------------------------- | ---------------------------------------------------------------- | -------------------------------------- |
| ![a](../docs/methodology/meth1.svg) | (a) 从 $q_{near}$ 朝随机样本 $q_{rand}$ 走一步，得到 $q_{steer}$ | `local_planner.hh:129-143`             |
| ![b](../docs/methodology/meth2.svg) | (b) 在 $q_{steer}$ 附近撒 $n$ 个粒子，并行投影，取最先收敛者     | `project_any`，`constraint_set.hh:180` |
| ![c](../docs/methodology/meth3.svg) | (c) 在 $q_{near}\to q_{proj}$ 的弦上插 $n$ 个点，并行投影        | `trace`，`local_planner.hh:231-283`    |
| ![d](../docs/methodology/meth4.svg) | (d) 相邻投影点之间递归插值/投影/校验，直到分辨率 $\delta$        | `trace` 的细分扫描，`:297-337`         |

#### 第一阶段：粒子投影（找落脚点）

```cpp
// local_planner.hh:129-150（节选）
const bool reach = distance < range;
const float step_length = (reach) ? distance : range;
const auto vector = (target - from) * ((reach) ? 1.F : range / distance);

// 每个 lane 的步长乘一个抖动系数；lane 0 恰好指向候选点
const auto perturb = FloatVector<rake>::fill(1.F) +
                     FloatVector<rake>(StdDevSamples<rake>::samples) *
                         constraints.settings().perturbation_scale;

Block block;
for (auto i = 0U; i < Robot::dimension; ++i)
    block[i] = from.broadcast(i) + vector.broadcast(i) * perturb;

renormalize_so3<Robot, rake>(block);
const auto winner = constraints.project_any(block, step_length);
if (winner < 0) return {SteerStatus::Trapped, chain_};
```

其中抖动系数是确定性的"星形"序列（`local_planner.hh:21-32`）：

$$
m=(0,\,-0.25,\,+0.25,\,-0.5,\,+0.5,\,-0.75,\,+0.75,\,-1.0),
\qquad
q^{(i)}=q_s+\Delta\,\big(1+m_i\,\sigma\big)\,v,
$$

$\sigma$ = `perturbation_scale`（实验网格 $\{0.01,0.1,0.2\}$），$\Delta=\min(\|q_t-q_s\|,r)$，$v$ 为单位方向。

**为什么是乘性抖动而不是论文写的加性 $\epsilon_i v$？** 两者等价：$q_s+\Delta(1+m_i\sigma)v=q_{steer}+(\Delta m_i\sigma)v$，即沿方向向量加一个长度扰动。乘性写法的好处是**自动随步长缩放**——长步时扰动大、短步时扰动小，始终是"步长的 10%"，不需要额外归一化。

**为什么 lane 0 不动？** 保证"如果精确的 $q_{steer}$ 本来就能投影成功，一定优先选它"，抖动只作为后备。

**接受条件**（对应 Alg. 2 第 9 行）：$\|\hat q-q_s\|^2\le 4\,\Delta^2$（不超过 2 倍步长的漂移），否则 `Trapped`。随后先做**端点碰撞预检**（`fkcc_block`），再进入第二阶段——顺序很重要：碰撞预检比完整 trace 便宜得多（`local_planner.hh:174-190`）。

#### 第二阶段：弦离散 + 并行投影 + 连续性检查

```cpp
// local_planner.hh:237-283（节选）
const auto vector = b - a;
const float distance = vector.l2_norm();
const auto percents = FloatVector<rake>(Percents<rake>::percents);   // {1/n, 2/n, ..., 1}

Block block;
for (i) block[i] = a.broadcast(i) + vector.broadcast(i) * percents;  // 弦上的 n 个点
renormalize_so3<Robot, rake>(block);
if (not constraints.project_all(block, distance)) return false;      // 8 个点一起投影

// 投影不能把路径撕开：相邻 lane 的间距不得超过 2 倍的无约束间距
const float lane_spacing = distance / static_cast<float>(rake);
const float max_gap2_allowed = 4.F * lane_spacing * lane_spacing;
for (lane) {
    gap2 = ||block[lane] - block[lane-1]||^2;      // lane 0 的前驱是 a
    if (gap2 > max_gap2_allowed) return false;
}

if (not fkcc_block<Robot, rake>(e, block)) return false;             // 8 个点一起碰撞检测
```

**为什么需要连续性检查？** 投影是逐点独立的：$q^{(3)}$ 可能被投影到流形的"另一支"（比如肘部翻到另一侧），于是 $q^{(2)}\to q^{(3)}$ 之间在流形上并没有连续路径。检查相邻间距（$\le 2\times$ 无约束间距）就是检测"撕裂"。`task_space_projection` 这个调试例子专门演示了这一点（§3.10 有实测输出）。

#### 第二阶段（续）：递归加密到分辨率 $\delta$

```cpp
// local_planner.hh:297-337（节选）
const float max_gap = std::sqrt(max_gap2);
const auto n_steps = static_cast<std::size_t>(std::max(std::ceil(max_gap * resolution), 1.F));
if (n_steps == 1) return true;

const auto shift  = Block(diffs) / static_cast<float>(n_steps);      // 每个 lane 回退的步长
const float substep = max_gap / static_cast<float>(n_steps);
const auto max_drift2 = FloatVector<rake, 1>::fill(4.F * substep * substep);

auto linear = block;
for (auto i = 1U; i < n_steps; ++i)
{
    for (j) linear[j] = linear[j] - shift[j];                        // 沿每个 gap 线性回退
    auto projected = linear;
    renormalize_so3<Robot, rake>(projected);
    if (not constraints.project_all(projected, max_gap * rake)) return false;
    if (squared_distance(projected, linear).test_any_greater_equal(max_drift2)) return false;
    if (not fkcc_block<Robot, rake>(e, projected)) return false;
}
```

数学上：第 $i$ 个 gap 的第 $j$ 个子步是

$$
\xi^{(j)}=\operatorname{proj}\!\left(q^{(i)}-\frac{j}{n_{steps}}\big(q^{(i)}-q^{(i-1)}\big)\right),
\qquad j=1,\dots,n_{steps}-1,
$$

并检查 $\xi^{(j)}$ 距其线性插值点不超过 $2\,\delta$ 且无碰撞。其中

$$
n_{steps}=\left\lceil \frac{g_{\max}}{\delta}\right\rceil,\qquad \delta=\frac{1}{\texttt{Robot::resolution}}=\frac{1}{32}.
$$

**物理含义**：这是**离散测地线的自适应加密**。弦（直线）在环境空间里离流形很远，直接投影得到的是"投影后的折线"，相邻投影点之间仍可能有很长的弧；细分成 $\delta\approx0.03$ 的子步后，每一步的"弦–弧误差"都被控制在 $\delta$ 量级，等价于对流形上的连续路径做了一次**近似连续碰撞检测**。$n_{steps}$ 与 gap 成正比，所以平坦处的边不会被过度细分（`n_steps == 1` 直接返回）。

> 注意：这些子步**只做校验，不加进树**。树的节点仍是那 $n$ 个投影点（`emit_all_waypoints=true` 时全加，否则只加最后一个）。这与论文 Alg. 2 第 15 行"递归插值直到分辨率"的表述略有差别：论文更像"递归加节点"，代码是"递归校验 + 端点入树"。对正确性没有影响（校验更严），对路径密度有影响。

#### 收尾

```cpp
// local_planner.hh:192-204
next = waypoints.back();                                  // 第 n 个 lane ≈ b
if (settings.emit_all_waypoints) chain_.assign(all);      // 整条投影链入树
else                             chain_.emplace_back(next);
const bool reached = reach and (next - target).squared_l2_norm() < settings.reached_radius2;
```

`reached_radius2 = 1e-2` 对应 $0.1$ 的构型距离——因为投影几乎不可能精确落在目标点上，必须给一个"到达半径"。

### 2.7 追踪编译、可微 log 映射与手写 Cholesky

#### 追踪编译（tracing compiler）

论文说 "in a manner similar to VAMP, we generate SIMD parallel code for evaluating constraint functions using a tracing compiler"。代码里这条流水线是：

1. 用 Pinocchio 的 `ADModel = model.cast<ADCG>()` 建立 CppAD 的自动微分模型；
2. `Independent(ad_inp)` 把"构型 + 约束参数"声明为自变量（参数也进 tape，所以一个内核可以服务所有实例）；
3. 在 AD 标量上做正运动学、帧更新、误差计算；
4. `ADFun<CGD> error_func(ad_inp, data)` 固化函数；`error_func.Jacobian(...)` 得到解析 Jacobian；
5. `generate_code(handler, ...)` 由 CppADCodeGen 输出**直线型 C++**，临时量自动分配、循环全展开、无分支。

最终产物就是 `panda.hh:41164` 那种几千行的 `tsr_error`：每次调用只做算术，没有堆分配、没有循环、没有分支，天然适合 SIMD 与乱序执行。这是"单核 CPU 上跑出 1000×"的底层原因之一——**baseline 的 Pinocchio 每次求值都要走通用数据结构**。

#### 可微的 $\log(R)^\vee$

论文明确说："we implement a differentiable version of $\log(R_{sw'})^\vee$ by using a first order Taylor Series approximation of the sinc function at singularities." 代码（`cricket/src/tracing/se3_ops.hh`）：

$$
\operatorname{sinc}(x)=\frac{\sin x}{x},\qquad
\operatorname{sinc}_{\text{smooth}}(x)=(1-w_x)\left(1-\frac{x^2}{6}\left(1-\frac{x^2}{20}\right)\right)+w_x\,\frac{\sin x}{x},
\quad w_x=\frac{x^2}{x^2+10^{-7}},
$$

$$
\log(R)^\vee=(1-w)\cdot\frac{1}{2}\left(R-R^\top\right)^\vee
+w\cdot\frac{1}{2\,\operatorname{sinc}_{\text{smooth}}(\theta)}\left(R-R^\top\right)^\vee,
\quad w=\frac{\sin^2\theta}{\sin^2\theta+10^{-7}}.
$$

**为什么需要**：$\theta=\arccos\frac{\operatorname{tr}R-1}{2}$ 在 $\theta\to0$ 处导数 $\to\infty$。若直接对精确公式求 AD，Jacobian 在流形附近会爆炸甚至产生 NaN，投影的牛顿步就毁了。Taylor 分支在 $\theta\lesssim3\times10^{-4}$ 内与精确值相差 $O(\theta^5)$，且导数有界。权重 $w$ 用 $\sin^2\theta$ 而非 $\theta$，是为了避免再引入一次 `acos`。

**一个诚实的细节**：即使做了平滑，在**恰好** $\theta=0$ 处 `w * (sin x / x)` 仍是 `0 * NaN = NaN`。代码并不试图消除它，而是**依赖 hinge 把 NaN 夹成 0**（§2.2 第 3 点），并且 `ChartBuilder` 有专门的"Jacobian 非有限就换个点重算"的回退（`chart.hh:220-245`）。所以准确的说法是：平滑让**流形附近**的梯度可用，而**流形上**的奇点被 hinge 语义无害化。实现与编译器 flag 强耦合（不能用 `-ffinite-math-only`）。

#### 手写 Cholesky

Eigen 的 `.llt()` 不能被 CppAD 追踪，所以 `cricket/src/tracing/cholesky.hh` 手写了 `cholesky_factor` / `lower_solve` / `upper_solve` / `cholesky_solve`。因为 $k$（或 $d$）在追踪期是编译期常量，三重循环被完全展开成直线代码，正好是论文说的 "loop-unrolled Cholesky solve expressions that can be parallelized"。

> 顺带一提：`solve_tsr_error_lm_inner` 生成代码里的 `sqrt(1e-06 + x[0]*x[0] + ...)` 就是 Cholesky 的第一个对角元 $\sqrt{(JJ^\top)_{00}+\lambda}$，后面的 `/v[0]` 是前代，`y[...]` 是回代——一眼就能对上数学。

### 2.8 四个具体约束的数学（Digit 全身例子）

论文 Fig. 7 给出 Digit 的约束栈，代码在 `experiments/cpp/digit_transport.cc:96-125` 按同样顺序组装：

| 约束               | 数学                                                                                                      | 行数 | 代码                                |
| ------------------ | --------------------------------------------------------------------------------------------------------- | ---- | ----------------------------------- |
| 双脚固定（TSR）    | $d_f=\begin{bmatrix}t\\ \log R^\vee\end{bmatrix}$，$f\in\{l,r\}$，位置 $\pm1$ mm、姿态 $\pm0.1$ rad       | 12   | `task_space_constraint.hh`          |
| 质心（支撑多边形） | $d_{CoM}=x^{xy}_{com}-x_{near}$，$x_{near}=\operatorname{proj}(x^{xy}_{com},\mathcal P)$                  | 2    | `com_constraint.hh`                 |
| 闭环连杆           | $d_{CL}=\|X^{l}_{hip}-X^{l}_{tar}\|-L_l$，左右各一                                                        | 2    | `closed_loop_constraint.hh`         |
| 双手保持刚体       | $d_B=\begin{bmatrix}t_{err}\\ \log(R_{err})^\vee\end{bmatrix}$，$T_{err}=(T^w_l)^{-1}T^w_r\,(T^l_r)^{-1}$ | 6    | `bimanual_task_space_constraint.hh` |

#### 质心约束：数学与实现的一处差异

论文写"error to the nearest point in the hull"，即 $d_{CoM}=x_{com}-x_{near}$。代码实现的是**半平面铰链求和**（`com_constraint.hh:124-160`）：

$$
e=\sum_{i\in\text{violated}} r_i\,n_i,\qquad r_i=n_i^{\top}\!\left(x_{com}-a_i\right)>0,
\qquad
\frac{\partial e}{\partial q}=\left(\sum_{i\in\text{violated}} n_i n_i^{\top}\right)J_{com}(q),
$$

其中 $n_i$ 是第 $i$ 条边的**单位外法向**，$a_i$ 是边起点，$J_{com}\in\mathbb R^{3\times d}$ 由生成内核 `com_jacobian` 给出。由于法向是常量，这个 Jacobian 是**精确**的（不是近似）——这一点在代码注释里明确写了。

**物理解释**：只违反一条边时，$e=r_i n_i$ 恰好等于"到该边的最短距离向量"，与论文一致；同时违反两条边（在顶点外）时，$e$ 是两条法向之和，方向指向"外角平分线"，比最近点略"用力"，但仍然是朝多边形内部推的正确方向。这是**工程上更简单、且解析可微**的替代，代价是顶点附近有轻微过冲。数值上 2 行误差、$2\times d$ 的 Jacobian，比"最近点"版本更省。

**约束的"软硬"**：`active_rows()` 把 CoM 两行标记为**非**流形定义（`com_constraint.hh:89-94`：多边形内部有正测度，是 slab），所以它只影响投影、不参与切空间构造——这正是"平衡是一个不等式约束"的数学表达。

#### 闭环连杆：为什么只用 1 行

`closed_loop_error` 的输出是

$$
d_i=\left\|p^{end}_i-p^{start}_i\right\|-L_i ,
$$

即**两个切口帧之间距离与名义杆长之差**，每闭环一行（`cricket/src/tracing/constraints.cc:391-397`）。物理上，Digit 的踝部四连杆被建模成"两端由刚性杆连接"——杆可以绕两端自由转动，所以只有"长度固定"这一个约束，不需要完整的 6 维相对位姿闭合。这是**用物理洞察换维度**的典型例子：$6\to1$，投影的难度大幅下降。

#### 双手刚体约束的左右乘顺序

论文式 (2) 写 $T_{err}=T_{ref}^{-1}(T_{lef}^{-1}T_{right})$；代码（`constraints.cc:186-187`）写

```cpp
const auto lTr_rob = lTw.inverse() * rTw;   // 实际相对位姿
const auto errT    = lTr_rob * lTr.inverse();  // 实际 × 期望^{-1}
```

两者零集相同（都要求 $lT_r^{rob}=lT_r^{des}$），但误差的平移分量相差一个旋转共轭，因此**收敛路径与代价略有不同**。这不影响正确性，但如果要逐位复现论文曲线，需要知道这个差异。

#### 维度账本

Digit 的 `dimension = 31`（7 维浮动基 + 24 关节），`sample_dimension = 30`（四元数按 3 自由度计）。约束栈的等式行数：12（双脚）+ 2（闭环）+ 4（双手位置 3 + roll 1）$=18$，双手的 pitch/yaw 两个 slab 与 CoM 的 2 个 slab 只在"越界"时才产生误差，所以**规划实际上是在 30 维里的约 12 维流形上搜索**。这与论文"28-DoF（22 关节 + 6 维浮动基）"的建模差 2 个关节：论文把 4 个高刚度弹簧关节视为刚性、把踝部 4 个闭环交给底层控制器；代码保留了 24 个关节、只把 2 个被动关节用 2 个闭环约束表示（README 第 5 节）。数字不同，物理模型一致。

### 2.9 与采样规划器的集成（Alg. 1）

#### 数学 / 算法

Alg. 1 就是 RRT-Connect，只是把"直线插值 + 碰撞检测"换成"约束扩展"：

1. 在环境空间采样 $q_{rand}$（**不**在流形上）；
2. 找最近邻 $q_{near}$，调用 `ParallelConstrainedExtend` 试图连到 $q_{rand}$；
3. 成功则从新节点反复朝另一棵树扩展（`connect_slack` 允许比直线距离更多的步数，因为投影会"绕路"）；
4. 两树相遇则回溯路径。

#### 代码

`rrtc.hh` 的关键点：

- **最近邻 / 距离**用 `Robot::distance`；对 Digit 这种四元数基座，它是 $SE(3)$ 度量而不是坐标差的欧氏距离（`digit.hh:221`），所以"最近的构型"在几何上是有意义的。
- **dynamic domain**：`if (settings.dynamic_domain and nearest_radius < nearest_distance) continue;`（`rrtc.hh:158`），成功时放大半径（`rrtc.hh:181-185`）、失败时收缩（`rrtc.hh:278-291`），用于逃离局部极小。
- **connect 循环**：
  ```cpp
  // rrtc.hh:200-203
  const std::size_t n_extensions =
      std::ceil(lp.connect_slack * other_nearest_distance / settings.range);
  ```
  `connect_slack = 2`（约束局部规划器的默认值，`settings.hh:38`），因为投影漂移使"直线距离 / range"低估所需步数。
- **插入整条链**：`insert_chain(extension.waypoints, ...)` 把投影链的每个点都变成树节点（`emit_all_waypoints`），于是树本身"贴着流形长"，后续扩展的起点天然在流形上。
- **直线连接快速路径**：`solve` 开头先试 `lp.connect_within(start, goal, ...)`（`rrtc.hh:72-91`），成功就直接返回——这就是"无障碍时 0 次迭代、0.01 ms"的来源（§3.10 实测中很多问题的 `iterations = 0`）。

### 2.10 复杂度与"为什么是 100×而不是 8×"

单次投影迭代的代价（$k$ = 约束行数，$d$ = 构型维数，$n$ = rake）：

$$
\text{InnerLM}:\ O\!\left(k^3+k^2d\right),\qquad
\text{OuterLM}:\ O\!\left(d^3+d^2k\right),\qquad
\text{一次内核求值}:\ O\!\left(\text{展开后的乘加数}\right)\ \text{条 SIMD 指令}.
$$

一次约束扩展（两阶段）的代价约为

$$
O\!\Big(n\cdot I\cdot \underbrace{(k^3+k^2d)}_{\text{每 lane}}\Big)\ \text{除以 }n\text{（SIMD）}
\;+\;O\!\left(\sum_{\text{gaps}} n_{steps}\cdot I'\cdot(\cdot)\right),
$$

$I$ 为投影迭代上限（实验网格 5–100，默认 15/25）。串行实现要做 $n$ 倍的工作，SIMD 把它压成一条指令流。

但实测加速是 $100$–$2000\times$，远超 $8\times$。拆解（这是论文没有明说、但可以从代码与消融反推的）：

| 来源                                                       | 大致量级                   | 依据                                                            |
| ---------------------------------------------------------- | -------------------------- | --------------------------------------------------------------- |
| SIMD 8 lane                                                | $8\times$                  | `FloatVectorWidth = 8`                                          |
| 编译型无分支内核 vs Pinocchio 通用求值                     | 内核级 $5$–$20\times$      | OMPL(comp.) 消融的端到端 $2$–$5\times$ 是被搜索开销稀释后的下界 |
| 早退：一个 lane 失败即放弃整条边                           | 取决于问题，$2$–$10\times$ | `project_all` 的 `test_all` + `trace` 的 `return false`         |
| 两阶段 + 粒子：更少的投影迭代、更高成功率                  | $1.3$–$2\times$            | Table II 的 1-step 消融                                         |
| 无堆分配 / SoA / LTO / 单核无锁                            | $1.5$–$3\times$            | 代码结构                                                        |
| baseline 自身开销（OMPL 状态空间、约束 wrapper、动态分配） | 额外放大                   | 同硬件对比                                                      |

**关键点**：加速不是"把 8 个点塞进一条指令"这么简单，而是"**把投影从串行优化改造成数据并行 + 提前否决**"。这就是为什么论文强调 "revisits projection-based constraint satisfaction through the lens of parallelization"，而不是"写了个更快的 IK"。

### 2.11 设计取舍一览

| 设计                       | 目的                                                 | 代价                                     | 代码 / 论文                                                        |
| -------------------------- | ---------------------------------------------------- | ---------------------------------------- | ------------------------------------------------------------------ |
| 两阶段投影                 | 先落到流形附近，再保证弦上连续，减少"撕裂"与无效迭代 | 多一次批量投影                           | `local_planner.hh:117-340`，论文 §IV-B                             |
| 粒子（乘性抖动）           | 用 $n$ 个起点提高投影成功率；顺带实现"自适应步长"    | 抖动尺度要调（网格 $\{0.01,0.1,0.2\}$）  | `local_planner.hh:21-32,135-146`                                   |
| 早退（漂移 / 失败 / 碰撞） | 不在注定失败的边上浪费投影                           | 可能误杀"能绕过去"的边                   | Alg. 2 行 9/13/26                                                  |
| `emit_all_waypoints`       | 树贴着流形生长，后续扩展更短                         | 内存与节点数增加                         | `settings.hh:34`                                                   |
| `connect_slack=2`          | 补偿投影漂移                                         | 单次 connect 步数上限变大                | `settings.hh:38`                                                   |
| `reached_radius2=1e-2`     | 投影落不到精确目标点                                 | 目标精度受限于 $0.1$                     | `settings.hh:42`                                                   |
| `hold_satisfied_rows=true` | 保持已满足分量，步长不被"拉短"                       | 强耦合时可能锁死（Digit 种子投影需关掉） | `settings.hh:56`；`digit_transport_sequence.py:195-201` 有注释说明 |
| 循环投影                   | 避免大方程组的规模爆炸，且天然可并行                 | 顺序敏感、强耦合时轮数多                 | Alg. 2 行 30；`constraint_set.hh:151`                              |
| 单线程 / 单核              | 公平对比，且避免线程调度抖动                         | 不能用多核进一步加速                     | 论文 §V                                                            |

---

## 3. Experimental evaluation：如何验证 Methodology

这一节不按"结果罗列"组织，而按**"方法论主张 → 实验设计 → 观测量 → 结论"**组织，因为论文的实验设计本身是有层次的：先证明"快"，再证明"快不是靠牺牲正确性/质量"，再通过消融把加速归因到具体组件，最后用真机证明"快是有用的"。

### 3.1 验证逻辑总表

| 方法论主张                                              | 对应实验                                                                            | 关键观测量                                | 论文结果                                                                                                       | 代码                                                                            |
| ------------------------------------------------------- | ----------------------------------------------------------------------------------- | ----------------------------------------- | -------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| **C1** SIMD 批量投影把约束扩展从"秒级"降到"毫秒/微秒级" | 单臂线/平面 4 类约束 × 障碍密度分箱（Fig. 5）；迷宫（Table I）                      | 规划时间的 CDF / 分位数、成功率           | 多数 $<1$ ms；迷宫中位 $0.006$ s vs OMPL $17.3$ s；$100$–$2000\times$                                          | `benchmark_line_plane.cc`、`benchmark_maze.cc`                                  |
| **C2** 加速来自"批量 + 早退"，不只是"更快的单点投影"    | Table I 的 `OMPL (comp.)` 臂：同一 OMPL 规划器 + VAMP 碰撞检测 + **我们的编译投影** | 与 OMPL 原始臂、与 McVAMP 的三方对比      | OMPL $25.10$ s → OMPL(comp.) $13.25$ s（约 $2\times$），而 McVAMP $0.032$ s（相对 OMPL(comp.) 约 $400\times$） | `ompl_constrained_projected.cc`、`ompl_constrained_statespace.cc`               |
| **C3** 两阶段投影值得（对难约束）                       | 双臂问题上的 1-step 消融（Table II 中 `McVAMP (1-step)` 行）                        | 时间、路径代价                            | $2.11\to1.65$ ms（T→M）、$2.36\to1.37$ ms（B→T）；代价持平或略好                                               | 本树无该开关（见 §3.11）                                                        |
| **C4** 可扩展到高维（14/28-DoF）                        | 双臂 14-DoF（Table II）；Digit 28-DoF（Table III）                                  | 时间、成功率、路径代价                    | 双臂 $1.4$–$3.5$ ms；Digit $4.9$–$16.3$ ms；成功率 100%                                                        | `bimanual_panda_planning.cc`、`iiwa_bimanual_planning.cc`、`digit_transport.cc` |
| **C5** 多约束复合（交集流形）可解                       | Digit 四约束栈（Fig. 7）+ 真机执行                                                  | 规划时间、**关节跟踪误差**、`on_manifold` | 跟踪误差平均 $<10°$、最大 $12°$；所有轨迹可行                                                                  | `digit_transport.cc`、`digit_transport_sequence.py`                             |
| **C6** 实时到可以闭环重规划                             | 迷宫动态障碍（Fig. 6b）、Digit 动态避障（Fig. 8）                                   | 重规划频率、单次规划时间                  | 迷宫 20 Hz；Digit $<40$ ms、多数 $<10$ ms                                                                      | `viser_dynamic_obstacle_avoid.py`、`maze_meshcat_example.py`                    |
| **C7** 不牺牲路径质量                                   | Table II 的 Path Cost 列；与 IK-BiRRT / IK-GCS 对比                                 | 路径代价                                  | 与优化型 IK-GCS 同量级（略长），优于 IK-BiRRT                                                                  | Table II                                                                        |

### 3.2 度量口径与统计方式（代码级）

这一节是"如何验证"的底层：**数字是怎么算出来的**。

1. **计时只含规划**。`RRTC::solve` 内部用 `std::chrono::steady_clock` 记 `nanoseconds`，在 NN 树与缓冲区分配之后、直接连接尝试之前开始，到搜索循环结束为止（`rrtc.hh:66,292`），所以不含内存分配与种子投影。`experiments::attempt` 把它原样存入 `planning_ns`（`common.hh:544`）。**不包含**起点/终点的投影、也不包含 simplify（simplify 单独计时，`common.hh:557-571`）。这一点很重要：很多论文的"规划时间"把预处理算进去，这里的口径是"热启动后的纯搜索时间"。
2. **成功率**。`result.solved` 由 `RRTC` 在 connect 成功时置位；`attempt` 只在路径非空时置 `solved`（`common.hh:543-552`）。起点投影失败或投影后碰撞的情况会被单独打印（`result.attempted == false`），但仍然计入成功率的分母——即"投影不出种子"也算失败（`benchmark_suite.hh:112-116` 与 `common.hh:525-538`）。
3. **`on_manifold`**：对路径（若 simplify 过则对简化后路径）的**每一个** waypoint 调 `lp.satisfied(q)`，即 $F(q)\le\epsilon$（`common.hh:553-555`）。这是"严格性"的直接证据：**不是所有点都近似满足，而是逐点断言**。基准套件把它汇总为 "Paths with a waypoint off the manifold"（`benchmark_suite.hh:163`）。
4. **分位数**：Table I 的 Q1 / 中位 / Q3 / 95% 不是 C++ 直接打印的（它只打印 mean/median），而是用 `--results=FILE` 输出的每条 JSON 记录（`problem_number, success, solve_time_ns, iterations, num_cuboid_obstacles, path_cost, on_manifold`，`benchmark_suite.hh:138-146`）后处理得到。Fig. 5 的 CDF 也是按 `num_cuboid_obstacles` 分箱后画同一份 JSON。
5. **超时**：线 / 平面 OMPL 10 s、迷宫 OMPL 100 s（论文 §V-A）；超时即记为失败。CuRobo 也用同样的 10 s。
6. **统计单位**：代码默认每个问题 / 每个网格点跑一次（`--trials=1`），重复由 `--trials=N` 控制。论文对 ± 的口径没有逐表说明：Table III 的脚注是"每个任务重复 3 次"，所以至少那一组是跨重复统计；Table II 的 ± 更可能是跨重复或跨试验，阅读时不宜把它直接当作"跨问题的难度差异"。

### 3.3 公平性控制与超参扫描

论文说："We perform hyperparameter sweeps on all planners on each problem and choose the best performing configuration."

代码里这就是 `Problem::sweep` 与 `experiments::run` 的双层循环（`common.hh:321-340, 653-700`）：对笛卡尔积的每个超参组合各跑一次，最后把成功者按时间排序打印。每个实验的网格不同（`crrtc_spheres` 960 组、`bimanual_panda_planning` 1200 组、`digit_transport` 32 组……见 README 的 "The grid" 表）。

**为什么这算公平性控制**：

- baseline（OMPL/CuRobo）与 McVAMP 都做同样的扫描，取各自最优，避免"只给自己的方法调参"。
- **碰撞检测统一用 VAMP**（论文 §V："For fair comparison, OMPL uses the collision checking of VAMP; constraint evaluation is done using Pinocchio"）。于是 OMPL 与 McVAMP 的差异只剩"约束投影 + 搜索策略"。
- CuRobo 不接受关节空间目标，所以给它末端位姿目标（论文 §V-A 明确说明）——这是对不同接口的必要妥协，但也意味着 CuRobo 的"目标"信息更少。
- IK-GCS 的 70 s 预处理**不计入**规划时间（论文明确说明），这对 baseline 有利，作者仍选择这样做。

### 3.4 实验 A：单臂线/平面四类约束（Fig. 5）

**要验证什么**：在"约束本身不难、但障碍很密"的场景下，批量投影的**吞吐**是否随难度线性退化。

**设置**（对应代码 `experiments/cpp/benchmark_line_plane.cc`）：Panda 7-DoF，末端约束到直线或平面，分四种（LP / LPO / PP / PPO，见 §1.2 表）。问题集 `assets/problems/benchmark/{plane,line}_problems.json`：平面 1000 题、直线 766 题，每题**自带**一组随机长方体障碍，按障碍数分箱（平面 11 箱：0,10,…,100；直线 6 箱：0,20,…,100），起点终点都在约束上。设置固定（range 1、15 次投影迭代、OuterLM、descend 1、insert-all、扰动 0.1、无 dynamic domain、20 万次迭代），一题一次。

**观测**：按障碍数分箱的规划时间 CDF + 成功率。

**结果**（论文 Fig. 5）：McVAMP 绝大多数 $<1$ ms，$100$–$2000\times$ 快于 OMPL 与 CuRobo；成功率接近 100%，而 baseline 随障碍数增加而下降。

**它验证了方法论的哪一部分**：

- **C1 的"吞吐"**：CDF 的**形状**比中位数更有信息量——McVAMP 的曲线在 $1$ ms 处几乎竖直（方差小），说明时间不随障碍密度爆炸；baseline 的曲线长尾拖到秒级。这正是"批量 + 早退"的预期特征：障碍越多，早退越早触发，单次扩展反而更便宜。
- **严格性**：CDF 只统计成功的问题，而成功的问题里 `on_manifold` 全为真（基准套件单独计数），说明快不是靠放松约束。
- **约束类型的影响**：LPO/PPO（带姿态）比 LP/PP 难，因为等式行更多、流形维数更低（$2$ vs $5$），投影更容易被"卡住"——这与 §2.3 的"约束越硬、牛顿步越容易过冲"一致。

### 3.5 实验 B：迷宫（Table I）与 OMPL(comp.) 消融

**要验证什么**：(i) 长时程、强约束、密障碍下能否保持毫秒级；(ii) **加速到底来自哪里**。

**设置**（`experiments/cpp/benchmark_maze.cc`）：100 个起终点对，场景是长方体迷宫，手上装 9 个 1 cm 球组成的"笔"，笔尖被约束在迷宫底面（$z$ 钉住、roll/pitch 钉住、$x,y$、yaw 自由）。OMPL 超时 100 s。

**三方对比（Table I）**：

| 方法         | Mean      | Q1        | Median    | Q3        | 95%       | Succ.   |
| ------------ | --------- | --------- | --------- | --------- | --------- | ------- |
| OMPL         | 25.10     | 7.33      | 17.323    | 29.20     | 85.82     | 72%     |
| OMPL (comp.) | 13.25     | 1.00      | 3.29      | 16.80     | 52.07     | 76%     |
| **McVAMP**   | **0.032** | **0.002** | **0.006** | **0.014** | **0.058** | **82%** |

**这个消融的精妙之处**（也是本文最值得学习的地方）：`OMPL (comp.)` 把 OMPL 的 Pinocchio 投影换成作者的**编译投影**，但仍然是**逐配置串行调用**。结果只快了约 $2\times$（mean $25.1\to13.3$）——说明"把投影写快"本身收益有限。而 McVAMP 相对 OMPL(comp.) 又快了约 $400\times$（mean）/ $550\times$（median）。这个差值**只能**来自：

1. 每次扩展**同时**处理 8 个配置（SIMD）；
2. 一个 lane 失败立即否决整条边（早退），而 OMPL 要先构造 geodesic 再逐点验证；
3. 两阶段 / 粒子的采样策略提高了扩展成功率（OMPL(comp.) 成功率只从 72% 升到 76%，McVAMP 到 82%）。

也就是说：**论文用这个消融把"更快的投影"与"向量化的投影"区分开了**，这是对 C2 的直接验证。

**它同时验证了 C1 的长尾**：mean $0.032$ s 而 median $0.006$ s，均值是中位的 $5\times$，说明存在少数"难问题"耗时到几十 / 几百毫秒——这与本机复核（§3.10）中出现的 $1.13$ s 离群点一致，是 RRT 类算法的固有重尾，不是实现缺陷。

### 3.6 实验 C：双臂 14-DoF（Table II）与 1-step 消融

**要验证什么**：(i) 维度翻倍后是否仍毫秒级；(ii) **两阶段投影**是否比单阶段好；(iii) 与"用解析 IK 预计算流形"的方法（IK-BiRRT / IK-GCS）相比如何。

**设置**（`experiments/cpp/bimanual_panda_planning.cc`、`iiwa_bimanual_planning.cc`）：14-DoF 双臂，右手在左手系中的相对位姿固定（±1 mm），像共同抬一个刚体；场景是带中间隔板的两层货架；任务为在三个层（Top / Middle / Bottom）之间移动。

**结果（Table II，单位 ms，括号内为路径代价）**：

| 方法            | T→M                             | M→B                             | B→T                             |
| --------------- | ------------------------------- | ------------------------------- | ------------------------------- |
| IK-BiRRT        | $96.74\pm48.48$ $(3.69)$        | $90.11\pm21.70$ $(5.64)$        | $121.90\pm53.61$ $(6.26)$       |
| IK-GCS          | $63.47\pm1.95$ $(2.52)$         | $44.18\pm2.74$ $(4.73)$         | $77.87\pm1.19$ $(5.82)$         |
| McVAMP (1-step) | $2.11\pm1.16$ $(7.72)$          | $3.53\pm2.44$ $(6.08)$          | $2.36\pm1.63$ $(8.93)$          |
| **McVAMP**      | $\mathbf{1.65\pm0.53}$ $(7.62)$ | $\mathbf{3.49\pm1.21}$ $(7.69)$ | $\mathbf{1.37\pm0.65}$ $(8.28)$ |

**它验证了方法论的哪一部分**：

- **C4（高维）**：$14$ 维、6 行等式约束，仍有 $1.4$–$3.5$ ms 与 100% 成功率。注意标准差很小（$\pm0.5$–$2.4$ ms），说明**延迟可预测**——这对实时控制比"平均快"更重要。
- **C3（两阶段）**：1-step 是"直接在 $q_{near}\to q_{steer}$ 的弦上插值投影"（相当于省掉粒子阶段）。2-step 在三个任务段上分别快 $1.28\times$、$1.01\times$、$1.72\times$，路径代价持平或略好。**这正是论文所说的"约束越复杂，两阶段越有价值"**：T→M 与 B→T 需要穿过层板之间的窄缝，粒子阶段能找到"更容易投影的落脚点"，减少后续的迭代。
- **复现注记**：本树的 `ConstrainedLocalPlanner` 没有单独的 "1-step" 开关；把 `perturbation_scale=0` 会让 8 个 lane 完全重合，从而消掉"粒子"这一层（仍保留弦投影），是复现该消融的近似方式（`settings.hh:31`）。
- **C7（质量）**：McVAMP 的路径代价（$7.6$–$8.3$）比 IK-GCS（$2.5$–$5.8$）长 $1.3$–$3\times$，比 IK-BiRRT 长 $1.3$–$2\times$。论文的表述是 "IK-GCS produces shorter paths, owing to their optimization-based formulation"，并在 intro 说 "similar path quality"——**这里需要读者自己掂量**：严格说 McVAMP 的路径更长，但它在三个数量级更快的同一句话里，且路径仍可执行。这是一个诚实的、未被夸大的取舍（见 §3.11）。

### 3.7 实验 D：Digit 全身 28-DoF（Fig. 7/8、Table III、真机）

**要验证什么**：(i) **多约束复合**（循环投影）在真实人形上是否可行；(ii) 毫秒级规划是否足以支撑准静态全身搬运；(iii) 规划出的轨迹**能不能被真实控制器执行**（这是"物理可行性"的唯一硬证据）。

**设置**（`experiments/cpp/digit_transport.cc` + `experiments/python/digit_transport_sequence.py`）：

- 机器人：Digit，30 个关节 + 6 维浮动基；建模为 31 维构型（四元数基座 + 24 关节）。
- 四个约束（见 §2.8）：双脚 TSR（±1 mm / ±0.1 rad）、质心在支撑多边形内、两条腿的四连杆闭环、双手保持箱子刚体。
- 任务：把箱子从货架顶层搬到中层、底层，再搬回；每条腿（leg）单独规划，起终点先投影到流形。
- 真机：passivity-based 控制器跟踪关节轨迹（论文引 [34]），记录**跟踪误差**。

**结果**：

| 运动段              | S→T           | T→B            | B→M            | M→T            |
| ------------------- | ------------- | -------------- | -------------- | -------------- |
| 左膝跟踪误差（°）   | $1.70\pm1.57$ | $0.64\pm0.38$  | $1.28\pm0.34$  | $1.00\pm0.50$  |
| 左踝（tarsus）（°） | $4.37\pm2.26$ | $4.80\pm1.82$  | $8.63\pm1.73$  | $6.38\pm1.31$  |
| 右膝（°）           | $3.81\pm1.78$ | $2.57\pm1.05$  | $5.61\pm0.71$  | $4.51\pm0.63$  |
| 右踝（°）           | $3.51\pm1.73$ | $5.15\pm1.93$  | $7.74\pm1.93$  | $5.85\pm1.89$  |
| 规划时间（ms）      | $4.93\pm0.12$ | $10.11\pm1.65$ | $13.42\pm3.68$ | $16.28\pm2.11$ |

**它验证了方法论的哪一部分**：

- **C5（多约束复合）**：四个约束、18 行等式（见 §2.8 的维度账本），循环投影能收敛（否则规划直接失败），且 `on_manifold` 全真。**关键洞察**：如果循环投影不收敛，规划器会大量返回 `Trapped`，成功率会掉——但论文报告所有轨迹可行，说明"依次投影"在强耦合下仍然有效（这与 §2.4 的 Jacobi/Gauss–Seidel 讨论相关）。
- **C5 的"物理"部分**：跟踪误差小 $\Rightarrow$ 规划轨迹与控制器"想要"的轨迹接近 $\Rightarrow$ 规划没有产生违反稳定性/脚/闭环约束的漂移。论文明确说："A small tracking error indicates that the motion plan is more dynamically feasible"。这是**用控制器的表现反证规划器的物理合理性**——因为准静态规划本身不含动力学，"跟踪误差"是唯一能上真机测的代理指标。
- **C6（实时）**：$5$–$16$ ms 的规划时间，加上 20 Hz 的重规划频率，意味着"边走边算"是可行的。Table III 的最慢段（M→T，$16.3$ ms）仍在 $20$ Hz（$50$ ms 预算）以内。
- **误差的结构**：踝关节误差（$4$–$9°$）显著大于膝关节（$1$–$4°$）。这与建模一致：踝部的两个闭环连杆在规划中被"距离约束"近似，实际机构有柔性 / 间隙，控制器要额外补偿；膝关节没有闭环，误差就小。**这个结构性差异本身就是对"约束模型是否正确"的一个侧面验证**。

### 3.8 实验 E：动态环境重规划

**要验证什么**：毫秒级规划的真实价值——**感知到障碍后能在控制周期内重规划**，而不是离线算一条轨迹。

两处证据：

1. **迷宫 + 动态障碍（Fig. 6b）**：`viser` 里放一个红色球挡住当前路径，规划器在 20 Hz 下重规划并绕开。代码：`experiments/python/maze_meshcat_example.py`、`experiments/python/viser_dynamic_obstacle_avoid.py`（ROS 2 节点，订阅障碍位姿 → 重建 `Environment` → 调 `rrtc` → 发布 waypoints）。
   - 注意脚本里的一个细节：`plan_settings.range = 0.2`（比离线小），因为动态场景要更快返回；且**不对路径重新插值**，因为线性插值会离开流形（脚本注释明确写了）。这从侧面说明"路径本身已经在流形上足够密"。
2. **Digit + 动态障碍（Fig. 8a）**：向上搬运途中检测到障碍，重规划出绕行轨迹；Fig. 8b 给出跟踪误差与规划时间：**$<40$ ms，多数 $<10$ ms**，最差跟踪误差 $<8°$（论文正文说平均 $<10°$、单点最大 $12°$）。

**它验证了 C6**，并且揭示了"实时"的两个必要条件：(i) 单次规划够快（ms 级）；(ii) 重规划**频繁且稳定**（延迟方差小，否则控制器会等）。Table III 的小标准差（$\pm0.12$–$3.68$ ms）正是第二点的证据。

### 3.9 主张 ↔ 证据对照

把上面的分析压缩成一张"审稿人视角"的表：

| 论文主张                      | 证据强度 | 说明                                                                                    |
| ----------------------------- | -------- | --------------------------------------------------------------------------------------- |
| 比 SOTA 快 $100$–$2000\times$ | **强**   | 同硬件、同碰撞检测、同问题、baseline 也做了超参扫描；三方消融把归因做实                 |
| 成功率不降反升                | **强**   | 迷宫 82% vs 72% / 76%；线 / 平面 100%                                                   |
| 严格满足约束（非软惩罚）      | **强**   | 逐 waypoint 的 `on_manifold` 断言，且基准套件单独报告 off-manifold 计数（本机复现为 0） |
| 路径质量相当                  | **中**   | 双臂路径比 IK-GCS 长 $1.3$–$3\times$；论文措辞是 "similar"，实际是"可接受但更长"        |
| 高维可扩展                    | **强**   | 14-DoF 与 28-DoF 都有数据；$d$ 只进入 OuterLM 的 $d^3$，InnerLM 与 $d$ 线性             |
| 真机可执行                    | **中强** | 跟踪误差 $<12°$ 是代理指标；只有一台 Digit、少量重复（每任务 3 次）                     |
| 实时闭环                      | **强**   | 20 Hz 重规划 + 动态障碍实验                                                             |
| 两阶段优于单阶段              | **中**   | 只在一个问题类（双臂）上做了消融；加速 $1.0$–$1.7\times$，不是数量级                    |

### 3.10 本机复核测量

为了确认"代码确实在跑论文的方法"，我在本机（Intel i5-12500H，16 线程，仅用单核；与论文的 i7-13700K 不同）用仓库自带二进制做了小规模复核。**这些数字不是论文数字的复现**（CPU 不同、问题子集不同），只用于验证趋势与代码路径：

| 实验                                     | 命令                                                         | 结果                                                                                         |
| ---------------------------------------- | ------------------------------------------------------------ | -------------------------------------------------------------------------------------------- |
| 平面约束（前 30 题）                     | `benchmark_line_plane --problems=plane --first=0 --limit=30` | 30/30 成功；中位 **0.055 ms**，均值 0.060 ms；中位迭代 **1**；off-manifold **0**             |
| 直线约束（前 40 题）                     | `benchmark_line_plane --problems=line --first=0 --limit=40`  | 40/40 成功；中位 **0.156 ms**，均值 1.37 ms；中位迭代 5；off-manifold **0**                  |
| 迷宫（前 10 题）                         | `benchmark_maze --first=0 --limit=10`                        | 6/10 成功（60%）；中位 **8.8 ms**，均值 228.7 ms（含一个 1.13 s 离群点）；off-manifold **0** |
| Digit 全身搬运（单配置）                 | `digit_transport --quick`                                    | 成功；**84.7 ms**，1755 次迭代，99 个状态，`on_manifold = 1`                                 |
| 投影调试（InnerLM / OuterLM / GradDesc） | `task_space_projection --method=...`                         | 见下                                                                                         |

`task_space_projection` 的输出非常有教学意义（这是仓库里专门用来"看投影内部"的例子）：

```
Squared TSR error before projection: start 9.28303, goal 10.01384
Projection converged: start 0, goal 1; satisfied afterwards: 0, 1
Projected the straight-line rake (8 lanes, length 6.40578): all lanes converged = 0
  lane 0: gap to previous 3.94817 (unconstrained spacing 0.80072)
  ...
Largest gap 8.54175; an edge would be rejected by the continuity check (gap > 2x lane spacing)
```

它把 §2.6 的两件事同时展示了：(i) 起点投影**失败**（`start 0`，$F>\epsilon$），说明种子投影需要更大的迭代预算（`experiments::Problem::seed_iterations` 默认 200 次，而规划时每次投影只有 15/25 次；这个调试例子自身的默认是 100 次）；(ii) 弦上 8 个 lane 的投影**撕裂**（最大间距 8.54，是无约束间距 0.80 的 $10\times$），这条边会被 `trace` 的连续性检查直接拒绝。换句话说：**"投影成功"不等于"得到一条合法路径"，这正是两阶段 + 连续性检查存在的理由**。

另一个值得记录的观察：对同样的种子，InnerLM 能收敛（`start 0, goal 1`），OuterLM 与 GradDesc 在 100 次迭代内都失败（`start 0, goal 0`）；且 OuterLM 下 lane 2 的间距为 0（两个 lane 被投影到同一点，秩亏导致坍缩）。这与 §2.3 的分析一致：**InnerLM 在约束可满足时步长更"准"，OuterLM 在不可满足时才显优势**。同时它也提醒：投影方法的性能高度依赖问题，论文用超参扫描选最优是必要的。

关于迷宫：论文报告 82%，本机 10 题子集只有 60%，但 README 记录全量 100 题为 79/100。差异来自随机种子、编译器（GCC / Clang）、以及这 10 题恰好包含 4 个"难例"（其中一个耗时 1.13 s）。**结论：成功率在不同机器 / 编译器上会有几个百分点的漂移，这是 RRT 类方法的固有属性，不改变"毫秒级 + 高成功率"的总体结论。**

### 3.11 效度威胁与批判性阅读

1. **硬件不同**：论文主实验在 5.4 GHz i7-13700K，Digit 实验在 4.8 GHz Core Ultra 7 258V。跨机器的绝对时间不可直接比，但论文的对比都在同一台机器上完成，内部效度没问题。
2. **baseline 的调参预算**：OMPL / CuRobo 也做了网格扫描（论文声称），但扫描的**范围**是否与 McVAMP 同样宽？论文没有给出 baseline 的网格细节。这是常见但难以完全消除的偏差。
3. **CuRobo 是 GPU 方法**：论文用 RTX 4090 跑 CuRobo、单核 CPU 跑 McVAMP。这个对比对 McVAMP 有利（GPU 的绝对算力更高），但也符合"实时控制通常只有 CPU"的实际约束。读者应意识到这不是同硬件对比。
4. **1-step 消融只在一个问题类上做**（双臂三段的 Table II），没有在线 / 平面或 Digit 上重复。所以"两阶段对难约束更好"的证据是**单点**的，虽然方向一致。
5. **路径质量**：McVAMP 的路径比 IK-GCS 长 $1.3$–$3\times$。"similar path quality" 的说法偏乐观；更准确的表述是"路径可用、但不如优化型方法短"。论文在 Table II 里如实给出了代价，没有隐藏。
6. **`on_manifold` 的容差**：默认 $\epsilon=10^{-6}$ 对应 $1$ mm / $0.001$ rad。如果读者认为"流形约束"应当更严，可以看 `--tolerance` 的敏感性（README 说 $10^{-4}$ 会快数倍但允许 $1$ cm 松弛）——论文没有报告这个敏感性，是一个可补的实验。
7. **真机样本量**：每个任务重复 3 次（Table III 脚注），统计意义有限；跟踪误差的分布（均值 ± 标准差）比单点最大值更有信息量，但也没有做显著性检验。
8. **没有与 continuation 方法对比**：related work 提到 AtlasRRT，但实验只对比了投影式（OMPL / CBiRRT²）、优化式（CuRobo / IK-GCS）与解析 IK（IK-BiRRT）。对"投影 vs 连续化"这一根本路线的对比缺失。
9. **单线程**：论文明确说 single-core，这是优点（可预测、无锁），但也意味着没有用满现代 CPU。多核 + SIMD 的叠加是显然的 future work。

---

## 4. 代码地图

### 4.1 规划器与约束（`vamp/`）

| 文件                                                                                                                                                                                                                                   | 作用                                                     | 关键符号                                                                                                                                                                                                         |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`vamp/src/impl/vamp/planning/constraints/settings.hh`](../vamp/src/impl/vamp/planning/constraints/settings.hh)                                                                                                                        | 投影超参                                                 | `ProjMethod`、`ConstraintSettings`（method / descend_rate / tolerance / max_iterations / perturbation_scale / emit_all_waypoints / connect_slack / reached_radius2 / endpoint_tolerance2 / hold_satisfied_rows） |
| [`.../constraints/manifold/constraint.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/constraint.hh)                                                                                                                          | 约束抽象基类 + TSR 铰链公共实现                          | `Constraint`、`SolveBuffer`、`HingedTSRConstraint`、`equality_row_width`                                                                                                                                         |
| [`.../constraints/manifold/constraint_set.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/constraint_set.hh)                                                                                                                  | 约束集合与投影循环                                       | `ConstraintSet::project_all/project_any/project/satisfied`、`step_in_place`、`descend`、`set_pinned`                                                                                                             |
| [`.../constraints/local_planner.hh`](../vamp/src/impl/vamp/planning/constraints/local_planner.hh)                                                                                                                                      | **两阶段约束扩展**                                       | `ConstrainedLocalPlanner::steer`、`trace`、`connect_within`、`generate_stddev_samples`                                                                                                                           |
| [`.../constraints/utils.hh`](../vamp/src/impl/vamp/planning/constraints/utils.hh)                                                                                                                                                      | 步长积分与四元数 retraction                              | `integrate_step`、`renormalize_so3`                                                                                                                                                                              |
| [`.../constraints/chart.hh`](../vamp/src/impl/vamp/planning/constraints/chart.hh)                                                                                                                                                      | 切空间（chart）构造：对 $J_{active}^\top$ 做列主元 QR    | `Chart`、`ChartBuilder`（**未接入本树的规划器**，见 §6）                                                                                                                                                         |
| [`.../constraints/manifold/task_space_constraint.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/task_space_constraint.hh)                                                                                                    | TSR 约束（$6n_{eef}$ 行）                                | `TaskSpaceConstraint`                                                                                                                                                                                            |
| [`.../constraints/manifold/bimanual_task_space_constraint.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh)                                                                                  | 相对位姿约束（6 行）                                     | `BimanualTaskSpaceConstraint`                                                                                                                                                                                    |
| [`.../constraints/manifold/com_constraint.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/com_constraint.hh)                                                                                                                  | 质心支撑多边形（2 行，半平面铰链）                       | `CoMConstraint`                                                                                                                                                                                                  |
| [`.../constraints/manifold/closed_loop_constraint.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/closed_loop_constraint.hh)                                                                                                  | 闭环杆长（每环 1 行）                                    | `ClosedLoopConstraint`                                                                                                                                                                                           |
| [`.../constraints/manifold/pfaffian_constraint.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/pfaffian_constraint.hh)                                                                                                        | 速度约束基类（无位置误差，只收紧切空间）                 | `PfaffianConstraint`                                                                                                                                                                                             |
| [`.../constraints/manifold/lead_screw_constraint.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/lead_screw_constraint.hh) / [`twist_constraint.hh`](../vamp/src/impl/vamp/planning/constraints/manifold/twist_constraint.hh) | 丝杠 / 旋量速度约束（扩展，论文未用）                    | `LeadScrewConstraint`、`TwistConstraint`                                                                                                                                                                         |
| [`.../planners/rrtc.hh`](../vamp/src/impl/vamp/planning/planners/rrtc.hh)                                                                                                                                                              | Alg. 1                                                   | `RRTC::solve`、`insert_chain`、dynamic domain、balance                                                                                                                                                           |
| [`.../planning/simplify.hh`](../vamp/src/impl/vamp/planning/simplify.hh)                                                                                                                                                               | 流形上的 shortcut（用同一个局部规划器）                  | `simplify`                                                                                                                                                                                                       |
| [`.../robots/panda.hh`](../vamp/src/impl/vamp/robots/panda.hh)                                                                                                                                                                         | 生成的 Panda 内核                                        | `tsr_error`（:41164）、`solve_tsr_error_lm_{inner,outer}`（:41993 / :42040）、`fkcc`                                                                                                                             |
| [`.../robots/digit.hh`](../vamp/src/impl/vamp/robots/digit.hh)                                                                                                                                                                         | 生成的 Digit 内核                                        | `com_jacobian`（:18992）、`closed_loop_error`（:20592）、`solve_closed_loop_error_lm_inner`（:21213）                                                                                                            |
| [`.../bindings/python/robot_helper.hh`](../vamp/src/impl/vamp/bindings/python/robot_helper.hh)                                                                                                                                         | Python API：`project`/`satisfied`/`simplify`/受限 `rrtc` | `constraint_project`、`solve_single_constrained`                                                                                                                                                                 |

### 4.2 代码生成（`cricket/`）

| 文件                                                                                                           | 作用                                                                 |
| -------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| [`cricket/src/tracing/constraints.cc`](../cricket/src/tracing/constraints.cc)                                  | 追踪 TSR / 双臂 / CoM / 闭环 / 丝杠 / 旋量误差，并追踪"一步 LM"求解  |
| [`cricket/src/tracing/se3_ops.hh`](../cricket/src/tracing/se3_ops.hh)                                          | `so3_log_smooth`、`sinc_smooth`（可微 log 映射）                     |
| [`cricket/src/tracing/cholesky.hh`](../cricket/src/tracing/cholesky.hh)                                        | 手写 Cholesky（Eigen `.llt()` 不可追踪）                             |
| [`cricket/src/fkcc_gen.cc`](../cricket/src/fkcc_gen.cc)、[`cricket/src/codegen.cc`](../cricket/src/codegen.cc) | 生成器入口与代码输出                                                 |
| [`cricket/resources/`](../cricket/resources/)                                                                  | 机器人配方（`digit.json`、`g1_unitree.json`、`bimanual_iiwa.json`…） |

### 4.3 实验（`experiments/`）

| 文件                                                                                                                                                                                       | 对应论文内容                                                                  |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------- |
| [`experiments/cpp/common.hh`](../experiments/cpp/common.hh)                                                                                                                                | 问题描述、超参网格、计时、`on_manifold` 断言、CSV 输出（§3.2 的口径都在这里） |
| [`experiments/cpp/benchmark_suite.hh`](../experiments/cpp/benchmark_suite.hh)                                                                                                              | 多问题基准套件（成功率 / 中位时间 / JSON 记录）→ Fig. 5、Table I              |
| [`experiments/cpp/benchmark_line_plane.cc`](../experiments/cpp/benchmark_line_plane.cc)                                                                                                    | 线 / 平面 4 类约束 → Fig. 5                                                   |
| [`experiments/cpp/benchmark_maze.cc`](../experiments/cpp/benchmark_maze.cc)                                                                                                                | 迷宫 100 题 → Table I、Fig. 6                                                 |
| [`experiments/cpp/crrtc_spheres.cc`](../experiments/cpp/crrtc_spheres.cc)                                                                                                                  | 球笼 + 直线 TSR（960 组网格）→ 方法验证 / 消融                                |
| [`experiments/cpp/crrtc_maze.cc`](../experiments/cpp/crrtc_maze.cc)、[`crrtc_transport_mug.cc`](../experiments/cpp/crrtc_transport_mug.cc)                                                 | 迷宫 / 架子场景的单问题研究                                                   |
| [`experiments/cpp/bimanual_panda_planning.cc`](../experiments/cpp/bimanual_panda_planning.cc)                                                                                              | 双臂 14-DoF → Table II                                                        |
| [`experiments/cpp/iiwa_bimanual_planning.cc`](../experiments/cpp/iiwa_bimanual_planning.cc)                                                                                                | 两个 KUKA iiwa（需生成机器人）                                                |
| [`experiments/cpp/digit_transport.cc`](../experiments/cpp/digit_transport.cc)                                                                                                              | Digit 四约束栈 → Fig. 7、Table III                                            |
| [`experiments/cpp/task_space_projection.cc`](../experiments/cpp/task_space_projection.cc)                                                                                                  | 投影调试（本文 §3.10 的输出）                                                 |
| [`experiments/cpp/ompl_constrained_projected.cc`](../experiments/cpp/ompl_constrained_projected.cc)、[`ompl_constrained_statespace.cc`](../experiments/cpp/ompl_constrained_statespace.cc) | OMPL baseline 与 OMPL(comp.) 消融 → Table I                                   |
| [`experiments/python/digit_transport_sequence.py`](../experiments/python/digit_transport_sequence.py)                                                                                      | 真机 / 仿真的三段搬运序列（reach → transport → transport back）               |
| [`experiments/python/viser_dynamic_obstacle_avoid.py`](../experiments/python/viser_dynamic_obstacle_avoid.py)                                                                              | ROS 2 动态障碍重规划 → Fig. 6b / Fig. 8                                       |
| [`experiments/python/benchmark_common.py`](../experiments/python/benchmark_common.py)                                                                                                      | 与 C++ 套件等价的 Python 实现                                                 |

---

## 5. 复现命令

```bash
# 0) 构建（需要 AVX2；机器人头文件很大，按需构建单个目标）
cmake -S experiments -B build/experiments -G Ninja
cmake --build build/experiments --target digit_transport -j

# 1) 单臂线/平面（Fig. 5）：1000/766 题，输出 JSON 后可画 CDF
./build/experiments/bin/benchmark_line_plane --problems=plane --results=/tmp/plane.json
./build/experiments/bin/benchmark_line_plane --problems=line  --results=/tmp/line.json

# 2) 迷宫（Table I）
./build/experiments/bin/benchmark_maze --results=/tmp/maze.json

# 3) 投影内部（§2.3/§2.6 的直观演示；注意用 --output 避免覆盖默认文件）
./build/experiments/bin/task_space_projection --method=InnerLM --output=/tmp/proj.csv

# 4) 双臂 / Digit 的完整超参网格（较慢）
./build/experiments/bin/bimanual_panda_planning
./build/experiments/bin/digit_transport --quick --output=/tmp/digit.csv

# 5) Python API（与论文等价的三种规划器 + 约束）
PYTHONPATH=$PWD/vamp/src python experiments/python/digit_transport_sequence.py --output_prefix=/tmp/leg
PYTHONPATH=$PWD/vamp/src python experiments/python/benchmark_maze.py
```

> 提示：`digit_transport`、`task_space_projection`、`crrtc_spheres` 的默认输出分别是 `trajectory.txt` / `trajectory.txt` / `spheres.txt`（均在仓库根目录，已被 `.gitignore` 忽略）。批处理时务必用 `--output` 或 `--spheres_dump` 指定路径。

---

## 6. 论文与代码的差异、阅读注记

这一节记录"论文没写、但读代码才能知道"的点，以及论文与实现之间需要小心的地方。

1. **循环投影的线性化点**：论文 Alg. 2 是 Gauss–Seidel（每步用更新后的 $q$ 重新求 $F_i,J_i$），代码 `descend` 是"一次全栈求值 + 依次施加"（Jacobi 线性化点）。见 §2.4。这会影响强耦合约束的收敛轮数，但不改变算法本质。
2. **两阶段的"递归"实现方式**：论文 Alg. 2 第 15 行说"递归插值并投影直到分辨率 $\delta$"，代码把它实现为**对相邻投影点之间的 gap 做细分子步校验**（`n_steps = ceil(max_gap * resolution)`），子步**不入树**。语义上等价于"更密的连续碰撞检测"，但树上节点数不等于子步数。
3. **式 (2) 的顺序**：论文 $T_{err}=T_{ref}^{-1}(T_l^{-1}T_r)$，代码 $T_{err}=(T_l^{-1}T_r)T_{ref}^{-1}$。零集相同，误差度量相差旋转共轭。见 §2.8。
4. **CoM 的"最近点" vs "半平面和"**：论文说 "nearest point in the hull"，代码是"违反边的外法向和"。单边违反时等价，顶点外略有不同。见 §2.8。
5. **`equality_row_width = 0.5` 不是投影参数**：它只用于 `active_rows()`（切空间 / chart），不改变 hinge 投影。README 里"宽于 0.5 视为 slab、窄于 0.5 视为 pinning"的说法是**针对 chart 的分类**，不要误读为"投影只处理等式行"。
6. **`chart.hh` 没有被规划器使用**：`ChartBuilder` / `Chart` / `to_chart` 在本树中没有任何调用点（`grep` 可验证），它是"用切空间坐标做连续化 / 局部参数化"的基础设施，为未来的 continuation 方法预留。论文正文也没有提到它。**不要把它当成方法的一部分**。
7. **Pfaffian / 丝杠 / 旋量约束是扩展**：`LeadScrewConstraint`、`TwistConstraint` 在 Python 绑定里可用，但论文的四个实验都没有用。它们展示的是"约束插件化"的设计能力。
8. **Digit 的维度**：论文 28-DoF（22 关节 + 6 浮动基），代码 31 维（7 + 24 关节，`sample_dimension=30`）。见 §2.8 的"维度账本"。
9. **NaN 依赖**：整个约束栈依赖"$\log$ 在恒等旋转处产生 NaN，再被 hinge 夹成 0"这一行为。Clang 的 `-ffinite-math-only` / `-fno-honor-nans` 会破坏它（README §9 明确警告）。
10. **`interpolate_dump` 会离开流形**：`digit_transport` 的 `problem.interpolate_dump = true` 会在写 CSV 前把路径线性插值到碰撞分辨率——线性插值**不**保持约束。所以落盘的 CSV 不保证逐点 `on_manifold`（`on_manifold` 是在插值前算的）。这是"给控制器用"与"给分析用"的口径差异。
11. **论文与 README 的数字有差别**：论文迷宫 82%，README 记录本树 79/100；这是编译器 / 随机种子 / 子集差异，不是矛盾。
12. **加速倍数的措辞不一致**：摘要 "100–1000×"、Fig. 5 caption "1000×"、结论 "over 500×"、正文 "100–2000×"。这些是不同实验 / 不同分位数的口径，阅读时以具体表格为准。

---

## 7. 一页公式汇总

$$
\begin{aligned}
&\text{流形：} && \mathcal M=\bigcap_i F_i^{-1}(0),\quad \mathcal M_{free}=\mathcal M\cap\mathcal C_{free}\\
&\text{TSR 误差：} && T^r_{obj}(q)=(T^r_w)^{-1}T^e_w(q)(T^r_e)^{-1},\quad
d(q)=\big[t;\ \log(R)^\vee\big]\in\mathbb R^6\\
&\text{铰链：} && e_j=\min(d_j-l_j,0)+\max(d_j-u_j,0),\quad F(q)=\textstyle\sum_j e_j^2\le\epsilon\\
&\text{LM 步：} && \Delta q=-J^\top(JJ^\top+\lambda I)^{-1}e\ \ \text{(Inner)},\quad
\Delta q=-(J^\top J+\lambda I)^{-1}J^\top e\ \ \text{(Outer)}\\
&\text{更新：} && q\leftarrow q+\alpha\Delta q,\quad \text{再夹限位、再归一化四元数}\\
&\text{循环投影：} && q^{(t+1)}=(P_m\circ\cdots\circ P_1)(q^{(t)})\\
&\text{粒子：} && q^{(i)}=q_s+\Delta(1+m_i\sigma)v,\quad m=(0,-0.25,+0.25,-0.5,+0.5,-0.75,+0.75,-1)\\
&\text{弦：} && q^{(i)}=q_s+\tfrac{i}{n}(q_{proj}-q_s),\quad
\|q^{(i)}-q^{(i-1)}\|^2\le 4\big(\tfrac{\|q_{proj}-q_s\|}{n}\big)^2\\
&\text{加密：} && n_{steps}=\lceil g_{\max}/\delta\rceil,\quad \delta=1/32,\quad
\|\xi^{(j)}-\text{lerp}\|\le 2\delta\\
&\text{接受：} && \|q_{proj}-q_s\|\le 2\Delta,\quad \text{collision-free},\quad \|q_{proj}-q_{target}\|^2\le r_{reached}^2
\end{aligned}
$$

---

*本文依据仓库当前 checkout（`vamp` 子模块 `6b454dc`，`experiments`/`cricket` 同版本）与 arXiv:2604.13323v1 撰写。代码行号可能随分支演进而变化，函数名稳定。*
