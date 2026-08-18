# SBM FPGA 算法实现 — 代码架构与概要解析

> **算法基础**：Shape Based Matching（形状匹配），源自论文《Gradient Response Maps for Real-Time Detection of Textureless Objects》与 LineMOD 框架。参考实现为 `01_技术预研报告/04_参考代码/line2Dup.cpp`。
>
> **工程目标**：在 Zynq UltraScale+（XCZU3EG）PL 端以纯流水线 Verilog 实现从图像预处理到候选匹配位置输出的完整模板匹配管线，PS 端 C 代码做 NMS 后处理。所有 RTL 与 C 参考模型可逐字节联仿比对。

---

## 目录

- [1. 算法背景与设计思路](#1-算法背景与设计思路)
- [2. 目录文件结构](#2-目录文件结构)
- [3. 系统架构总览](#3-系统架构总览)
- [4. 几何参数单一真源（sbm\_geometry.vh）](#4-几何参数单一真源sbm_geometryvh)
- [5. 各模块详细解析](#5-各模块详细解析)
  - [5.1 SBM.Alg.1 高斯平滑（sbm\_alg1\_gaussian）](#51-sbmalg1-高斯平滑sbm_alg1_gaussian)
  - [5.2 SBM.Alg.2 Sobel 梯度计算（sbm\_alg2\_sobel）](#52-sbmalg2-sobel-梯度计算sbm_alg2_sobel)
  - [5.3 SBM.Alg.3 梯度量化（sbm\_alg3\_quantize）](#53-sbmalg3-梯度量化sbm_alg3_quantize)
  - [5.4 SBM.Alg.8 扩散 OR 运算（sbm\_alg8\_spread）](#54-sbmalg8-扩散-or-运算sbm_alg8_spread)
  - [5.5 SBM.Alg.9 响应图 LUT 查表与线性化落盘（sbm\_alg9\_lut）](#55-sbmalg9-响应图-lut-查表与线性化落盘sbm_alg9_lut)
  - [5.6 SBM.Alg.11 相似度滑窗累加（sbm\_alg11\_accum）](#56-sbmalg11-相似度滑窗累加sbm_alg11_accum)
  - [5.7 SBM.Alg.13 NMS 后处理（sbm\_alg13\_nms.c）](#57-sbmalg13-nms-后处理sbm_alg13_nmsc)
- [6. 支撑模块](#6-支撑模块)
- [7. 黄金参考模型与测试平台](#7-黄金参考模型与测试平台)
- [8. 关键数据流与地址布局](#8-关键数据流与地址布局)
- [9. 构建与联仿](#9-构建与联仿)
- [10. 当前状态与审查记录](#10-当前状态与审查记录)
- [11. 算法正确性与落地可行性评审（2026-08-15）](#11-算法正确性与落地可行性评审2026-08-15)
  - [11.1 总体结论](#111-总体结论)
  - [11.2 算法正确性（论文 ↔ C++ ↔ RTL）](#112-算法正确性论文--c--rtl)
  - [11.3 关键缺陷（D1/D2）](#113-关键缺陷d1d2)
  - [11.4 接口与数据流一致性](#114-接口与数据流一致性)
  - [11.5 工程落地可行性（XCZU3EG）](#115-工程落地可行性xczu3eg)
  - [11.6 验证完备性与盲点](#116-验证完备性与盲点)
  - [11.7 行动建议（按优先级）](#117-行动建议按优先级)
  - [11.8 第二轮全量复审判定（2026-08-15 融合）](#118-第二轮全量复审判定2026-08-15-融合)
- [12. N-8 前端协议统一与验证闭环（2026-08-18）](#12-n-8-前端协议统一与验证闭环2026-08-18)
  - [12.1 任务背景与验收标准](#121-任务背景与验收标准)
  - [12.2 任务 1：统一 AXI4-Stream 帧协议](#122-任务-1统一-axi4-stream-帧协议)
  - [12.3 任务 2：sbm_gauss_h 行尾冲刷硬件强制](#123-任务-2sbm_gauss_h-行尾冲刷硬件强制)
  - [12.4 任务 3：C-golden 功能 TB 与 Makefile 接入](#124-任务-3c-golden-功能-tb-与-makefile-接入)
  - [12.5 缺陷捕获与修复总账](#125-缺陷捕获与修复总账)
  - [12.6 回归结果](#126-回归结果)

---

## 1. 算法背景与设计思路

### 1.1 算法原理

Shape Based Matching 是一种基于梯度方向的模板匹配方法，核心思想：

1. **梯度方向量化**：对场景图计算 Sobel 梯度，将连续方向角量化为 8 个离散方向（label 0–7），幅值低于阈值的像素标记为弱梯度。
2. **响应图扩散（Spread）**：对量化后的方向图做 T×T 邻域按位或运算，使每个像素携带其 T×T 邻域内的所有方向信息。扩散后的图称为"响应图"（Response Map）。
3. **相似度 LUT**：预计算一个 8 方向 × 32 项的查找表 `SIMILARITY_LUT`。给定模板特征方向 `ori` 和场景像素的 8bit 扩散响应 `byte`，查表得到相似度得分：
   ```
   resp_ori(x,y) = max(SIM[ori*32 + byte&0xF], SIM[ori*32 + 16 + byte>>4])
   ```
4. **线性化（Linearize）**：将 8 方向响应图重排为线性内存布局，使得模板匹配时的地址计算可一步完成：
   ```
   addr = RESP_BASE + (ori*T*T + block)*CELLS + cell
   其中 block = (y%T)*T + x%T, cell = (y/T)*WC + x/T
   ```
5. **滑窗累加**：对模板的每个特征点 f（含像素坐标和方向标签），在所有候选位置 j 上累加响应值：`score[j] += lm[ori(f)][block(f)][lm_index(f) + j]`。
6. **NMS 后处理**：对累加得分做归一化、IoU 非极大抑制、级 0 细化，输出最终匹配位置。

### 1.2 多分辨率策略

系统采用两级金字塔：
- **级 0**（Level-0）：相机原始帧经边缘复制填充后的尺寸，供 alg1/alg2/alg3 使用。
- **级 1**（Level-1）：级 0 二倍降采样后的金字塔层，供 alg8/alg9/alg11/alg13 使用。粗层匹配快速定位候选区域，再在级 0 精细化。

### 1.3 FPGA 实现策略

| 策略 | 说明 |
|---|---|
| **全流水线** | alg1–alg9 采用 AXI4-Stream 逐像素流水线，1 像素/时钟吞吐，无气泡 |
| **分离式高斯** | 7×7 高斯分解为水平 7 抽头 + 垂直 7 抽头，节省行缓冲 |
| **CORDIC 方向角** | 使用 Xilinx CORDIC IP（向量模式 atan2），全流水 21 拍延迟 |
| **移位加法树** | 高斯/Sobel 系数全部用移位实现（`<<`/`>>`），0 DSP |
| **LUTRAM 并行查表** | alg9 对 8 个方向各做低/高半字节 2 次查表（共 16 路并行 LUTRAM 读），一拍得到 8 方向响应 |
| **双 bank 转置** | alg9 用 ping-pong SDPRAM 做响应图转置，写/读并发 |
| **多通道并行累加** | alg11 用 LANES（默认 24）通道并行累加，256bit AXI4 读主机 |
| **PS 端 NMS** | alg13 以 C 语言在 PS 端运行，通过 AXI4-Lite 读取 PL 候选 |

---

## 2. 目录文件结构

```
Alg.Code/
├── 算法管线源码（SBM 流水线各 stage）
│   ├── sbm_alg1_gaussian.v          # 高斯滤波顶层（AXI4-Stream 封装）
│   ├── sbm_gauss_h.v                # 高斯 水平方向 7 抽头
│   ├── sbm_gauss_v.v                # 高斯 垂直方向 7 抽头（6 行缓冲轮转）
│   ├── sbm_alg2_sobel.v             # Sobel 梯度 + CORDIC 方向角
│   ├── sbm_alg3_quantize.v          # 幅值/角度量化 + 3×3 投票 + 滞后门控
│   ├── sbm_alg8_spread.v            # T×T 前向窗口按位或扩散
│   ├── sbm_alg9_lut.v               # LUT 响应图 + 双 bank 转置 + AXI4-MM 落盘
│   ├── sbm_alg11_accum.v            # 响应图累加顶层（多 lane + Top-32 收集）
│   ├── sbm_accum_lane.v             # 累加 单 lane 引擎（双 BRAM 乒乓 RMW）
│   └── sbm_alg13_nms.c             # NMS 后处理（PS 端 C 语言）
├── 支撑模块
│   ├── cordic_atan2_beh.v           # CORDIC 反正切行为模型（仿真用）
│   ├── cordic_atan2_func.vh         # CORDIC 角度换算函数（被 include）
│   ├── sbm_geometry.vh              # 几何参数单一真源（换相机只改此处）
│   ├── xpm_memory_sdpram_beh.v      # XPM 双口 RAM 行为模型
│   ├── xpm_memory_spram_beh.v       # XPM 单口 RAM 行为模型
│   └── xpm_fifo_async_beh.v         # XPM 异步 FIFO 行为模型
├── 黄金参考模型
│   └── golden_sbm.c                 # C 参考实现（生成 stimulus/hits/expected）
├── 测试平台（testbench）
│   ├── tb_sbm_alg1_gaussian.v       # C-golden 两级高斯功能 TB（N-8 闭环，重写）
│   ├── tb_sbm_alg2_sobel.v          # C-golden Sobel+CORDIC 功能 TB（N-8 闭环，新增）
│   ├── tb_sbm_alg3_quantize.v       # C-golden 量化投票功能 TB（N-8 闭环，新增）
│   ├── tb_sbm_alg2_cosim.v
│   ├── tb_sbm_alg3_cosim.v
│   ├── tb_sbm_alg8_spread.v
│   ├── tb_sbm_alg9_cosim.v
│   ├── tb_sbm_alg11_accum.v
│   ├── tb_sbm_accum_lane_cosim.v
│   └── tb_sbm_geom_check.v          # 多分辨率几何自检
├── 数据与配置
│   ├── similarity_lut.mem           # SIMILARITY_LUT 数据（256 行 8bit，源自 line2Dup.cpp L737）
│   └── （stimulus.hex / expected_pos.txt / hits.txt 由 golden_sbm.c 再生成，make 自动构建，不入库存）
└── 构建 / 文档 / 工程
    ├── Makefile                     # 编译 / 联仿构建脚本
    ├── README.md                    # 本文件
    ├── README_cosim.md              # 联仿专项说明
    └── package_gauss.tcl            # 高斯模块 IP 打包（Vivado Tcl）
```

---

## 3. 系统架构总览

### 3.1 数据流管线

```
                         级 0（原始分辨率，边缘填充后）
  ┌─────────────────────────────────────────────────────────────┐
  │                                                             │
  │  AXI-S   ┌─────────┐  ┌──────────────┐  ┌──────────┐        │
  │  8bit ─▶ │ Alg.1   │─▶│   Alg.2      │─▶│  Alg.3   │        │
  │  像素流   │ 高斯     │  │ Sobel+CORDIC │  │  量化    │        │
  │          │ 7×7     │  │ mag2+angle   │  │ 8dir 1hot│        │
  │          └─────────┘  └──────────────┘  └──────────┘        │
  │                                                             │
  └──────────────────────────┬──────────────────────────────────┘
                             │ 降采样 ×2
                         级 1（金字塔粗层） ▼
  ┌──────────────────────────┴──────────────────────────────────┐
  │  ┌──────────┐  ┌──────────────────────┐                      │
  │  │  Alg.8   │─▶│       Alg.9          │── AXI4-MM ──▶ DDR    │
  │  │ T×T OR   │  │ LUT查表+转置+线性化    │   64bit 写            │
  │  │ 扩散      │  │ 8方向响应图落盘        │                      │
  │  └──────────┘  └──────────────────────┘                      │
  │                       │ DDR 响应图                           │
  │                       ▼                                      │
  │  ┌──────────────────────────┐    ┌────────────────────┐      │
  │  │       Alg.11             │    │     Alg.13         │      │
  │  │  多 lane 滑窗累加          │─-─▶│   NMS 后处理       │      │
  │  │  + Top-32 候选收集        │    │   (PS 端 C)        │      │
  │  │  AXI4-Lite 输出候选       │    │   IoU + 级0细化    │      │
  │  └──────────────────────────┘    └────────────────────┘      │
  └─────────────────────────────────────────────────────────────┘
```

### 3.2 接口协议分层

| 层级 | 协议 | 使用模块 |
|---|---|---|
| 像素流（级 0/级 1 管线间） | AXI4-Stream（8bit tdata + tuser/tlast；**N-8 闭环后全链统一：tuser=帧首像素、tlast=帧末像素，行边界由各模块内部列计数器自行派生，不依赖外部行同步**，见 §12） | alg1→alg2→alg3, alg8→alg9 |
| 响应图落盘 | AXI4 Memory-Mapped（64bit 写主机） | alg9 → DDR |
| 累加读回 | AXI4 Memory-Mapped（256bit 读主机） | alg11 ← DDR |
| 配置/候选 | AXI4-Lite（32bit 寄存器/FIFO） | alg11 ←→ PS |
| 特征输入 | AXI4-Lite 写 → 异步 FIFO → 核心域 | PS → alg11 |

---

## 4. 几何参数单一真源（sbm_geometry.vh）

### 4.1 设计动机

`IMG_W/IMG_H` 取决于相机，属于易变参数。旧代码在多个模块硬编派生常量（WC=313、CELLS=97969、CHUNK=3652、BANK_DEPTH=2048、各计数器位宽），换一次相机要在十几处同步改数，漏改即产生**静默**的地址错位或位置丢弃。

### 4.2 单一真源设计

只修改顶部 4 个宏，其余全部自动派生：

```verilog
// ─── 用户输入区（只改这里）───
`define SBM_CAM_W       5000      // 相机原始帧宽
`define SBM_CAM_H       5000      // 相机原始帧高
`define SBM_T           8         // line2Dup 的 T（块边长，须为 2 的幂）
`define SBM_LANES       24        // alg11 累加通道数
```

派生链：

```
级0 = ALIGN(CAM, 16T)           → 5000 → 5120（保证降采样后级1满足约束）
级1 = 级0 / 2                   → 2560
WC  = 级1 / T                   → 320
HC  = 级1 / T                   → 320
CELLS = WC × HC                 → 102400
```

### 4.3 对齐约束链

```
级1 须为 T 整数倍 (linearize 断言同口径)
  ∧ WC = 级1/T 须被 8 整除 (alg9 64bit AXI 写 8B 对齐)
    ⇒ 级1 须为 8T 整数倍
      ⇒ 级0 须为 16T 整数倍
        ⇒ 级0 = ceil(CAM / 16T) × 16T
```

### 4.4 位宽派生宏

```verilog
`define SBM_W(n)  (((n) < 2) ? 1 : $clog2(n))   // 表示 0..n-1 所需位宽
```

各模块内部所有计数器、地址、FIFO 指针位宽均由此宏按几何量派生，不再写死。换相机时只需修改 4 个宏，全链路自动跟随。

### 4.5 防御性断言

- `IMG_W % T != 0` → 编译期 `$error`
- `WC % 8 != 0` → 编译期 `$error`（AXI 8B 对齐）
- `SEG_BEATS > 256` → 编译期 `$error`（AXI awlen 上限）
- `cfg_tp > LANES*CHUNK` → 运行期 `$error`（容量溢出）
- DDR 区间 > 4GB → 编译期 `$error`

---

## 5. 各模块详细解析

### 5.1 SBM.Alg.1 高斯平滑（sbm_alg1_gaussian）

#### 文件

| 文件 | 角色 |
|---|---|
| `sbm_alg1_gaussian.v` | 顶层封装，AXI4-Stream 接口，组合 H+V 两级 |
| `sbm_gauss_h.v` | 7 抽头水平高斯滤波 |
| `sbm_gauss_v.v` | 7 抽头垂直高斯滤波（6 行缓冲轮转） |

#### 算法

- **核系数**：`[2, 7, 14, 18, 14, 7, 2] / 64`（σ ≈ 1.37），与 OpenCV `GaussianBlur(7×7, σ=0)` 自动推导核一致。
- **分离实现**：7×7 二维卷积分解为水平 7 抽头 + 垂直 7 抽头两级一维卷积，行缓冲从 6 行降至 6 行（垂直级），水平级仅需 7 级移位寄存器。
- **舍入**：四舍五入 `(sum + 32) >> 6`，全部移位实现，0 DSP。

#### 边界处理（BORDER_REPLICATE）

与 `line2Dup.cpp quantizedOrientations` 一致：

| 边界 | 处理方式 |
|---|---|
| 左边界 | 行首 `i_row_start=1` 时 7 个抽头整体装载行首像素 |
| 右边界 | 行末后内部自动补 3 拍，以末像素持续右移复制 |
| 上边界 | 帧首行（第 0 行）期间 6 个行缓冲全部写入第 0 行 |
| 下边界 | 末行结束后自生成 3 行底部复制 |

#### 中心对齐

因果窗口输出 = 中心窗口滞后 3 列/3 行，故每行丢弃前 3 个因果输出，每帧恰好输出 `IMG_W × IMG_H` 个有效像素。

#### 资源与延迟

| 指标 | 值 |
|---|---|
| 吞吐 | 1 像素/时钟 |
| 水平级延迟 | 2 拍（移位寄存器 1 + 加法树 1） |
| 垂直级行缓冲 | 6 × IMG_W × 8bit（XPM SDPRAM） |
| DSP | 0 |
| 帧间约束 | 底部冲刷期间 `s_axis_tready` 拉低，帧间强制 ≥3 行间隙（底部复制排空），由硬件反压保证，无需上游软约定 |

#### N-8 协议统一整改（2026-08-18）

顶层已重写为与 alg2/alg3 对齐的帧协议风格：

- **N-8a**：删除旧版 `row_active` 对输入消费的门控（原 `i_valid = tvalid && row_active` 在帧首前/行间空隙拒绝合法数据，帧首拍被吞、每行第 0 像素丢失）。数据消费仅取决于 AXI-S 握手（`s_axis_tvalid && s_axis_tready`）。
- **行边界内部派生**：自由列计数器 `pix_cnt` + 行计数器 `row_cnt`，仅在握手成立时推进；行首标记为寄存单拍脉冲（N-9：组合判据 `pix_cnt==0` 会因寄存器更新时点产生 2 拍宽脉冲，把下游行同步带偏一拍）。
- **帧状态机**：tuser 帧首拍显式清零计数器与冲刷状态（帧间无状态残留）；tlast 帧末拍置位底部冲刷状态。
- **N-8b**：gauss_h 行尾补 3 拍期间 `o_ready=0`，顶层联合拉低 `s_axis_tready` 硬件强制行间隙（照搬 alg8 P2 修法），背靠背满速送数亦不吞补零拍。

#### 顶层接口（AXI4-Stream）

```
输入: s_axis_tvalid/tready/tdata[7:0]/tuser/tlast
输出: m_axis_tvalid/tready/tdata[7:0]/tuser/tlast
```

---

### 5.2 SBM.Alg.2 Sobel 梯度计算（sbm_alg2_sobel）

#### 算法

- **Sobel 核**：
  ```
  Gx = [-1 0 1; -2 0 2; -1 0 1],  Gy = [-1 -2 -1; 0 0 0; 1 2 1]
  ```
- **梯度幅值平方**：`mag2 = dx² + dy²`（22bit，避免余量 0 溢出），使用 2 个 DSP48E2。
- **方向角**：由 Xilinx CORDIC IP（向量模式 atan2）计算，输入 `{dx, dy}`（12bit 有符号），输出 16bit 归一化相位（`±π` 全圆）。

#### 关键修正

1. **CORDIC 输入位序**：按 PG105 修正为 `{X_IN, Y_IN} = {dx, dy}`（原版 `{dy, dx}` 打包导致角度差约 90°）。
2. **上边界复制**：`preload_flag` 覆盖整个第 0 行（原版仅帧首预拍触发，lb2 写入陈旧数据）。
3. **左边界复制**：行首 3 列整体装载（原版缺失）。
4. **帧尾补 1 行**：末行结束后补 1 行复制（原版缺失，末行输出丢失）。
5. **行缓冲**：改为 2 缓冲按行号 mod 2 轮转 + 端口 B 读取（原版链式 `lb2.dina=lb1_out` 在读延迟 1 拍下错位）。

#### N-8 闭环新增修正（⑥–⑪，C-golden TB2 逐项捕获）

| 编号 | 缺陷 | 修法 |
|---|---|---|
| N-14 | 帧首拍被 `row_active=0` 门控丢弃，整帧左移 1 列，行尾补拍触发永不命中 | `w_in_valid` 补入 `s_axis_tuser` 项 |
| N-19 | 行尾补拍依赖"上游行间 ≥1 拍空闲"软契约，满速背靠背时补拍被吞、输出相位逐行对角漂移 | 补拍期间拉低 `s_axis_tready`，tready 纳入消费条件（照搬 alg8 P2） |
| N-20 | 加法树 10bit 无符号和直接 `$signed()` 相减，≥512 别名为负，亮区 dx/dy 错乱 | 先零扩展为 12bit 有符号再相减 |
| N-22 | 末行 pad 窗口当前行源误用 lb 读（应为 `cur_d=last_pix`） | 切换条件收紧为 `flush_row && !pad_d1` |
| N-23 | 帧尾补行行首装载三行全取末行复制（`win[0]` 应为行 H-2） | `win[0]` 改取 `w_row_p2` |
| N-24 | `out_row` 帧末递增为 IMG_H 后不回卷，第二帧起 tuser 帧首标记丢失 | 末帧像素发射后计数器回卷归零 |

#### CORDIC 延迟对齐（F2 修复）

CORDIC IP 的流水线延迟（`CORDIC_IP_LATENCY`，默认 21）必须与幅值路径的延迟线严格对齐：

```
幅值路径延迟 = 1(mag2_r) + CORDIC_LAT(延迟线) = CORDIC_IP_LATENCY
故 CORDIC_LAT = CORDIC_IP_LATENCY - 1
```

- 编译期断言：`CORDIC_IP_LATENCY >= 2`
- 运行期自检：`cordic_out_valid !== vld_dly[CORDIC_LAT-1]` → `$error`（一旦 IP 延迟不匹配即报错，避免整条匹配链静默失效）

#### 输出

| 信号 | 位宽 | 含义 |
|---|---|---|
| `m_axis_mag2` | 22bit | 梯度幅值平方 `dx²+dy²` |
| `m_axis_angle` | 16bit | CORDIC 归一化相位（`±π` 全圆，`out = θ/π × 2^15`） |

输出中心对齐：丢弃第 0 行与每行第 0 列，每帧输出 `IMG_W × IMG_H` 个有效像素。

---

### 5.3 SBM.Alg.3 梯度量化（sbm_alg3_quantize）

#### 算法（与 line2Dup.cpp `hysteresisGradient` 逐像素对齐）

1. **桶号计算**：`q16 = (angle + 2048) >> 12`，16 桶；`label = q16 & 7`，合并 8 方向。
2. **强梯度门控**：`strong = (mag2 > 900)`（`weak_threshold² = 900`）。
3. **边界清零**：边界像素 label 强制 0。
4. **3×3 投票**：在 3×3 窗口内对 8 个方向做直方图统计（9 个标签单热展开相加）。
5. **最大票方向**：链式严格大于比较，平票取小索引。
6. **双条件门控输出**：`strong && !border && (best_votes >= 5)` → 输出 `8'b1 << best_dir`（单热编码），否则输出 0。

#### 中心对齐

投票窗口由因果 `[r-2..r] × [c-2..c]` 改为与 C++ 一致的中心窗口（输出像素 = 窗口中心 `(r-1, c-1)`），丢弃第 0 行与每行第 0 列。

#### 幅值门控对齐（F3 → N-27 重写）

**N-27（当前实现）**：行缓冲携带 4bit `{strong, label}`，消费拍当拍 lb 读出口即中心像素 `(r-1, c-1)` 的 strong；经 `sc_d` 4 级移位链寄存至与 `best_dir_r/best_votes_r` 同沿（消费拍 +4），级5 门控采样 `sc_d[3]`。

原 F3 方案（`strong_s1→strong_s5` 随数据路径同拍传递 + `STRONG_DLY=3` 固定深度移位链）实际门控的是**消费像素 `(r,c)`** 的 strong——比中心像素多 1 行 1 列，块边界 ±1 行/列强弱反转（C-golden TB3 捕获：块边缘对角错 577 处）。固定深度移位链无法实现跨行延迟，已移除（`STRONG_DLY` 参数标废弃保留接口兼容，F3 自检一并移除）。

#### 输出

8bit 单热编码（`1<<dir` 或 0），边界像素输出 0。

#### N-8 闭环新增修正（⑥–⑪，C-golden TB3 逐项捕获）

| 编号 | 缺陷 | 修法 |
|---|---|---|
| N-14 | 帧首拍被 `row_active=0` 门控丢弃，整帧左移 1 列，量化方向整体错列 | `w_in_valid` 补入 `s_axis_tuser` 项 |
| N-19 | 行尾补拍软契约依赖 | 补拍期间拉低 `s_axis_tready`（照搬 alg8 P2） |
| N-24 | `out_row` 帧间不回卷，第二帧 tuser 丢失 | 末帧像素发射后回卷归零 |
| N-25 | `flush_row` 清除沿与补行换行拍（`c=IMG_W`）同拍重叠，末行末列输出永缺 | 补拍（`flush_c≠0`）期间不清除，延迟一拍 |
| N-26 | 坐标标签链与数据路径错位 2 拍（数据 5 级 vs 标签 3 级）：c=0 拍不再丢弃、帧尾多发 H-1 拍、背景区方向错 | 标签链补 2 级延迟（`s_row_w3/w4`）对齐 `vld_out`；补注册 `best_votes_r`（原版级5 直用级3 组合 votes） |
| N-27 | strong 门源错位 1 行 1 列（门控消费像素而非中心像素） | 行缓冲 3bit→4bit 携带 strong，消费拍当拍 lb 读出口 + `sc_d` 4 级链（见上节） |

流水线时序基准：消费拍 c → 级1寄存(c+1) → 级2窗口(c+2) → 级3投票(c+3) → 级4最优(c+4) → 级5输出(c+5)；帧内每帧窗口拍 = `(IMG_H+1)×(IMG_W+1)`，丢弃第 0 行与每行第 0 列后每帧发射 `IMG_W×IMG_H` 拍。

---

### 5.4 SBM.Alg.8 扩散 OR 运算（sbm_alg8_spread）

#### 算法（与 line2Dup.cpp `spread()` 逐像素一致）

```
dst(x,y) = OR_{r,c ∈ [0,T)} src(x+c, y+r)    // 越界按 0（右缘/下缘零填充）
```

#### 实现

采用因果（后向）T×T OR + 输出丢弃前缀：

- **水平级**：T 级移位寄存器 + 前缀 OR 树，行尾补零 T-1 拍。
- **垂直级**：T-1 个行缓冲（按行号 mod (T-1) 轮转）+ 因果 OR 树，底部补零 T-1 行。
- **输出对齐**：丢弃每行前 T-1 列、每帧前 T-1 行（利用按位或交换律，因果 OR 在 `(p,q)` = 前向窗口在 `(p-T+1, q-T+1)`）。

#### 输出背压（P0-3 修复）

`m_axis_tready` 接入全流水 stall：下游（alg9）拉低 tready 时输入反压 + 全部计数器冻结，恢复后从断点续传，不再静默丢像素。`out_row` 每帧回绕，修复 `tuser` 仅复位后首帧有效的缺陷。

#### 帧间约束

两帧之间需预留 ≥ T-1 行空闲供底部补零排空，由 `s_axis_tready` 硬件强制（`flush_c != 0` 时反压）。

---

### 5.5 SBM.Alg.9 响应图 LUT 查表与线性化落盘（sbm_alg9_lut）

> **本模块是整个管线的核心**，完成响应图查表、转置、线性化和 DDR 落盘。

#### 算法（与 line2Dup.cpp `computeResponseMaps()` + `linearize()` 语义一致）

```
查表:  resp_ori(x,y) = max(SIM[ori*32 + byte&15], SIM[ori*32 + 16 + byte>>4])
落盘:  addr = RESP_BASE + (ori*T*T + block)*CELLS + cell
       其中 block = (y%T)*T + x%T, cell = (y/T)*WC + x/T
```

#### 实现架构

```
     ┌──────────────┐     ┌─────────────────┐     ┌──────────────────┐
输入 │ 16路并行     │     │ T行角转         │     │ 8方向×SEG_BUFS   │
流 ─▶│ LUTRAM 查表  │────▶│ ping-pong       │────▶│ 段缓冲           │──▶ AXI4-MM
     │ (8方向同时)  │     │ 双 bank SDPRAM  │     │ (旋转槽号)       │   64bit 写
     └──────────────┘     └─────────────────┘     └──────────────────┘
                                                           │
                                                    ┌──────┴──────┐
                                                    │ 写请求 FIFO  │
                                                    │ {slot,ori,  │
                                                    │  block,band}│
                                                    └─────────────┘
                                                           │
                                                    ┌──────┴──────┐
                                                    │ AXI4-MM     │
                                                    │ 写主机 FSM  │
                                                    │ (AW/W/B握手)│
                                                    └─────────────┘
```

#### 各级详解

1. **查表级**：8 个方向各做低/高半字节 2 次查表（共 16 路并行 LUTRAM 读）。`SIMILARITY_LUT`（256 行 8bit）**源自 `line2Dup.cpp` 的 `SIMILARITY_LUT[256]`（L737）**，转储到 `similarity_lut.mem`，通过 `$readmemh` 装载（注：并非由 `golden_sbm.c` 生成，见 §11 D1）。8 个方向同时查表，输出 64bit（8 方向 × 8bit）。

2. **双 bank 转置**：两个 64bit SDPRAM（ping-pong）。写侧按行顺序写入当前 band，读侧按块序 `(gy, gx, cx)` 扫描读出，实现 T×T 块内转置。`band_filled` 标志 + 独立 drain FSM 控制写满即读。

3. **段缓冲**：8 方向 × `SEG_BUFS`（默认 4）槽旋转缓冲。每段 `SEG_LEN = WC` 字节，段满后压入写请求 FIFO。高水位节流（`req_cnt >= 8` 时暂停 drain）保证段槽不被覆盖。

4. **AXI4-MM 写主机**：单 outstanding，`SEG_BEATS = ceil(WC/8)` 拍突发/请求。末拍 `wstrb` 部分选通（WC 非 8 整数倍时）。请求地址由 `{ori, block, band}` 组合计算。

#### 关键修正

| 修正项 | 说明 |
|---|---|
| F1 | SIMILARITY_LUT 改由 `$readmemh("similarity_lut.mem")` 装载（原版内联手填表方向 2/3 值错误、方向 4–7 全 0）。⚠️ 数据须源自 `line2Dup.cpp` L737；2026-08-15 复审发现该 `.mem` 一度为线性斜坡占位（详见 §11 / D1，已修复重生成） |
| F4 | req_fifo 写入/读出位域严格对齐；req_cnt 合并计数；FIFO 深度参数化 |
| F5a | 全部几何量与位宽由 `sbm_geometry.vh` 派生，修复 5 个"换相机即崩坏"的硬编码缺陷 |
| P0-1 | 帧首拍强制写 bank0，修掉多帧连续输入时 tuser 拍残留旧坐标的错位 |
| P0-2 | 几何 16T 对齐保证 WC 被 8 整除 → AXI 8B 地址对齐 |

#### 接口

```
AXI4-Stream 输入: s_axis_tvalid/tready/tdata[7:0]/tuser/tlast
AXI4-MM 写主机:   m_axi_aw*/m_axi_w*/m_axi_b*
中断:             irq_done（响应图落盘完成）
```

---

### 5.6 SBM.Alg.11 相似度滑窗累加（sbm_alg11_accum）

#### 算法（与 line2Dup.cpp `similarity()` 语义一致）

```
for each feature f:
    for j in [0, template_positions):
        score[j] += lm_ori[ block(f)*CELLS + lm_index(f) + j ]
其中 ori = f.label, block(f) = (f.y%T)*T + (f.x%T), lm_index(f) = (f.y/T)*WC + (f.x/T)
方向偏移: lm_ori 基址 = RESP_BASE + ori*T*T*CELLS
```

#### 实现架构

```
                    ┌─────────────────────────────────────────┐
   AXI4-Lite ──────▶│ 特征 FIFO (异步, s_axi_aclk → clk)       │
   (PS 写特征)       └───────────────┬─────────────────────────┘
                                   │ feat_rd
                                   ▼
                    ┌─────────────────────────────────────────┐
                    │ 顶层 FSM                                │
                    │ ST_IDLE → ST_ZERO → ST_FEAT → ST_FEAT2  │
                    │   → ST_RUN → ST_SCAN → ST_PUSH → ST_DONE│
                    └──┬──────────────────────────────┬───────┘
                       │ init/base/tp/thresh          │ scan_en
                       ▼                              ▼
  ┌────────────────────────────────────┐  ┌────────────────────┐
  │ LANES(24) × sbm_accum_lane         │  │ Top-32 归并         │
  │ 双 BRAM 乒乓 RMW 累加              │  │ 停留式轮询各 lane    │
  │ 分块位置区间 [l*CHUNK, l*CHUNK+CHUNK)│  │ 命中 FIFO → t32[32] │
  └──────────────┬─────────────────────┘  └─────────┬──────────┘
                 │ need_req/req_addr                │ cand_wr
                 ▼                                  ▼
  ┌──────────────────────────┐     ┌─────────────────────────┐
  │ 读仲裁(轮询) + 256bit    │     │ 候选 FIFO (异步,         │
  │ AXI4 读主机(单拍32B突发)  │     │ clk → s_axi_aclk)        │
  │ ← DDR 响应图              │     │ → AXI4-Lite → PS         │
  └──────────────────────────┘     └─────────────────────────┘
```

#### 顶层 FSM 状态机

| 状态 | 功能 |
|---|---|
| `ST_IDLE` | 等待 `start_pulse`（PS 写 0x00 寄存器触发） |
| `ST_ZERO` | 清零所有 lane 的双 BRAM（按 `BANK_DEPTH` 地址遍历写 0） |
| `ST_FEAT` | 从特征 FIFO 弹出一个特征，计算 `base_addr`（含方向偏移 `ori*T*T*CELLS`） |
| `ST_FEAT2` | 1 拍流水寄存，做越界判定（`x < IMG_W && y < IMG_H`） |
| `ST_RUN` | 所有 lane 并行累加该特征贡献；全部 `lane_done` 且出队弹空后回 `ST_FEAT` |
| `ST_SCAN` | 各 lane 自扫全部位置，命中（`score > thresh`）压入命中 FIFO；顶层并发归并 Top-32 |
| `ST_PUSH` | 将 Top-32 候选写入候选 FIFO（`{score, y, x}` 32bit 打包） |
| `ST_DONE` | 置 `irq_done`，回 `ST_IDLE` |

#### 单 lane 引擎（sbm_accum_lane.v）

- **双 BRAM 乒乓 RMW**：偶数位置存 `bank_e`，奇数位置存 `bank_o`。读延迟 1 拍（`read_first`），本拍读旧值 + 下拍写回 `dout + byte_d`。
- **关键修正**：读地址 `w_addr_r = j_local>>1`（预取下一位置），写地址 `w_addr_w = (j_local-1)>>1`（当前位置），二者相邻两拍数值恒等但落于不同 bank，消除"奇数位置差一"致命 bug。
- **双缓冲消费**：256bit AXI4 读数据双槽轮转，`fill_vld` 回填后 `consume` 逐字节消费。
- **候选扫描**：1 位置/拍自扫，`scan_done` 在末位置完成判定拍置位（与末位置命中压 FIFO 同拍，`hcnt` 同步更新）。
- **命中 FIFO**：参数化深度（`HIT_FIFO_DEPTH`，默认 64），背压（`scan_stall` 在近满时暂停扫描），并发读写不丢。
- **F6 修复**：扫描与归并并发——各 lane 边扫边把命中压入 FIFO，顶层边弹出边归并 Top-32。背压 `scan_stall` 在 FIFO 近满时暂停对应 lane，任意命中密度不溢出。

#### AXI4-Lite 寄存器映射

| 偏移 | 读 | 写 | 说明 |
|---|---|---|---|
| 0x00 | status/done/tperr | start（bit[0]=1 触发） | 状态/启动 |
| 0x08 | — | nfeat | 特征数 |
| 0x0C | — | feature（32bit） | 特征 FIFO 写入 |
| 0x10 | — | tp | template_positions |
| 0x14 | — | thr | 归一化阈值（0–100） |
| 0x18 | — | resp | 响应图基址 |
| 0x1C | — | wc | 级1每行单元数 |
| 0x20 | cand_cnt | — | 候选条数 |
| 0x24 | cand（读即弹） | — | 候选 FIFO 读出 |
| 0x28 | max_score | — | 本模板最高得分 |

#### 阈值口径

```
raw_thr = floor(thr * 4 * nfeat / 100)    // 与 C++ 严格大于口径一致
命中条件: score > raw_thr（严格大于）
```

---

### 5.7 SBM.Alg.13 NMS 后处理（sbm_alg13_nms.c）

> **PS 端 C 语言模块**，运行在 ARM A53 上。

#### 处理流程

```
PL Top-32 候选 (AXI4-Lite)
       │
       ▼
  ① 归一化: score = raw * 100 / (4 * nfeat)
       │ 低分过滤 (score >= MIN_SCORE=20)
       ▼
  ② 按得分降序插入排序 + IoU NMS (阈值 0.5)
       │ 最多 TOP_K=8 进细化
       ▼
  ③ 级 0 细化: 坐标链换算 + 16×16 单元窗口重算得分
       │
       ▼
  ④ 排序输出 Top-N (TOP_N=5)
```

#### 坐标换算链（与 C++ `matchClass` 完全一致）

```
级1单元 (cx,cy)
  → 级1像素: c*T1 + OFFSET1     (T1=8, OFFSET1=3)
  → 级0像素: ×2 + 1
  → 级0单元: /T0                (T0=4)
  → 16×16 单元细化窗口
  → 原图像素: (cx-8+bdx)*T0 + OFFSET0   (OFFSET0=1)
```

#### 级 0 线性内存寻址（与 4.2.6 节 `linearize` 同构）

```c
block = (y % T0) * T0 + (x % T0);
cell  = (y / T0) * wc0 + (x / T0);
ptr   = base + (ori * T0*T0 + block) * wc0*wc0 + cell;
```

#### IoU 计算

候选框 = 模板在级 1 的跨度（`w1 × h1` 单元），在级 1 单元坐标系计算交并比。

#### 硬件接口

通过裸机寄存器寻址（`0xA0000000` 基址）读取 PL 候选 FIFO：
- `REG_CAND_CNT`（0x20）：候选条数
- `REG_CAND_RD`（0x24）：读即弹出
- `REG_MAX_SCORE`（0x28）：本模板最高得分

---

## 6. 支撑模块

### 6.1 CORDIC 行为模型（cordic_atan2_beh.v / cordic_atan2_func.vh）

- **`cordic_atan2_beh.v`**：Xilinx CORDIC IP 的行为级仿真替代，参数化延迟 `LATENCY`（默认 21）。valid + data 同步流水，总延迟 = `LATENCY`。
- **`cordic_atan2_func.vh`**：共享的向量模式 CORDIC `atan2` 参考函数，返回 16bit 有符号角度分数（`θ/π × 2^15`）。被行为模型和 testbench 共同 `include`，保证两者数值完全一致。
- **用途**：联仿时验证 F2 对齐保护（可用正确/错误延迟分别跑 PASS/FAIL）；绝对精度是 IP 厂商职责，不在联仿范围。
- **综合时**：替换为真实 Xilinx CORDIC IP（`cordic_atan2`），配置 ArcTan / 12bit 有符号输入 / 16bit 输出 / 全流水。

### 6.2 XPM 行为模型

| 文件 | 替代原语 | 语义 | 使用模块 |
|---|---|---|---|
| `xpm_memory_sdpram_beh.v` | `xpm_memory_sdpram` | 简单双口 RAM，`read_first` / 延迟 1 拍 | alg1(v), alg2, alg3, alg8, alg9 |
| `xpm_memory_spram_beh.v` | `xpm_memory_spram` | 单口 RAM，`read_first` / 延迟 1 拍 | alg11_accum_lane |
| `xpm_fifo_async_beh.v` | `xpm_fifo_async` | 异步 FIFO，distributed / fwft | alg11（特征 FIFO + 候选 FIFO） |

> 这三个文件**仅用于 iverilog 联仿**，综合时使用真实 Xilinx XPM 原语（RTL 中的例化代码不变）。

---

## 7. 黄金参考模型与测试平台

### 7.1 黄金参考模型（golden_sbm.c）

C 语言参考实现，生成三个联仿数据文件：
- `stimulus.hex`：逐位置特征字节（hex 格式）
- `expected_pos.txt`：全位置参考（`pos score x y`）
- `hits.txt`：期望命中位置（`pos score x y`）

> **注**：`similarity_lut.mem`（256 行 8bit，供 alg9 `$readmemh` 装载）的**真实来源是 `line2Dup.cpp` 的 `SIMILARITY_LUT[256]`（L737）**，并非由 `golden_sbm.c` 转储（早期文档表述有误，见 §11 D1）。

### 7.2 测试平台

| Testbench | 验证目标 | 黄金比对 |
|---|---|---|
| `tb_sbm_alg1_gaussian.v` | 高斯滤波输出（N-8 闭环重写，V-2 关闭） | C-golden 两级高斯 [2,7,14,18,14,7,2]/64 逐行逐字节比对 + tuser/tlast 协议校验，LFSR/随机非退化激励，双帧（PASS 32768 pixels） |
| `tb_sbm_alg2_sobel.v` | Sobel+CORDIC 全链（N-8 闭环新增，V-3 关闭） | C-golden dx/dy/mag2/angle 逐拍比对（ramp 基底 + 中心亮方块 + LCG 噪声非退化激励），双帧协议校验（PASS 32768 pixels） |
| `tb_sbm_alg3_quantize.v` | 量化投票全链（N-8 闭环新增，V-5 关闭） | C-golden 单热输出逐拍比对；分区激励覆盖投票边界（棋盘格 5/4 票）、强弱阈值（mag2=900/901 交替）、桶回绕（桶13/5）、首末行列清零，双帧（PASS 32768 pixels） |
| `tb_sbm_alg2_cosim.v` | Sobel + CORDIC 输出（历史对齐验证，功能比对已由上方 TB 取代） | mag2 + angle 逐拍对齐 |
| `tb_sbm_alg3_cosim.v` | 量化单热输出（历史对齐验证，功能比对已由上方 TB 取代） | 逐像素对齐 |
| `tb_sbm_alg8_spread.v` | 扩散 OR + 背压 + 逐帧 tuser | 黄金比对 |
| `tb_sbm_alg9_cosim.v` | LUT 落盘布局（AW/W/B 握手） | DDR 地址 + 数据 x 值检测 |
| `tb_sbm_alg11_accum.v` | 端到端累加 + Top-32 | 黄金模型（越界过滤 + top-32 排序） |
| `tb_sbm_accum_lane_cosim.v` | 单 lane 扫描坐标/得分对齐 | 7 FIFO hits 匹配 |
| `tb_sbm_geom_check.v` | 多分辨率几何自检 | 全链路派生量断言 |

### 7.3 联仿要点

- **alg9**：TB 用 16bit LFSR 非退化图样（旧图样在地址错位场景下退化巧合造成假 PASS）；显式检测 `m_axi_awaddr`/`m_axi_wdata` 的 x 值（旧 TB 用 integer 承接地址得 unknown → 假 PASS）。
- **alg11**：3 个 case——常规命中（thr=40）、全命中背压压力（thr=0）、8 特征 top-32 淘汰（满表淘汰语义）。
- **alg1/2/3（N-8 闭环）**：三个 C-golden 功能 TB 均为双帧发送（帧间 ≥4×IMG_W 拍间隙、行间 3 拍间隙，验证任意上游节奏下硬件强制补拍正确）；逐拍比对 tdata/tuser/tlast 与期望坐标推进，任意像素或标记不符即计 err。验收实证：N-8 类协议级缺陷（行门控错误、补零拍被吞、帧间状态残留）在修复前均被逐行逐字节比对捕获报 FAIL（累计驱动修复 20+ 真实缺陷，见 §12）。

---

## 8. 关键数据流与地址布局

### 8.1 端到端数据流

```
stimulus.hex ─▶ [alg1 高斯]─[alg2 Sobel+CORDIC]─[alg3 量化]
                                                          │ 降采样 ×2
                                                          ▼
                              [alg8 T×T OR 扩散]
                                                          │
                similarity_lut.mem ─▶ [alg9 LUT+转置+AXI落盘] ─▶ DDR
                                                          │ 响应图线性内存
                                                          ▼
                              [alg11 多lane累加 + Top-32] ◀── AXI4 读 DDR
                                                          │
                              golden_sbm.c(真值) ◀── tb_*_cosim.v(比对)
                                                          │
                              [alg13 NMS(PS)] ◀── AXI4-Lite ── 匹配结果
```

### 8.2 DDR 响应图线性内存布局

```
RESP_BASE (= 0x0800_0000)
  │
  ├── ori=0: [T*T 个 block × CELLS 字节]
  │     block=0:  cell 0, cell 1, ..., cell CELLS-1
  │     block=1:  cell 0, cell 1, ..., cell CELLS-1
  │     ...
  │     block=T*T-1: ...
  │
  ├── ori=1: ...
  ├── ...
  └── ori=7: ...

总大小 = 8 × T*T × CELLS 字节
       = 8 × 64 × 102400 = 52,428,800 字节 ≈ 50 MB（5000×5000 相机）
```

**地址公式**：
```
addr = RESP_BASE + (ori*T*T + block)*CELLS + cell
其中 block = (y%T)*T + x%T, cell = (y/T)*WC + x/T
```

> alg9 的落盘地址布局须与 alg11 的读回公式严格一致，否则累加读错位置导致整条匹配链静默失效。

---

## 9. 构建与联仿

### 9.1 环境依赖

- `iverilog` / `vvp`（icarus-verilog，`brew install icarus-verilog`）
- `similarity_lut.mem` 须与 `tb_*.v` 同目录（alg9 `$readmemh` 装载）
- XPM 行为模型：`xpm_memory_spram_beh.v`、`xpm_memory_sdpram_beh.v`、`xpm_fifo_async_beh.v`

### 9.2 快速开始

```bash
make all              # 跑全部联仿: F1 / F4 / F5a / F6+F9 / P0-1 / P0-3 / 几何自检 / N-8 前端闭环
make run_fix          # F1: lane 候选扫描(7 FIFO hits matched)
make run_alg9         # F4: alg9 落盘布局(64x64)
make run_alg9_multi   # F5a: alg9 多分辨率(128x64 / 64x40 / 512x512)
make run_alg9_cont    # P0-1: alg9 连续两帧
make run_alg8         # P0-3: alg8 背压 + 逐帧 tuser
make run_alg11        # F6+F9: alg11 端到端黄金比对(3 case)
make run_geom         # 几何单一真源自检(默认 5000x5000)
make run_alg1         # N-8 闭环: alg1 两级高斯 C-golden TB
make run_alg2         # N-8 闭环: alg2 Sobel+CORDIC C-golden TB
make run_alg3         # N-8 闭环: alg3 量化投票 C-golden TB
```

预期输出均为 `RESULT: PASS` / `GEOM RESULT: PASS` / `PASS: 32768 pixels ...`。前端三个 TB 的 PASS/FAIL 由 Makefile 层 `grep '^PASS'` 校验（TB 无 `$fatal`，无 PASS 行即 make 非零退出）；仿真日志留存于 `sim_alg1/2/3.log`（`make clean` 一并清理）。

### 9.3 换相机分辨率

只改 `sbm_geometry.vh` 顶部 4 个宏（`SBM_CAM_W` / `SBM_CAM_H` / `SBM_T` / `SBM_LANES`），级0 对齐、级1 尺寸、WC/HC/CELLS、通道容量、全部位宽自动派生。

```bash
# 用 -D 覆盖宏模拟换相机后跑几何自检
iverilog -g2012 -DSBM_CAM_W=1920 -DSBM_CAM_H=1080 -o /tmp/g.out tb_sbm_geom_check.v
vvp /tmp/g.out
```

### 9.4 IP 打包

`package_gauss.tcl` 将高斯模块打包为 Vivado IP：
```tcl
create_project -in_memory -part xczu3eg-sfvc784-1-e
add_files {sbm_gauss_h.v sbm_gauss_v.v sbm_alg1_gaussian.v}
ipx::package_project -root_dir ./ip_repo -vendor muteng -library mvtm ...
```

---

## 10. 当前状态与审查记录

| 审查项 | 状态 | 说明 |
|---|---|---|
| F1（SIMILARITY_LUT 装载 / lane 扫描对齐） | ✅ 已修复并验证 PASS | LUT 改 `$readmemh` 装载；lane 候选扫描坐标/得分对齐 |
| F2（CORDIC 延迟对齐） | ✅ 已修复并验证 PASS | 延迟线深度由 `CORDIC_IP_LATENCY` 派生 + 运行期对齐自检 |
| F3（幅值门控对齐） | ✅ 已重写（N-27，2026-08-18） | 原方案门控消费像素而非中心像素（错位 1 行 1 列）；已改为行缓冲回读中心 strong + `sc_d` 链，C-golden TB3 验证 PASS |
| F4（alg9 写请求 FIFO / AXI 落盘布局） | ✅ 已修复并验证 PASS | 64x64/120x64/64x40/512x512 多分辨率 |
| F5（参数一致性 / 单一真源） | ✅ 已修复 | 几何量全部由 `sbm_geometry.vh` 派生 + 多分辨率自检 |
| F5a（参数可维护性） | ✅ 已修复 | 见 §4；5 个"换相机即崩坏"硬编码缺陷已修复 |
| F6（alg11 命中 FIFO 溢出 / 并发 drain） | ✅ 已修复并验证 PASS | 含全命中背压压力 case |
| F7（mag2 位宽） | ✅ 已修复并验证 PASS | 21bit → 22bit |
| F9（alg11 端到端黄金比对） | ✅ 已重写并通过 | 黄金模型含越界特征过滤 + top-32 淘汰比对 |
| P0-1（alg9 连续帧） | ✅ 已修复并验证 PASS | 帧首 bank0 竞争修复 |
| P0-2（AXI 8B 对齐） | ✅ 已修复 | 几何 16T 对齐 + 首拍 awaddr 断言 |
| P0-3（alg8 背压） | ✅ 已修复并验证 PASS | 下游反压 + 逐帧 tuser + 黄金比对 |
| **D1（similarity_lut.mem 内容）** | ✅ **已修复（2026-08-15）** | 该 `.mem` 曾为线性斜坡占位而非真实 LUT，导致响应值全错（"假 PASS"）；已从 `line2Dup.cpp` L737 重生成，联仿复跑 PASS。详见 §11 |
| **D2（累加器位宽）** | ⚠️ **待修复（高风险）** | lane 累加器 8-bit 无饱和，与 C++ `CV_16U` 不符；`FEAT_MAX=64` 强匹配时 256 回绕归零 → 漏检。详见 §11 |
| **N-1（alg9 seg_buf 寄存器阵列）** | ⚠️ **待整改（P0 资源风险，2026-08-15 复审新增）** | `seg_buf` ≈ 81,920 FF（约 XCZU3EG FF 的 58%），为 FF 绑定约束，须 RAM 化。详见 §11.8 |
| **V-2（tb_alg1 恒真空壳）** | ✅ **已关闭（2026-08-18）** | 重写为 C-golden 两级高斯逐行逐字节比对 TB（`tb_sbm_alg1_gaussian.v`）并接入 `make all`，PASS 32768 pixels。详见 §12 |
| **V-3/V-5（alg2/alg3 TB 覆盖）** | ✅ **已关闭（2026-08-18）** | 新建 `tb_sbm_alg2_sobel.v` / `tb_sbm_alg3_quantize.v` C-golden 功能比对并接入 `make all`，各 PASS 32768 pixels；过程中累计捕获并修复 N-14→N-27 系列缺陷。详见 §12 |
| **V-4（CORDIC 真实 IP 延迟）** | ⚠️ 待确认（P1） | 行为模型延迟 21 拍为假设值，真实 IP Latency 需在 Vivado 中核对 |
| **N-3/N-4/N-6/N-7/N-2（次级项）** | ⚠️ 待处理（P1–P3） | cand_fifo 复位跨域、start 脉冲同步余量为 0、alg13 方形假设与 `>=` 口径、L98 无效断言。详见 §11.8.5 |
| **N-8（前端 AXI-S 帧协议级联缺陷）** | ✅ **已闭环（2026-08-18）** | alg1 行门控丢像素（N-8a）+ gauss_h 冲刷软契约（N-8b）+ 全链协议统一；C-golden TB 闭环中累计捕获并修复 N-9/N-14/N-19/N-20/N-22→N-27 共 12 项实体缺陷。详见 §12 |
| **N-9（alg1 行首脉冲宽度）** | ✅ 已修复（TB1 逐拍探针定位） | 行首标记组合判据产生 2 拍宽脉冲，改寄存单拍脉冲 |
| **N-14/N-19/N-22/N-23/N-24（alg2 协议/时序缺陷）** | ✅ 已修复（C-golden TB2 逐项捕获） | 见 §5.2 修正表 |
| **N-14/N-19/N-24/N-25/N-26/N-27（alg3 协议/时序缺陷）** | ✅ 已修复（C-golden TB3 逐项捕获） | 见 §5.3 修正表 |

---

## 11. 算法正确性与落地可行性评审（2026-08-15）

> 评审依据：①《基于几何形状的图像匹配技术预研报告》 ②参考论文 *Gradient Response Maps for Real-Time Detection of Textureless Objects*（Hinterstoisser et al., TPAMI 2012） ③ `line2Dup.h` / `line2Dup.cpp` 参考实现
> 方法：逐模块"论文 ↔ C++ ↔ RTL"算法口径比对、接口/数据流一致性核查、联仿复跑（iverilog/vvp）、资源/时序/功耗/可综合性评估。

### 11.1 总体结论

| 维度 | 结论 |
|---|---|
| 算法逻辑正确性 | ✅ 前端（高斯/Sobel/量化）、扩散、LUT 查表+线性化落盘、24 通道累加、NMS 的**算法逻辑**与论文及 `line2Dup.cpp` 语义一致；几何"单一真源"设计良好 |
| 数据/精度类缺陷 | ⚠️ **D1：`similarity_lut.mem` 误用线性斜坡（致命，已修复）**；**D2：累加器 8-bit 与 C++ 16-bit 不符（高风险，待修复）** |
| 接口/数据流一致性 | ✅ 模块间位宽衔接一致；alg2→alg3→alg8→alg9→DDR→alg11→alg13 链路自洽；无覆盖缺口 |
| 工程落地可行性 | ⚠️ 资源（BRAM ~66%，复审修正）可行但偏紧；**seg_buf ≈81.9k FF 为绑定约束（N-1，须 RAM 化）**；时序/功耗需 Vivado/XPE 实测；**500 模板@10fps 仅中小模板可行**（吞吐量风险） |
| 验证完备性 | ⚠️ 后端（alg8/9/11）联仿坚实；**前端存在系统性空洞：tb_alg1 恒真空壳（V-2）、alg2/alg3 TB 仅对齐验证且未接入回归（V-3/V-5）、CORDIC 真实 IP 延迟未验证（V-4）**；无系统级集成测试台 |

**核心判断**：代码在**算法结构**上准确实现了论文的梯度响应图与无纹理目标检测框架，且相对参考 C++ 的映射基本正确；但存在**两个会直接破坏功能正确性的数据/精度缺陷**——D1 已修复、D2 必须修复后方能在"特征数接近上限"的模板上保证召回。第二轮复审（§11.8）确认：**除 D2 外，数据链路与参考 C++ 语义一致**；另识别 N-1（seg_buf FF）与前端验证空洞两项 P0 级问题，与 D2 并列为放行前置项。

### 11.2 算法正确性（论文 ↔ C++ ↔ RTL）

**与论文一致性（关键论据）**：

- **T 与方向数**：论文原文 *"In practice, we use T = 8 and n₀ = 8"* —— RTL `SBM_T=8`、`SIM[gi*32+..]` 8 方向，完全吻合。
- **扩散**：论文 3.3 节，在 `[-T/2, +T/2]²` 邻域**按位 OR**（"merging all shifted versions with an OR operation"）—— RTL `sbm_alg8_spread` 实现 T×T 因果 OR，吻合。
- **响应图 LUT**：论文 3.4 节，用"扩散二进制串 J"作索引查表预计算最大相似度 —— RTL `SIMILARITY_LUT[256]` + `resp[gi]=max(SIM[gi*32+lsb], SIM[gi*32+16+msb])` 完全对应。
- **线性化存储**：论文 3.5 节，`T²·n₀` 个线性内存、每 T 像素取数 —— RTL `block=(y%T)*T+x%T, cell=(y/T)*WC+x/T` 对应。
- **量化**：论文"omit the gradient direction"（忽略梯度方向）—— RTL `q16=(angle+2048)>>12; label=q16&7` 等价于把 16 桶/2π 折叠为 8 桶/π，**即无向量化**，与论文意图一致。

**与 `line2Dup.cpp` 一致性（逐算子）**：

| 算子 | C++ 口径 | RTL 实现 | 结论 |
|---|---|---|---|
| 量化 | `q16=(angle+2048)>>12`, `label=q16&7`, `strong=mag2>900`, 3×3 投票 `best_votes>=5`, 边界清零 | `sbm_alg3_quantize` 同口径；输出 `8'b1<<best_dir_r` 单热 | ✅ |
| 扩散 | `spread()` T×T OR | `sbm_alg8_spread` 因果 OR + 丢前 T-1 行列 | ✅ |
| LUT 查表 | `max(lut_low[lsb], lut_low[msb+16])`, `lut_low=SIMILARITY_LUT+32*ori` | `max(SIM[gi*32+lsb], SIM[gi*32+16+msb])` | ✅ |
| 线性化落盘 | `linearize` cell/block 序 | 写 `w_wraddr=(eff_row&T-1)*IMG_W+eff_col`；读 `w_rdaddr=rd_gy*IMG_W+rd_cx*T+rd_gx`（双 bank 转置），与写互为逆运算 | ✅ |
| 累加基址 | `base + block*CELLS + lm`, `lm=feature.y*WC+feature.x` | `w_base=cfg_resp+({feat[26:24],block}*CELLS)+{feat[23:15]*cfg_wc_c+feat[11:3]}` | ✅ |
| NMS | `score=raw*100/(4*nf)`, `MIN_SCORE=20`, IoU 0.5, 坐标链 `c*T1+OFFSET1→×2+1→/T0→16×16 细化→(cx-8+bdx)*T0+OFFSET0` | `sbm_alg13_nms.c` 同口径 | ✅ |

### 11.3 关键缺陷（D1/D2）

#### D1 〔已修复〕`similarity_lut.mem` 误用线性斜坡 —— 致命数据缺陷

- **现象**：文件内容曾是线性斜坡 `00,01,02,…,FF`，而非 `line2Dup.cpp` L737 的真实 `SIMILARITY_LUT`（真实值开头为 `00 04 03 04 …`）。
- **影响**：`sbm_alg9_lut` 产生的 8 张响应图数值**全部错误**（查表退化为 `max(低半字节, 高半字节)` 而非相似度），下游 `alg11/alg13` 匹配失真。
- **为何联仿仍 PASS（假 PASS）**：`tb_sbm_alg9_cosim.v` 与 DUT 装载**同一个** `.mem`，golden `exv` 用相同斜坡反推 —— 只验证了**地址布局/乒乓/AXI 协议**，未验证查表**内容**。这正是自洽联仿的盲区。
- **根因**：早期文档称"由 `golden_sbm.c` 转储"，但 `golden_sbm.c` **并不生成**该文件（仅产出 stimulus/expected/hits）；实为占位斜坡，前序 F1 修复只加了 `$readmemh` 装载机制而未填充真实数据。
- **修复（已执行）**：从 `line2Dup.cpp` L737 提取真实 256 项，重生成 `similarity_lut.mem`（原斜坡备份为 `similarity_lut.mem.ramp_bak`）。复跑 alg9 全部分辨率联仿：**`tot=32768/65536/20480/2097152, err=0, oob=0, RESULT: PASS`**。

#### D2 〔待修复〕累加器 8-bit 位宽与 C++ 16-bit 不符 —— 高风险精度/正确性问题

- **现象**：`sbm_accum_lane.v` 双 BRAM 为 8-bit（`MEMORY_SIZE(BANK_DEPTH*8)`、`WRITE_DATA_WIDTH_A(8)`），RMW 为 8-bit 加法 `w_din_e = dout_e + byte_d`，**无饱和**；而 C++ `similarity()` 用 `CV_16U`（`short*`、`int16_t` SIMD）累加。
- **影响**：单位置累加上限 **255**。当 `FEAT_MAX=64` 且强匹配时，累加 = `64×4 = 256 > 255`，发生**回绕（256→0）**，最置信的匹配位置分数归零、漏检；且与 C++ 非 bit-exact。
- **触发条件**：特征数较多（接近声明的 64）且匹配质量高时。典型 20–40 特征模板在 8-bit 范围内仍安全，但设计声明的上限会出错。
- **建议修复**：
  1. 累加器加宽至 **≥9 bit**（最小修正，覆盖 0–511）或 **16-bit**（与 C++ 完全一致）；
  2. 同步加宽 `dout_e/o`、`w_score`、`max_r`、`ev_score`；
  3. `hit_dout` 当前为 `{score[7:0],y[11:0],x[11:0]}=32bit`，16-bit score 需改为 `{score[15:0],y,x}=40bit`，或保留 8-bit 上报但**阈值比较用全宽**（否则 `alg13` 用截断 raw 计算 `score` 仍错）——这是跨模块改动，须重跑 alg11 联仿。
- **优先级**：高（影响高特征数模板的召回率，且与参考实现语义偏离）。
- **第二轮复审补充（2026-08-15）**：新增**方案 A（零资源）**——PS 侧限制 `nfeat ≤ 63` 并在 `sbm_alg11_accum` 增加容量断言，功能上即可消除回绕；方案 B（加宽 9–16bit）会使 BRAM 合计升至 ~91%，须同步评估 lane 数折减。阈值口径已证明整数精确等价（§11.8.2），D2 修复不涉及阈值语义变更。

### 11.4 接口与数据流一致性

- **位宽链路**：`alg1(8b)→alg2(8b)→alg3(8b 单热)→alg8(8b)→alg9(8b)→DDR(64bit)→alg11(读256bit)→alg13` 衔接一致。
- **alg3→alg8**：alg3 输出 `out_r<=8'b1<<best_dir_r`（强梯度+边界清零+投票≥5 门控），alg8 直接 T×T OR —— 等价于 C++ `spread`（标签→位图→OR）。✅
- **位置覆盖**：`sbm_alg11_accum` `CHUNK=(TP_MAX+LANES-1)/LANES=4267`，`BANK_DEPTH=1<<$clog2(4267/2)=4096`，`LANES×CHUNK=102408 ≥ TP_MAX=102400` —— **无覆盖缺口**（前序"静默丢位置"缺陷已修复）。✅
- **双 bank 乒乓 RMW**：`addra_e/o` 用 `j_local>>1` 读、`(j_local-1)>>1` 写（off-by-one 修复已落地）。✅
- **坐标链**：alg9 写 / alg11 读地址互为逆运算，复现 C++ `linearize` cell 序。✅

### 11.5 工程落地可行性（XCZU3EG：216 BRAM/972KB、360 DSP、~154K LUT）

**资源（BRAM 为绑定约束）**：

| 模块 | 占用估算 | 备注 |
|---|---|---|
| alg9 双 bank SDPRAM | `2×(T×IMG_W)×64b = 2×20480×64b ≈ 2.62 Mbit ≈ 80 BRAM36` | 占 XCZU3EG **37%**，是最大单块消费者 |
| alg11 24 lane × 2 bank × `BANK_DEPTH(4096)×8b` | ≈ **48 BRAM36** | |
| 模板/特征/BRAM 缓冲 | ≈ **21 BRAM36**（按报告 756KB/1000 模板） | |
| **合计** | **≈149 / 216 ≈ 69%** | 满足 ≤85%，但余量有限 |

- **DSP**：Sobel 2 DSP；CORDIC ArcTan IP 消耗少量 DSP；预计 ≪70%。
- **LUT/FF**：24 lane + 仲裁 + Top-32 归并 + AXI，预计 <70%，**需 Vivado 实测确认**。⚠️ **N-1（第二轮复审新增）**：`sbm_alg9_lut.v` 的 `seg_buf[0:7][0:SEG_BUFS-1][0:SEG_LEN-1]` = 8×4×320×8b ≈ **81,920 FF**（XCZU3EG ~141k FF 的 58%），为 **FF 绑定约束**，放行前必须 RAM 化（BRAM/LUTRAM）并补读侧流水；整改后 BRAM 估算修正为 **≈142/216（66%）**。
- ⚠️ **注意**：若 D2 修复加宽至 16-bit，lane BRAM 由 48 升至约 96 → 合计 ≈197/216≈**91%，超 85%**。需在"加宽到 9-bit"与"降 lane 数"间权衡。

**时序**：全流水结构；关键路径为 alg9 LUT 读 + 组合比较、alg11 RMW 2 拍环、CORDIC 流水 ArcTan。**200MHz 目标 plausibly 可达，但必须经 Vivado STA 确认**（尤其 alg11 多 lane 仲裁与 AXI 互联）。联仿中 XPM 行为模型报的端口位宽警告为行为模型产物，综合用真实原语自适配。

**功耗**：整系统 ≤8W 预算；BRAM（~149 block）+ 24 lane@200MHz + DDR 访问为主源。**须用 Xilinx Power Estimator 实测**，列为验收项。

**硬件可综合性**：依赖 Xilinx XPM 原语（spram/sdpram/fifo_async），仿真用行为模型、综合用真实原语，结构正确；无不可综合结构（`$readmemh` 仅仿真装载）；`sbm_geometry.vh` 纯宏派生可综合。⚠️ **次要**：`sbm_alg11_accum.v` 中一处编译期断言恒成立（无效断言），真正保护在运行时 `cfg_tp` 检查，建议修正或删除。

**吞吐量 / 500 模板@10fps 风险**：24 lane 仅并行化"**位置扫描**"，不并行"**模板**"；单引擎串行处理模板。

- **小模板**（~1000 候选位/特征、40 特征）≈ 17µs/模板 → 500 模板 ≈ 8.3ms/帧，**可达 10fps**；
- **大模板**（候选位接近 `TP_MAX`、特征多）可升至数百 ms/模板 → 500 模板远超 100ms 预算。
- **结论**：500 模板@10fps 仅在**中小模板**下可行；大模板/高 TP 需模板级并行（多 `alg11` 实例）、降低模板数或提频。列为**架构风险**。

### 11.6 验证完备性与盲点

- 8 个模块级联仿**全部 PASS**（F1 lane 扫描、F4 alg9 布局、F5a 多分辨率 128×64/64×40/512×512/连续帧/背压、F6 端到端、几何自检）。
- **盲点**：均为模块自洽联仿（TB 用与 DUT 相同输入/`.mem` 反推 golden），验证的是**地址布局/握手/协议/坐标对齐**，而非**跨模块真实数据正确**。D1（斜坡）正是此类盲点的典型：模块 PASS 而真实响应值全错。
- **缺失**：①系统级集成测试台（`alg1→…→alg13` 用真实图像端到端）；②用 `line2Dup.cpp` 生成的**真值响应图/匹配结果**做 alg9/alg11 交叉验证。
- **第二轮复审新增盲点（2026-08-15）**：①`tb_sbm_alg1_gaussian` 为**恒真空壳**——无任何比对逻辑即打印 PASS，激励为 40 字节 lane 填充 64×48 图像（其余为 X），V-2；②`tb_sbm_alg2_cosim` 仅验证 mag2 与延迟后 CORDIC 输入平方对齐、`tb_sbm_alg3_cosim` 仅层次探针同义反复校验（angle 恒 0、`m_axis_tdata` 从未检查），且两者均未接入 Makefile 回归，V-3/V-5；③CORDIC 行为模型延迟 21 拍为假设值，真实 IP Latency 从未在 Vivado 中核对（F2 自检仅保证"运行时对齐"，不保证"延迟参数正确"），V-4。详见 §11.8.4。

### 11.7 行动建议（按优先级）

1. **[必须]** 修复 **D2**：累加器加宽至 16-bit（与 C++ 一致），同步 `hit_dout`/`alg13` raw 宽度，重跑 alg11 联仿。
2. **[已做]** **D1**：`similarity_lut.mem` 已重生成（真实 LUT），联仿复跑 PASS。
3. **[建议]** 新增系统级集成 TB + C++ 真值交叉验证，消除"假 PASS"盲点。
4. **[建议]** 在 Vivado/XPE 实测 BRAM/DSP/LUT/功耗/时序，确认 XCZU3EG 落地；评估 D2 加宽后 BRAM 余量。
5. **[建议]** 明确 500 模板@10fps 的模板尺寸假设，必要时引入模板级并行。
6. **[次要]** 修正文档偏差（LUT 来源/LUTRAM 路数，本次已更新）与 `sbm_alg11_accum.v` 无效断言。

### 11.8 第二轮全量复审判定（2026-08-15 融合）

> 复审方式：以 `Ref.Code/line2Dup.cpp` 为语义基准，对 Alg.Code 全部 RTL/C 模块重新逐行核验，覆盖算法转换正确性、XCZU3EG 移植可行性、验证闭环三条主线。本节判定与 §11.1–11.7 融合，冲突处以本节为准。

#### 11.8.1 逐模块判定

| 模块 | 判定 | 要点 |
|---|---|---|
| `sbm_geometry.vh` | ✅ 无缺陷 | 5000→级0 5120（16T 对齐）→级1 2560（8T 对齐）→WC=HC=320、CELLS=102400；级1 8T 倍数→WC 8 整除→64bit 写 8B 对齐，约束链自洽 |
| `sbm_alg1_gaussian` | ✅ 算法一致（验证缺失） | 核 `[2,7,14,18,14,7,2]/64` + `(sum+32)>>6`，四边界复制语义与 OpenCV BORDER_REPLICATE 一致（±1 LSB）；TB 为恒真空壳（V-2） |
| `sbm_alg2_sobel` | ✅ 保留两项 | dx/dy ±1020、mag2 22bit 安全（2×1020²=2,080,800<2²¹）；CORDIC 输入 `{dx,dy}` 位序与 PG105 一致；保留 V-3（功能比对缺失）、V-4（真实 IP 延迟未验证） |
| `sbm_alg3_quantize` | ✅ | `(angle+2048)>>12` 桶映射与 C++ 16 桶/360° 折叠在 0°/180°/348.75°–360° 边界逐点等价；投票 ≥5、严格大于链式平票取小索引、`mag2>900` 与 C++ 一致 |
| `sbm_alg8_spread` | ✅ | 因果 T×T OR + 丢前 T-1 行列 ≡ C++ 前向 OR（交换律+补零）；全流水冻结背压正确 |
| `sbm_alg9_lut` | ✅ 保留 N-1 | 查表 `max(SIM[gi*32+lsb], SIM[gi*32+16+msb])`、落盘 `RESP_BASE+(ori*T²+block)*CELLS+band*WC+cx` 与 C++ `linearize` 严格同构；**seg_buf ≈81.9k FF 须 RAM 化（N-1）** |
| `sbm_alg11_accum` | ✅ 保留 D2/N-2/N-3/N-4 | 读回基址含 ori 偏移，与 alg9 落盘公式互逆；阈值 `raw_thr=floor(thr*4*nfeat/100)` 严格大于，与 C++ 浮点口径整数精确等价 |
| `sbm_accum_lane` | ✅ 保留 D2 | 双 bank RMW、读 `j>>1`/写 `(j-1)>>1` 差一修复、hit 打包正确；8bit 加法在 nfeat=64 时 256 回绕（D2 核心） |
| `sbm_alg13_nms.c` | ✅ 保留 3 小项 | 坐标链 `x1=cands.x*T1+OFFSET1 → (x1*2+1)/T0 → (cx-8+bdx)*T0+OFFSET0` 正确；N-6 方形假设、N-7 `>=` vs `>`、`uint8_t acc[16][16]` 同受 nf≤63 约束 |

#### 11.8.2 关键口径核验结论

1. **阈值等价性**：RTL `score > floor(thr*4*nfeat/100)` ⇔ `raw*100 > thr*4*nfeat` ⇔ C++ 浮点 `score > threshold`，整数精确等价、无浮点漂移。
2. **DDR 布局互逆**：alg9 落盘公式、alg11 读回基址、alg13 级0 寻址三方严格同构于 C++ `accessLinearMemory`（`grid_index=(y%T)*T+x%T`、`lm_index=(y/T)*W+x/T`）。
3. **桶映射回绕**：signed fraction `(angle_q16+2048)>>12` 与 C++ 0–360°×16/360 在全部回绕边界逐点等价（0°/360° 均落 label 0）。
4. **D1 修复确认**：`similarity_lut.mem` 前 40 项与 `line2Dup.cpp` L737 逐一吻合（`00 04 03 04 …` / `00×8 03×8` / `00 03 04 04 03 03 04 04`）。
5. **pyrDown 缺席**：级1 图像由外部提供为架构前提，非代码缺陷。

#### 11.8.3 移植可行性判定（XCZU3EG）

- **BRAM**：≈142/216（66%）——可行，余量优于 §11.5 初估；
- **FF**：`seg_buf` 81.9k FF 为**绑定约束**（N-1），RAM 化为放行前置项；整改后 FF 充裕；
- **DSP**：仅 Sobel/CORDIC 共约 2 个，充裕；
- **吞吐**：单帧实时 ✅；500 模板@10fps 需模板级并行（架构风险，维持 §11.5 结论）。

#### 11.8.4 验证闭环判定

- **后端（alg8/9/11）坚实**：LFSR 非退化激励、显式 x 检测、wstrb 感知、8B 对齐断言、逐字节黄金反解 `(ori,block,band,cx)→(y,x)`、连续多帧回归均已覆盖。注意 alg9 golden 与 DUT 共用同一 `.mem`，为 D1 历史假 PASS 的根因模式。
- **前端（alg1/2/3）存在系统性空洞**：V-2（tb_alg1 恒真空壳）、V-3/V-5（alg2/3 TB 仅对齐验证且脱离 Makefile 回归）、V-4（CORDIC 真实 IP 延迟未验证），详见 §11.6。

#### 11.8.5 缺陷清单（融合优先级）

| 编号 | 缺陷 | 等级 | 影响范围 | 修复建议 |
|---|---|---|---|---|
| D2 | lane 累加器 8bit，nfeat=64 时 256 回绕 | **P0** | 高特征数模板漏检 | 方案 A：PS 限 nfeat≤63 + alg11 容量断言（零资源）；方案 B：加宽 9–16bit（BRAM 升至 ~91%，需权衡 lane 数） |
| N-1 | alg9 `seg_buf` ≈81.9k FF | **P0** | XCZU3EG FF 绑定约束 | RAM 化（BRAM/LUTRAM）+ 补读侧流水 |
| V-2 | tb_alg1 恒真空壳 | **P0** | 前端算法正确性无证据 | 重写为 C-golden 两级高斯逐字节比对并接入 Makefile |
| V-3/V-5 | alg2/alg3 TB 仅对齐验证且脱离回归 | P1 | 量化/投票逻辑无功能验证 | 补 C-golden 功能比对并接入 `make all` |
| V-4 | CORDIC 真实 IP 延迟未验证 | P1 | alg2 全链对齐 | Vivado 中核对 IP Latency 与 `CORDIC_IP_LATENCY` |
| V-1 | 无系统级集成测试台 | P1 | 跨模块真实数据正确性 | 建 alg1→alg13 端到端 TB + C++ 真值交叉验证 |
| N-3 | `cand_fifo` 核心域复位直连 `.rst(~rst_n)` | P1 | 复位跨域亚稳态 | 改为 wr 域复位 + 异步 FIFO 标准复位策略 |
| N-6 | alg13 级0 寻址方形假设（`wc0*wc0`） | P2 | 非方形图像 | 引入 `wc0`/`hc0` 分离参数 |
| N-4 | start 脉冲 2FF 同步余量为 0 | P2 | 偶发丢启动 | 加握手或展宽脉冲 |
| N-2 | `sbm_alg11_accum.v` L98 编译期断言恒假/无效 | P3 | 误导性保护 | 修正或删除 |
| N-7 | alg13 `score[i] >= MIN_SCORE` vs C++ `>` | P3 | 阈值恰等时多 1 候选 | 改为严格大于 |

> **复审结论**：除 D2 外，数据链路与参考 C++ 语义一致；**P0 三项（D2、N-1、V-2）闭环后可进入系统集成与上板阶段**。
>
> **2026-08-18 更新**：本表 V-2 与 V-3/V-5 已在 N-8 前端协议闭环中关闭（C-golden 功能 TB 重建并接入 `make all`，同时捕获并修复 N-8/N-9/N-14→N-27 系列实体缺陷）；P0 前置项仅剩 **D2 与 N-1**。详见 §12。

> *附：本次已落盘文件改动 —— `similarity_lut.mem`（重生成，256 项真实 LUT）。原斜坡备份 `similarity_lut.mem.ramp_bak` 已随目录清理移除（历史缺陷 D1 记录保留于本节）。*

---

## 12. N-8 前端协议统一与验证闭环（2026-08-18）

### 12.1 任务背景与验收标准

第六轮复核（§11.8）认定前端（alg1→alg3）**不可用**：存在 N-8 级联协议缺陷（alg1 行门控丢像素、gauss_h 冲刷软契约、帧标记语义不统一），且被 V-2/V-3/V-5 验证空洞长期掩盖。本轮整改三项任务全部闭环：

1. **任务 1**：统一 AXI4-Stream 帧协议（tuser=帧首、tlast=帧末，行边界内部派生）；
2. **任务 2**：sbm_gauss_h 行尾冲刷硬件强制（消除软契约依赖）；
3. **任务 3**：补建 alg1→alg3 C-golden 功能测试台并接入 Makefile。

**验收标准（已实证）**：N-8 类协议级缺陷（行门控错误、补零拍被吞、帧间状态残留）必须能被逐行逐字节比对捕获并报告 FAIL——闭环过程中三个 TB 累计捕获 20+ 真实缺陷（每项修复前均先见 FAIL、修复后复跑 PASS），见 §12.5。

### 12.2 任务 1：统一 AXI4-Stream 帧协议

- **协议风格**：全链统一采用 "tuser 标记帧首像素、tlast 标记帧末像素"，行边界由各模块内部列计数器自行派生，不依赖外部行同步信号。以 sbm_alg2_sobel.v / sbm_alg3_quantize.v 为参考基准（内部 `s_row/s_col` 计数器 + `row_active/flush_c/flush_row` 状态机）。
- **sbm_alg1_gaussian.v 顶层重写**（详见 §5.1）：删除 `row_active` 对输入消费的门控（N-8a）；改用自由列/行计数器 + tuser/tlast 驱动的帧状态机；数据消费仅取决于 `tvalid && tready` 握手；帧首拍显式清零全部计数器与冲刷状态（帧间无残留）。
- **一致性保证**：修改后 tuser/tlast 语义、输出像素总数（IMG_W×IMG_H）与下游 alg2 输入期望完全一致（TB1/TB2 级联风格验证）。

### 12.3 任务 2：sbm_gauss_h 行尾冲刷硬件强制

- **问题（N-8b）**：gauss_h 的 `flush_c`（行尾补 3 拍末像素复制）原仅在 `i_valid=0` 拍递减；若上游背靠背满速送数，补零拍被吞掉导致行尾数据错乱。
- **修法（方案 A，照搬 alg8 P2）**：`flush_c≠0` 期间 `o_ready=0`，顶层据此联合拉低 `s_axis_tready`，硬件强制行间隙，补零拍必然排空。（方案 B：输入端加 ≥3 拍 skid/FIFO 缓冲，未采用。）
- **验证**：任意上游发送节奏（含满速背靠背）下行尾补 3 拍均正确完成，每行输出 IMG_W 个有效像素无气泡（TB1 实证 `backpressure observed=1`，PASS 32768 pixels）。

### 12.4 任务 3：C-golden 功能 TB 与 Makefile 接入

三个 TB 均为非退化激励 + 逐拍比对（含 tuser/tlast 时序校验）+ 双帧发送（帧间状态残留/计数器不回卷立即暴露）：

| TB | 黄金参考 | 激励图样 | 结果 |
|---|---|---|---|
| `tb_sbm_alg1_gaussian.v`（重写，V-2 关闭） | C-golden 两级高斯 [2,7,14,18,14,7,2]/64（BORDER_REPLICATE）逐像素参考 | LFSR/随机非退化图样，覆盖帧首/帧尾/行首/行尾边界 | PASS 32768 |
| `tb_sbm_alg2_sobel.v`（新建，V-3 关闭） | C-golden dx/dy/mag2/angle（共享 `cordic_atan2_func.vh` 参考函数） | ramp 基底 + 中心 1/4 亮方块（四象限强梯度）+ LCG 噪声 | PASS 32768 |
| `tb_sbm_alg3_quantize.v`（新建，V-5 关闭） | C-golden 量化方向单热（q16 桶映射、边界清零、3×3 投票、strong 门、votes≥5） | 分区图样：强/弱块、棋盘格（5/4 票边界）、mag2=900/901 交替（阈值边界）、桶回绕、伪随机背景（全 8 方向） | PASS 32768 |

**Makefile 接入**：新增 `run_alg1 / run_alg2 / run_alg3` 并纳入 `all` 目标；PASS/FAIL 校验采用 `$(VVP) $< | tee sim_algN.log` + `grep -q '^PASS' sim_algN.log`（TB 无 `$fatal`，无 PASS 行即 make 非零退出）；`clean` 同步清理新增产物。历史 TB（`tb_sbm_alg2_cosim.v` / `tb_sbm_alg3_cosim.v`）仅做对齐/同义反复验证，功能比对已由新 TB 完全取代。

### 12.5 缺陷捕获与修复总账

闭环过程中 C-golden TB 逐项捕获并驱动修复的实体缺陷（全部先 FAIL 后 PASS）：

| 模块 | 编号 | 缺陷概要 | 捕获 TB |
|---|---|---|---|
| alg1 | N-8a | `row_active` 门控吞帧首拍/行首像素 | TB1 |
| alg1 | N-9 | 行首标记组合判据产生 2 拍宽脉冲，全图错位 | TB1 逐拍探针 |
| gauss_h | N-8b | 行尾补 3 拍软契约，满速背靠背吞补零拍 | TB1 |
| alg2 | N-14 | 帧首拍被 `row_active=0` 门控丢弃，整帧左移 1 列 | TB2 |
| alg2 | N-19 | 行尾补拍软契约，输出相位逐行对角漂移 | TB2（ramp 图样） |
| alg2 | N-20 | 加法树 ≥512 和 `$signed()` 别名为负 | TB2（ramp 图样） |
| alg2 | N-22 | 末行 pad 窗口当前行源误用 lb 读 | TB2 |
| alg2 | N-23 | 帧尾补行行首装载 `win[0]` 错取末行复制 | TB2 |
| alg2 | N-24 | `out_row` 帧间不回卷，第二帧 tuser 丢失（数据全对仅协议错） | TB2 双帧协议校验 |
| alg3 | N-14 | 同 alg2，量化方向整体错列 | TB3 |
| alg3 | N-19 | 行尾补拍软契约（预防性硬件强制） | TB3 |
| alg3 | N-24 | `out_row` 帧间不回卷 | TB3 双帧协议校验 |
| alg3 | N-25 | `flush_row` 清除沿吞补行换行拍，末行末列输出永缺 | TB3 |
| alg3 | N-26 | 标签链与数据路径错位 2 拍，帧尾多发 H-1 拍 | TB3 + 判定点仪表化探针 |
| alg3 | N-27 | strong 门源错位 1 行 1 列（块边界对角错 577 处） | TB3 错误模式分析 |

**定位方法论沉淀**：手推 beat 级时序多次自相矛盾时（N-25 三次方案迭代失败），改用判定点仪表化（直接打印 `vld_out` 拍的坐标标签与发射判定）与错误坐标模式分析（块边界对角错 → 门源错位），一击定位。

### 12.6 回归结果

`make all` 全量回归（2026-08-18）：既有 7 个目标（run_fix / run_alg9 / run_alg9_multi / run_alg11 / run_alg9_cont / run_alg8 / run_geom）+ 新增 3 个前端 TB 全部 PASS，退出码 0。

```
PASS: 32768 pixels matched, protocol clean, backpressure observed=1        # run_alg1
PASS: 32768 pixels (dx/dy/mag2/angle + protocol) matched                  # run_alg2
PASS: 32768 pixels (quantized dir one-hot + protocol) matched             # run_alg3
```

至此 §11.8.5 缺陷清单中 **V-2（P0）与 V-3/V-5（P1）全部关闭**，P0 前置项仅剩 D2 与 N-1；前端 alg1→alg3 已具备与后端同级的功能正确性证据。

---

> **说明**：本 README 描述代码架构、模块职责与设计思路，并整合 §11 算法正确性与落地可行性评审；具体联仿命令、开关与预期结果以 `README_cosim.md` 为准。
# SBM_FPGA
