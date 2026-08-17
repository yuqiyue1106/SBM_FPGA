# SBM FPGA 联仿手册（README_cosim.md）

基于 iverilog + vvp 的 OSS 联仿环境（无需 Vivado）。

## 快速开始

```bash
make all          # 跑全部联仿: F1 / F4 / F5a 多分辨率 / F6+F9 alg11 / 几何自检
make run_fix      # F1: lane 候选扫描(7 FIFO hits matched)
make run_alg9     # F4: alg9 落盘布局(64x64)
make run_alg9_multi  # F5a: alg9 多分辨率(120x64 末拍wstrb / 64x40 非方 / 512x512)
make run_alg11    # F6+F9: alg11 端到端黄金比对(3 case)
make run_geom     # 几何单一真源自检(默认 5000x5000)
```

预期输出均为 `RESULT: PASS` / `GEOM RESULT: PASS`。

## 换相机分辨率

只改 `sbm_geometry.vh` 顶部 4 个宏（`SBM_CAM_W` / `SBM_CAM_H` / `SBM_T` /
`SBM_LANES`），级0 对齐、级1 尺寸、WC/HC/CELLS、通道容量、全部位宽自动派生。
验证方式：

```bash
# 用 -D 覆盖宏模拟换相机后跑几何自检
iverilog -g2012 -DSBM_CAM_W=1920 -DSBM_CAM_H=1080 -o /tmp/g.out tb_sbm_geom_check.v
vvp /tmp/g.out
```

## alg9 联仿要点（F4/F5a 修复后）

- TB 用 **16bit LFSR 非退化图样**（旧图样 `(y*W+x)&0xFF` 在地址错位场景下会
  退化巧合，曾造成假 PASS）。
- TB 显式检测 `m_axi_awaddr` / `m_axi_wdata` 的 **x 值**（旧 TB 用 integer 承接
  地址，x 参与比较得 unknown → if 判假 → err 不累加而 tot 照常 → 假 PASS）。
- 末拍 `wstrb` 部分选通：`SEG_BEATS = ceil(WC/8)`，无效字节跳过比对。

## alg11 联仿要点（F6/F9 重写后）

- 3 个 case：常规命中（thr=40）、全命中背压压力（thr=0，验证并发 drain 不溢出）、
  8 特征 top-32 淘汰（验证满表淘汰语义）。
- 黄金模型：越界特征过滤（与 DUT bounds check 同口径）+ 命中收集后稳定选择
  排序取 top-32（与 DUT 无序 Top-32 多集等价）。
- `xpm_fifo_async_beh.v` 为 XPM 异步 FIFO 行为模型（distributed / fwft）。

## 联仿环境变量与依赖

- `iverilog` / `vvp`（icarus-verilog）
- `similarity_lut.mem` 须与 `tb_*.v` 同目录（alg9 `$readmemh` 装载）
- XPM 行为模型：`xpm_memory_spram_beh.v`、`xpm_memory_sdpram_beh.v`、
  `xpm_fifo_async_beh.v`（仅仿真，综合时用真实 XPM 原语）
