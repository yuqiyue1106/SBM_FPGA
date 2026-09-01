# 用本机 ModelSim 仿真 `sbm_alg1_gaussian.v` —— 从零到"输出与理论一致"

> 适用对象：想在 Windows 上用 ModelSim（或 Questa）跑通 alg1 高斯 IP 核，并**逐项确认输出和理论值一致**的人。
> 本文的**每一条命令和每一段输出都在本机实测过**（2026-09-01，仓库 commit `5dd2e30`）。
> 全程**不修改仓库里的任何文件**：所有改动都在 `sim_msim/alg1/` 这个仿真工作目录里做副本。

实测结果预览（照做完你就会得到一样的输出）：

```text
# PASS: 32768 pixels matched, protocol clean, backpressure observed=1
#    Time: 351555 ns  Iteration: 1  Instance: /tb_sbm_alg1_gaussian
```

---

## 0. 先说清楚：这套流程要解决什么

`tb_sbm_alg1_gaussian.sv` 是一个**自带标准答案（C-golden）的自校验 Testbench**：它用 LCG 伪随机数生成两帧 128×128 图像，逐像素喂给 DUT，同时在 TB 里用整数运算独立算一遍"两级 7 抽头高斯"的期望值，最后一个字节一个字节比。所以你**不需要自己再去算理论值**就能得到 PASS/FAIL。

但是直接把仓库文件丢进 ModelSim 跑，你会连续撞到 3 个坑：

| # | 现象 | 根因 | 在第几步解决 |
|---|------|------|--------------|
| A | `vopt-2732 Module parameter 'WRITE_MODE_B' not found` ×6，`Error loading design` | 仓库里给 iverilog 用的行为级 XPM 模型没声明 `WRITE_MODE_B` | 第 4 步 |
| B | 编译仿真都不报错，但**输出全是 `x`**，`ERR pixel ... got=x` | ModelSim 把 13bit 连到 32bit 地址端口的**高位补 Z**（iverilog 补 0），带 Z 的索引读出全 X | 第 5 步 |
| C | 想调试时 `examine` 报 `No objects found`；敲 `dir /...` 报 `Parameter format not correct` | 批处理模式默认不生成调试数据库；不认识的命令会被 Tcl 丢给 `cmd.exe` | 第 5、8 步 |

坑 B 是最值得理解的：**同一份 RTL 在 iverilog 下 PASS、在 ModelSim 下全 X**，问题不在被测设计，而在仿真器的 X 传播策略。搞清它，你以后遇到"换个仿真器就挂"就有排查路径了。

---

## 1. 环境自检（30 秒）

打开 **cmd.exe**（不要用 Git Bash，原因见 6.4），执行：

```bat
C:\modeltech64_2020.4\win64\vsim.exe -version
```

实测输出：

```text
Model Technology ModelSim SE-64 vsim 2020.4 Simulator 2020.10 Oct 13 2020
```

如果你装的是 Questa（本机也装了）：

```bat
C:\questasim64_2024.1\win64\vsim.exe -version
Questa Sim-64 vsim 2024.1 Simulator 2024.02 Feb  1 2024
```

> 两条命令的路径按你本机安装位置改。装 ModelSim 时如果勾了 PATH，可以直接敲 `vsim -version`。
> 后文所有命令里的 `vsim/vlog/vlib` 同理（Questa 的命令行与 ModelSim 一致，本文第 7 步末尾有 Questa 实测结论）。

---

## 2. 建一个仿真工作目录（仓库文件一个字都不动）

```bat
cd /d C:\Users\yuqiy\Desktop\SBM_FPGA
mkdir sim_msim\alg1
cd sim_msim\alg1
copy ..\..\tb_sbm_alg1_gaussian.sv .
copy ..\..\sbm_alg1_gaussian.v     .
copy ..\..\sbm_gauss_h.v           .
copy ..\..\sbm_gauss_v.v           .
copy ..\..\sbm_geometry.vh         .
copy ..\..\xpm_memory_sdpram_beh.v .
```

为什么要平铺到同一个目录：TB 第 42 行是 `` `include "sbm_alg1_gaussian.v" ``（**include**，不是实例化），顶层模块是被"抄"进 TB 编译单元的；`sbm_alg1_gaussian.v` 内部又会 `` `include "sbm_geometry.vh" ``。同一目录最省事。

少 copy 一个文件，`vlog` 会直接告诉你缺谁（实测）：

```text
** Error: tb_sbm_alg1_gaussian.sv(42): Cannot find `include file "sbm_alg1_gaussian.v" in directories:
```

⚠️ **不要**把 `sbm_alg1_gaussian.v` 再单独列进 `vlog` 的文件参数里——它已经被 TB include 过一次，重复编译会报：

```text
** Warning: sbm_alg1_gaussian.v(23): (vlog-2275) Existing module 'sbm_alg1_gaussian' at line 23 will be overwritten.
```

---

## 3. 第一次编译

```bat
C:\modeltech64_2020.4\win64\vlib.exe work
C:\modeltech64_2020.4\win64\vlog.exe -sv +incdir+. tb_sbm_alg1_gaussian.sv sbm_gauss_h.v sbm_gauss_v.v xpm_memory_sdpram_beh.v
```

实测输出（这就是"编译成功"的样子）：

```text
-- Compiling module sbm_alg1_gaussian
-- Compiling module tb_sbm_alg1_gaussian
-- Compiling module sbm_gauss_h
-- Compiling module sbm_gauss_v
-- Compiling module xpm_memory_sdpram
Top level modules:
	tb_sbm_alg1_gaussian
Errors: 0, Warnings: 0
```

> `-sv` 必须加：TB 是 `.sv`。`+incdir+.` 让 `` `include `` 在任何调用目录下都能找到本地文件。
> 注意 `Errors: 0` **只代表语法没问题**，端口/参数级的错要等 `vsim`  elaborate 才暴露——所以别在这里就宣布成功。

---

## 4. 第一次 elaborate —— 撞坑 A 并修掉它

```bat
C:\modeltech64_2020.4\win64\vsim.exe -c -do "run -all; quit -f" work.tb_sbm_alg1_gaussian
```

实测输出（`sbm_gauss_v.v` 里 6 个行缓冲各报一条）：

```text
** Error (suppressible): sbm_gauss_v.v(416): (vopt-2732) Module parameter 'WRITE_MODE_B' not found for override.
... ×6
Error loading design
```

读一下原因：`sbm_gauss_v.v:409-416` 例化行缓冲时写的是

```verilog
xpm_memory_sdpram #(
    .MEMORY_SIZE        (8192*8),
    ...
//  .WRITE_MODE_A       ("no_change")
    .WRITE_MODE_B       ("no_change")
) u_lb (
```

真实 Vivado XPM 原语确实有 `WRITE_MODE_B` 这个参数，但仓库里那份给 iverilog 用的**行为级替身** `xpm_memory_sdpram_beh.v` 只声明了 `WRITE_MODE_A`。iverilog 对多余的参数覆盖比较宽松，ModelSim 直接判错。

**修法（只改工作目录里那份行为级模型，它是纯仿真文件，不参与综合）：**

打开 `sim_msim/alg1/xpm_memory_sdpram_beh.v`，把参数表补全：

```verilog
	parameter READ_LATENCY_B      = 1,
	parameter WRITE_MODE_A        = "read_first",
	parameter WRITE_MODE_B        = "read_first",
	parameter SIM_ASSERT_CHK      = 0,
	parameter SIM_GENCHECK        = 0
)(
```

改完重新 `vlog`（第 3 步那条命令），再 elaborate：设计能加载了，但进入第 5 步的坑 B。

---

## 5. 输出全是 `x` —— 定位并修掉坑 B

重新跑：

```bat
C:\modeltech64_2020.4\win64\vsim.exe -c -do "run -all; quit -f" work.tb_sbm_alg1_gaussian
```

实测输出：

```text
# ERR pixel @ frame=0 row=0 col=0: got=x exp=169
# ERR pixel @ frame=0 row=0 col=1: got=x exp=135
...
# ERR pixel @ frame=0 row=0 col=9: got=x exp=156
# FAIL: err=32768 total=32768 (exp 32768)
```

`exp=` 是 TB 自己算的期望值（有数），`got=x` 说明 DUT 输出是未知态。注意 TB **只打印前 10 条** `ERR pixel`，真正的规模看最后那行 `FAIL: err=32768`——是**每一个像素**都错，不是个别位置算错。这种"全错 + 全 X"的形状基本可以锁定：**某个存储/地址被 X 污染了**，而行缓冲正是这里唯一的存储。

### 5.1 一个 `.do` 文件把证据抓出来（批处理模式）

先看信号值必须先加 `-debugdb`，否则 `examine` 找不到对象。把下面几行存成 `sim_msim/alg1/ex.do`：

```tcl
examine /tb_sbm_alg1_gaussian/u_dut/u_v/gen_lb[0]/u_lb.addra
run 2000ns
examine /tb_sbm_alg1_gaussian/u_dut/u_v/gen_lb[0]/u_lb.addra
examine -radix binary /tb_sbm_alg1_gaussian/u_dut/u_v/gen_lb[0]/u_lb.addra
quit -f
```

```bat
C:\modeltech64_2020.4\win64\vsim.exe -c -debugdb -do "do ex.do" work.tb_sbm_alg1_gaussian
```

实测输出（省略编译警告）：

```text
# 32'hzzzzXxxx
# 32'hzzzzZ034
# zzzzzzzzzzzzzzzzzzz0000000110100
```

**这就是证据**：第二行是跑过一段时间之后的 `addra`——低 13 位是正常值 `0x034`（=52），高 19 位全是 `Z`。`addra` 端口声明是 32 bit（`xpm_memory_sdpram_beh.v` 第 32 行），设计里连的是 13 bit 的 `w_addra`/`pix_cnt`，ModelSim 把高位填成 `Z`。行为级模型里做的是 `mem[addra]`——一个含 Z 的索引 → 读出 `X`，于是整帧全 X。iverilog 填的是 `0`，所以同一份代码在那边"恰好"是对的。

（顺带：编译时那 12 条 `vsim-3015 [PCDPC] Port size (32) does not match connection size (13) for port 'addra'/'addrb'` 警告说的就是同一件事，别忽略它。）

### 5.2 修法：把地址截到真实位宽

继续改 `sim_msim/alg1/xpm_memory_sdpram_beh.v`（同一份文件），在 `localparam DEPTH` 后面加掩码，并把 `mem[]` 的三处使用换成掩码后的信号：

```verilog
	localparam DEPTH = MEMORY_SIZE / WRITE_DATA_WIDTH_A;
	localparam ADDR_BITS = $clog2(DEPTH);
	...
	// The real XPM address ports are $clog2(DEPTH) bits wide. ModelSim pads a
	// narrow connection with Z (iverilog pads with 0), and mem[addr_with_z]
	// reads all-X -- so mask to the usable bits here.
	wire [ADDR_BITS-1:0] wr_addr = addra[ADDR_BITS-1:0];
	wire [ADDR_BITS-1:0] rd_addr = addrb[ADDR_BITS-1:0];

	always @(posedge clka) begin
		if (ena && wea) mem[wr_addr] <= dina;
	end

	wire w_collision = (ena && wea && (wr_addr == rd_addr));
	...
			pipe[0] <= mem[rd_addr];         // old (pre-write) data
```

> 两处改动都只做一件事：让替身模型的行为**贴住真实 XPM 原语**（真实原语地址端口只有 `$clog2(DEPTH)` 位，天然没有高位 Z）。所以这不是"为了过仿真而糊弄"，而是修正测试夹具。

### 5.3 再跑一次 —— 应该 PASS

```bat
C:\modeltech64_2020.4\win64\vlog.exe -sv +incdir+. tb_sbm_alg1_gaussian.sv sbm_gauss_h.v sbm_gauss_v.v xpm_memory_sdpram_beh.v
C:\modeltech64_2020.4\win64\vsim.exe -c -do "run -all; quit -f" work.tb_sbm_alg1_gaussian
```

实测输出：

```text
# PASS: 32768 pixels matched, protocol clean, backpressure observed=1
# ** Note: $finish    : tb_sbm_alg1_gaussian.sv(245)
#    Time: 351555 ns  Iteration: 1  Instance: /tb_sbm_alg1_gaussian
# End time: ... Elapsed time: 0:00:01
# Errors: 0, Warnings: 270
```

### 5.4 这行 PASS 到底证明了什么（判读方法）

`tb_sbm_alg1_gaussian.sv` 末尾的判定（第 234-242 行）是三件事同时成立：

| 判据 | 含义 | 数字怎么核对 |
|------|------|--------------|
| `pixels matched` 计数 | **每一个**输出字节都等于 TB 内 C-golden | `NFRAMES × IMG_W × IMG_H` = 2×128×128 = **32768**，对上了才说明不多不少 |
| `protocol clean` | `tuser` 只在帧首拍、`tlast` 只在行末拍，AXI4-Stream Video 时序无违例 | 由 `ERR protocol ...` 分支（第 175-186 行）驱动 |
| `backpressure observed=1` | 帧 1 紧跟帧 0 时**确实**观察到 `tvalid=1 && tready=0` | 说明底部冲刷期的反压路径被测到了，不是"侥幸没撞上" |

`Time: 351555 ns` 是**仿真时间**（时钟 10 ns 周期，见 TB 第 73 行 `always #5 clk = ~clk;`），`Elapsed time: 0:00:01` 才是你机等价的真实耗时。

### 5.5 关于那 270 条 Warning（重要：有一个是 DUT 自检写错了）

跑完先看一眼警告构成：

```bat
C:\modeltech64_2020.4\win64\vsim.exe -c -do "run -all; quit -f" work.tb_sbm_alg1_gaussian 2>&1 | findstr /C:"tlast off row-end" | find /c /v ""
```

实测：`256` 条，内容都是

```text
# ** Warning: ALG1: tlast off row-end (pix_cnt=127 exp=126)
```

**结论：这不是数据错，是 DUT 内部那条自检本身 off-by-one。** 推理链很短：
- 256 = `NFRAMES × IMG_H` = 每行恰好一次 → 每行都"报警"，但 TB 的协议校验却判定 `protocol clean`；两边对同一件事的结论矛盾，说明其中一个是错的。
- `sbm_alg1_gaussian.v:146` 的检查是 `if (w_consume && s_axis_tlast && (pix_cnt != (i_img_w-2)))`，而 `pix_cnt` 是在**消费前**采到的：行末拍的 `pix_cnt` 应该是 `IMG_W-1 = 127`，不是 `126`。TB 侧 `s_axis_tlast <= (c == IMG_W-1)`（第 154 行）才是符合 AXI Video 定义的做法。
- 所以该行的期望值应写成 `i_img_w-1`（**这是仓库待办，本文不改**）。

剩下 14 条是端口位宽/参数提示类（`vsim-3015`、`vopt-3040`），无害但值得读一遍。

---

## 6. 调参实验：不改文件换分辨率、加压力

### 6.1 换分辨率（验证"动态图像尺寸"这条新功能）

HEAD 的 `sbm_gauss_v` 已经没有 `IMG_W/IMG_H` 参数了，尺寸由 TB 通过 `i_img_w/i_img_h` 端口送进去（TB 第 64 行），所以**一个 `-g` 就够**：

```bat
C:\modeltech64_2020.4\win64\vsim.exe -c -gIMG_W=64 -gIMG_H=48 -do "run -all; quit -f" work.tb_sbm_alg1_gaussian
```

实测：

```text
# PASS: 6144 pixels matched, protocol clean, backpressure observed=1
```

`6144 = 2 × 64 × 48` ✅。想恢复默认就在同目录再跑一次不带 `-g` 的命令（`-g` 只影响本次运行，不写回任何文件）。

### 6.2 打开随机间隙压力测试

TB 默认把帧 0 的随机间隙关掉了（`tb_sbm_alg1_gaussian.sv:217-218`）：

```systemverilog
	//send_frame(0, 1);
	send_frame(0, 0);
```

`send_frame(f, gaps)` 里 `gaps=1` 会用**同一个 LCG** 在像素间插入 0~3 拍空闲。在**工作目录的副本**里把注释交换过来，重编译后实测：

```text
# PASS: 32768 pixels matched, protocol clean, backpressure observed=1
#    Time: 596045 ns
```

数据全对，仿真时间从 351,555 ns 涨到 596,045 ns（喂得慢，排空就慢）——这类"结果不变、时间变长"正是反压路径被正确消化的表现。

### 6.3 想跑更长/更多帧

TB 第 48 行 `parameter NFRAMES = 2;` → `-gNFRAMES=3` 即可（记得期望像素数改成 `3 × IMG_W × IMG_H`）。

### 6.4 Git Bash 的坑（实测）

**别在 Git Bash 里跑带 `/` 开头的 `-g` 参数**。实测把 `-g/u_dut/IMG_W=128` 传进去，ModelSim 收到的是：

```text
# ** Warning: (vopt-3040) Command line generic/parameter "C:/Program Files/Git/u_dut/IMG_W" not found in design.
```

MSYS 把它当路径改写了。要么用 cmd.exe，要么加 `MSYS2_ARG_CONV_EXCL="*"` 前缀。

---

## 7. 波形与 GUI

### 7.1 命令行抓波形（实测可用）

```bat
C:\modeltech64_2020.4\win64\vsim.exe -c -voptargs=+acc -do "log -r /*; run 60us; quit -f" work.tb_sbm_alg1_gaussian
```

生成当前目录的 `vsim.wlf`（128×128、60 µs 实测 2,359,296 字节）。注意用**有界时间 `run 60us`**：

- `run -all` 会让 TB 跑到 `$finish`，**`$finish` 之后的 do 脚本语句不再执行**，`write wave` 什么也不会写出来。
- 有界 run 不会打印 PASS 行（仿真被中途停掉），这是正常的。

### 7.2 打开 GUI 看

```bat
C:\modeltech64_2020.4\win64\vsim.exe -c -do "file open vsim.wlf; quit -f" 
```
更直接的做法：开始菜单启动 **ModelSim SE-64**，然后
1. `File → Open → Waveform...`，或在 Console 里输 `file open vsim.wlf`；
2. 在 Wave 窗口右键 → `Add → Digital Signal`，加 `/tb_sbm_alg1_gaussian/u_dut/*`；
3. 想看握手，就把 `s_axis_tvalid / s_axis_tready / s_axis_tlast / m_axis_tvalid / m_axis_tdata` 放同一组；
4. 右键信号 → `Format → Unsigned/Decimal`，`m_axis_tdata` 的第一个值应该正好是 **169**（第 8 步会解释为什么这个数能当理论锚点）。

### 7.3 GUI 里调试的三条硬规矩（都是踩出来的）

| 规矩 | 不遵守会看到 |
|------|--------------|
| 批处理要读信号值就加 `-debugdb` | `# No object found matching "u_v".` + `** UI-Msg: (vish-4014) No objects found matching '/tb.../u_lb.addra'` |
| 别用 `-novopt` | `** Error (suppressible): (vsim-12110) ... -novopt option is now deprecated` 之后紧跟 `Error loading design`（2020.4 上这个选项已经把设计加载路径打断了） |
| Console 里别敲 `dir /...` | Tcl 认为你不认识这条命令，转手交给 `cmd.exe /c dir /tb_sbm_alg1_gaussian`，报 `Parameter format not correct - "b_sbm_alg1_gaussian"`；看值用 `examine`，多命令写进 `.do` 文件再 `-do "do xxx.do"` |

### 7.4 Questa 实测

同一套文件在 Questa 上结果完全一致（把工具路径换成 `C:\questasim64_2024.1\win64`）：

```text
# PASS: 32768 pixels matched, protocol clean, backpressure observed=1
```

---

## 8. 对照理论：三层独立证据

"和 TB 的 golden 一致"只说明**RTL 和 TB 里的 C 模型一致**——万一两者同时错呢？所以再做两层交叉验证。三层全部实测通过：

### 8.1 第一层：TB 内置 C-golden（已在上文）

`gh()` / `gg()`（TB 第 112-139 行）用**整数**实现"水平 7 抽头 → 四舍五入到 8 bit → 垂直 7 抽头 → 再四舍五入"。这就是硬件的真实语义（两级各自量化），记住这点，8.3 会用到。

### 8.2 第二层：用 NumPy 从零再推一遍

`sim_msim/alg1/theory_check.py` **不读仿真输出**，而是从 LCG 种子出发重新生成同一张图，再用完全独立的向量化实现算一遍高斯：

```bat
cd C:\Users\yuqiy\Desktop\SBM_FPGA\sim_msim\alg1
python theory_check.py
```

实测输出：

```text
[128x128] python double-round  row0 col0..9: [169, 135, 105, 89, 88, 94, 107, 126, 146, 156]
[128x128] ModelSim TB golden                   : [169, 135, 105, 89, 88, 94, 107, 126, 146, 156]
[128x128] MATCH: True
[128x128] single-round (OpenCV style) vs HW: diff px 2798/16384 (17.08%), max |delta| = 1

[64x48] python double-round  row0 col0..0: [149]
[64x48] ModelSim TB golden                   : [149]
[64x48] MATCH: True
[64x48] single-round (OpenCV style) vs HW: diff px 481/3072 (15.66%), max |delta| = 1
```

TB 的激励用 `seed = 32'hC0FFEE`、`state = state*1664525 + 1013904223`、像素取 `lv[15:8]`（TB 第 78-90 行），Python 侧一比一复刻，所以两边的图**逐字节相同**，可以直接对。

### 8.3 第三层：直接从 DUT 输出流里取前 10 个像素

这一层绕开 TB 的 golden，只看硬件吐出来的数。`sim_msim/alg1/spot_check.do`：

```tcl
set ::n 0
set ::last -1
when {m_axis_tvalid === 1'b1 && total < 14} {
  set ::v [examine -radix unsigned /tb_sbm_alg1_gaussian/m_axis_tdata]
  if {$::n < 10 && $::v != $::last} {
    incr ::n
    echo "THEORY-SPOT \[$::n\] tdata=$::v"
  }
  set ::last $::v
}
run -all
quit -f
```

```bat
C:\modeltech64_2020.4\win64\vsim.exe -c -debugdb -do "do spot_check.do" work.tb_sbm_alg1_gaussian
```

实测输出（128×128）：

```text
# THEORY-SPOT [1] tdata=169
# THEORY-SPOT [2] tdata=135
# THEORY-SPOT [3] tdata=105
# THEORY-SPOT [4] tdata=89
# THEORY-SPOT [5] tdata=88
# THEORY-SPOT [6] tdata=94
# THEORY-SPOT [7] tdata=107
# THEORY-SPOT [8] tdata=126
# THEORY-SPOT [9] tdata=146
# THEORY-SPOT [10] tdata=156
# PASS: 32768 pixels matched, protocol clean, backpressure observed=1
```

和 8.2 的 NumPy 列表**逐项相同**。64×48 下第一个值是 `149`，也和 Python 一致。

三段 `.do` 里有几个细节值得记：
- 批处理模式没有 `breakpoint` 命令，`when` 才是等价物；
- `when` 的条件在**每个 delta cycle** 求值，同一个像素可能触发两次 → 用 `set ::last` 去重；
- HDL 变量（`total`、`m_row`）在 `when` 条件里能直接写名字，但在 action 的 Tcl 里**不能**当 `$total` 用，得走 `examine`。

### 8.4 为什么不能拿 OpenCV 一键对

脚本最后一行就是答案：把"两级各自量化"换成"一次浮点求和后只量化一次"（OpenCV `GaussianBlur` 的口径），128×128 上有 **17.08% 的像素差 ±1**，最大偏差 1。也就是说：

- 拿 `cv2.GaussianBlur(img,(7,7),0)` 直接逐像素 assert，**必然**大面积失败，而且失败不代表 RTL 错；
- 正确的比对方式是把硬件语义（`h_round = (sum_h+32)>>6`，再 `out = (sum_v_from_h_round+32)>>6`）搬进你的参考模型——TB 的 `gh()/gg()` 和 `theory_check.py` 都是这么做的。

### 8.5 两个"不用算也能自证"的不变量

想再确认一次"通路是干净的"，可以做这种不需要精确理论值的实验：

1. **常数图不变性（已实测）**：任何常数图经过"边界复制 + 归一化高斯"后必须**逐像素等于同一个常数**——这条不需要任何 golden 就能自证。做法：在 TB 副本里把第 91 行 `img[k] = lv[15:8];` 改成 `img[k] = 8'd128;`，重新 `vlog` 后跑 `spot_check.do` 和 `run -all`。实测：

   ```text
   # THEORY-SPOT [1] tdata=128
   # PASS: 32768 pixels matched, protocol clean, backpressure observed=1
   ```

   整个 32768 拍输出里**只出现 128 这一个值**（spot 脚本会自动去重，所以只打印一行），`ERR pixel` 计数 0。这才是这条实验的真正证据——注意 TB 的 golden 也是从同一个 `img[]` 算出来的，所以"PASS"本身不构成独立证据，"输出恒等于常数"才是。
2. **核权重和**：`2+7+14+18+14+7+2 = 64`，与除数 `/64` 严格相等 → 才不会整体偏亮/偏暗。这一条已经在 `KT[]`（TB 第 96 行）和 Python 的 `KT` 里各写了一遍，两边对得上。

---

## 9. 排错速查表

| 症状 | 真实原因 | 处理 |
|------|----------|------|
| `vopt-2732 ... 'WRITE_MODE_B' not found` + `Error loading design` | 行为级 XPM 替身缺参数 | 第 4 步补参数 |
| `ERR pixel ... got=x exp=...`（全 X） | ModelSim 给窄→宽地址端口高位补 Z | 第 5 步地址掩码 |
| 只在 ModelSim 挂、iverilog 过 | X 传播策略差异（补 Z vs 补 0） | 别怀疑算法，先看端口位宽警告 |
| `vlog-2275 ... will be overwritten` | 顶层被 include 又单独编译 | 从 `vlog` 参数里去掉 `sbm_alg1_gaussian.v` |
| `vopt-2912 / PCDPC 端口对不上` | 顶层与子模块端口集不匹配（重构中间态） | 看 commit；本文用的 HEAD 已对齐 |
| `examine` 说 `No objects found` | 没生成调试数据库 | `vsim -c -debugdb` |
| `-novopt` 报 `Error loading design` | 2020.4 已废弃 | 用 `-voptargs=+acc` |
| `write wave` 之后没有 `.wlf` | `run -all` 走到 `$finish`，后续 do 语句不执行 | 用 `log -r /*; run 60us; quit -f`，产物默认名 `vsim.wlf` |
| `-g/xxx=1` 报 `generic not found`（路径带 `C:/Program Files/Git`） | Git Bash 改写参数 | 换 cmd.exe，或 `MSYS2_ARG_CONV_EXCL="*"` |
| 控制台中文乱码 | cmd 默认 GBK(936) | 脚本/批处理输出保持 ASCII |
| 想确认像素数对不对 | 期望值 = `NFRAMES × IMG_W × IMG_H` | 128×128×2=32768；64×48×2=6144 |
| `tlast off row-end` 每行一条 | DUT 自检 off-by-one（`i_img_w-2` 应为 `i_img_w-1`） | 已知问题，见 5.5；不影响数据 |
| `Cannot find \`include file "sbm_alg1_gaussian.v"` | 第 2 步有文件没 copy 进平铺目录 | 按 §2 的 6 个 `copy` 补齐 |
| Console 里 `dir /...` 报 `Parameter format not correct` | Tcl 把它交给 `cmd.exe` 执行了 | 看值直接 `examine /tb_sbm_alg1_gaussian/u_dut/pix_cnt`（要 `-debugdb`）；看层次用 GUI 的 Browser |

---

## 10. 一键脚本

`sim_msim/alg1/run_all.bat`（内容就是本文命令的串接）：

```bat
cd C:\Users\yuqiy\Desktop\SBM_FPGA\sim_msim\alg1
run_all.bat                :: 128x128，只看 PASS
run_all.bat 64 48          :: 换分辨率
run_all.bat 128 128 wave   :: 顺带产 vsim.wlf（前 60us）
run_all.bat 128 128 spot   :: 打印前 10 个 DUT 输出像素
```

> 脚本第一行 `set MSIM=C:\modeltech64_2020.4\win64` 按你本机路径改。
> 脚本必须从 cmd.exe 跑（见 6.4），且刻意保持 ASCII 输出。
>
> 诚实说明：脚本里**每一条命令**本文都单独执行并核对过输出，但脚本整体没有在这台机器上跑过（我这侧的执行策略禁止直接运行 `.bat`）。所以第一次请用 cmd.exe 手动跑一遍，有任何一步对不上就回到第 3~5 步逐条对照。

---

## 11. 留在仓库里的待办（本次按"先不改代码"只记录不动手）

1. **`xpm_memory_sdpram_beh.v` 落后于 RTL**：缺 `WRITE_MODE_B`（→ ModelSim 直接 elaborate 失败），地址端口 32 bit 与真实原语不符（→ ModelSim 全 X）。建议把本文第 4、5 步的两处改动合回仓库，这样 iverilog / ModelSim / Vivado 三家口径一致。
   另外 `sbm_gauss_v.v:416` 现在传的是 `WRITE_MODE_B("no_change")` 而 `WRITE_MODE_A` 被注释成默认 `read_first`；真实 XPM 要求两者一致，而 N-28 的注释明确论证过"同一拍同地址读写必须返回旧值(read_first)"。这个不一致在行为模型下不报错（本模型只按 `WRITE_MODE_A` 行事），但在真实原语上值得澄清。
2. **`sbm_alg1_gaussian.v:146` 的 `tlast` 自检 off-by-one**，导致每次仿真 256 条误报告警（见 5.5）。
3. **TB 第 217-218 行随机间隙被注释掉**：默认只测了满速背靠背。第 6.2 步证明打开后同样 PASS，建议把 `send_frame(0,1)` 恢复为默认，让反压/稀疏输入常态进入回归。
4. **`.gitignore` 缺失**：`sim_msim/`、`work/`、`*.wlf`、`*.dbg`、`transcript` 都属于产物，建议加忽略（当前 `sim_msim/` 在 `git status` 里是未跟踪目录）。
5. **`sim_msim/` 目录清单**（本文只推荐一个，其余是我为验证结论搭的一次性实验台，可以直接删）：

   | 目录 | 是什么 | 处置 |
   |------|--------|------|
   | `alg1/` | **主工作目录**，第 2~8 步全部在这里，已含两处补丁 + `run_all.bat` / `spot_check.do` / `ex.do` / `theory_check.py` | 保留 |
   | `const_img/` | 8.5 常数图实验的 TB 副本（`img[k]=8'd128`） | 看完可删 |
   | `nomask/` | 反证实验：把地址掩码撤掉 → 复现"全 X"，证明第 5 步那处改动是必需的 | 看完可删 |
   | `nowriteb/` | 反证实验：把 `WRITE_MODE_B` 撤掉 → 复现 6 条 `vopt-2732`，证明第 4 步那处改动是必需的 | 看完可删 |
   | `alg1_q/` | Questa 2024.1 跑同一套文件的目录 | 看完可删 |
   | `kg/` | 重构**前**那份参数化版本的快照（`sbm_gauss_v #(.IMG_W(...))` + 老版 `.v` TB），当时唯一的障碍也是坑 B | 想 bisect"动态尺寸重构改了什么"才留 |
   | `head_check/` | 早期 HEAD 校验目录（日志已被本文引用过） | 可删；本机删除时被"目录被占用"挡住过，关掉占用的 vsim/GUI 再删 |

   注：这台机器的执行策略不允许我做递归删除，所以上面这些实验目录还留在盘上，请手动清。

---

## 12. 一分钟复述（给自己复习用）

- 编译过 ≠ 能跑：端口/参数错误在 `vsim` 阶段才炸。
- 全 X 先查**位宽不匹配的存储地址**，`-debugdb` + `examine` 两分钟就能定死。
- 换仿真器结果不同时，先怀疑**测试夹具**（行为级替身），再怀疑 RTL。
- "和理论一致"要三层证据：TB golden、独立 NumPy 重推、从 DUT 输出流直接取样；三者互相独立才有意义。
- 硬件是**两级量化**，拿 OpenCV 单次量化对标会得到 17% 的 ±1"假错"。
