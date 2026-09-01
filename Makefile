# ============================================================================
# Co-simulation for the SBM FPGA pipeline (F1 / F4 / F5a / F6 / F9)
# using the OSS iverilog + vvp toolchain (no Vivado needed).
#
#  * F1  (sbm_accum_lane) scan coordinate/score misalignment  -> run_fix
#  * F4  (sbm_alg9_lut)   response-map layout on DDR            -> run_alg9 / run_alg9_fail
#  * F5a (参数可维护性)    多分辨率联仿: 64x64 / 120x64 / 64x40 / 512x512
#                         非退化 LFSR 图样 + x 值检测(旧 TB 假通过修复)
#  * F6  (sbm_alg11_accum) 并发扫描/归并 + 背压 + Top-32        -> run_alg11
#  * F9  (alg11 端到端)    黄金模型(越界过滤 + top-32 排序)比对
#  * P0-1 (alg9 连续帧)    两帧连续输入, 复现帧首 bank0 竞争      -> run_alg9_cont
#  * P0-2 (AXI 8B 对齐)    几何 16T 对齐 + 每突发首拍 awaddr 断言  -> run_alg9 / run_geom
#  * P0-3 (alg8 背压)      下游反压 + 逐帧 tuser + 黄金比对      -> run_alg8
#  * 几何单一真源自检                                          -> run_geom
#  * N-8 前端协议闭环: alg1→alg3 C-golden 功能 TB (逐行逐字节比对,
#    tuser/tlast 协议校验, 行门控/补拍吞没/帧间残留即 FAIL):
#      alg1 两级高斯     -> run_alg1
#      alg2 Sobel+CORDIC -> run_alg2
#      alg3 量化投票     -> run_alg3
#
# Xilinx XPM RAM/FIFO primitives are substituted by behavioral models:
#   xpm_memory_spram_beh.v  (SPRAM, read_first / latency 1)
#   xpm_memory_sdpram_beh.v (SDPRAM, read_first / latency 1)
#   xpm_fifo_async_beh.v    (async FIFO, distributed / fwft)
#
# Required: iverilog, vvp   (e.g.  brew install icarus-verilog)
# ============================================================================

IV      ?= iverilog
VVP     ?= vvp

.PHONY: all run_fix run_alg9 run_alg9_multi run_alg11 run_geom run_alg9_cont run_alg8 \
        run_alg1 run_alg2 run_alg3 clean

all: run_fix run_alg9 run_alg9_multi run_alg11 run_alg9_cont run_alg8 run_geom \
     run_alg1 run_alg2 run_alg3

# --- 黄金数据再生成: golden_sbm.c 一次产出 stimulus.hex/expected_pos.txt/hits.txt ---
golden_sbm: golden_sbm.c
	$(CC) -O2 -o $@ $<

stimulus.hex: golden_sbm
	./golden_sbm

expected_pos.txt: stimulus.hex
hits.txt: stimulus.hex

# --- F1: lane 候选扫描联仿 -> 期望 PASS (7 FIFO hits matched) -----------------
sim_fix.out: tb_sbm_accum_lane_cosim.v sbm_accum_lane.v xpm_memory_spram_beh.v stimulus.hex expected_pos.txt hits.txt
	$(IV) -g2012 -o $@ \
		tb_sbm_accum_lane_cosim.v sbm_accum_lane.v xpm_memory_spram_beh.v

run_fix: sim_fix.out
	$(VVP) $<

# --- F4: alg9 落盘布局联仿(64x64, 加固版 TB) -> 期望 PASS --------------------
sim_alg9.out: tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v similarity_lut.mem
	$(IV) -g2012 -o $@ \
		tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v

run_alg9: sim_alg9.out
	$(VVP) $<

# --- F5a: alg9 多分辨率(128x64 / 64x40 非方 / 512x512) -------
#    任意分辨率下: ceil(SEG_BEATS)、FIFO 指针取模、节流不变量 全部生效
#    (WC%8==0 约束: IMG_W 须为 64 的整数倍, 否则 DUT 编译期断言报错)
sim_alg9_m.out:
	$(IV) -g2012 -P tb_sbm_alg9_cosim.IMG_W=128 -P tb_sbm_alg9_cosim.IMG_H=64 -o $@ \
		tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v
sim_alg9_m2.out:
	$(IV) -g2012 -P tb_sbm_alg9_cosim.IMG_W=64 -P tb_sbm_alg9_cosim.IMG_H=40 -o $@ \
		tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v
sim_alg9_m3.out:
	$(IV) -g2012 -P tb_sbm_alg9_cosim.IMG_W=512 -P tb_sbm_alg9_cosim.IMG_H=512 -o $@ \
		tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v

run_alg9_multi: sim_alg9_m.out sim_alg9_m2.out sim_alg9_m3.out
	$(VVP) sim_alg9_m.out
	$(VVP) sim_alg9_m2.out
	$(VVP) sim_alg9_m3.out

# --- P0-1 回归: alg9 连续两帧(第二帧不等 irq_done) -> 期望 PASS -------------
sim_alg9_cont.out: tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v similarity_lut.mem
	$(IV) -g2012 -DALG9_CONT_FRAMES -o $@ \
		tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v

run_alg9_cont: sim_alg9_cont.out
	$(VVP) $<

# --- P0-3 回归: alg8 背压(stall) + 逐帧 tuser + 黄金比对 -> 期望 PASS --------
sim_alg8.out: tb_sbm_alg8_spread.v sbm_alg8_spread.v xpm_memory_sdpram_beh.v
	$(IV) -g2012 -o $@ \
		tb_sbm_alg8_spread.v sbm_alg8_spread.v xpm_memory_sdpram_beh.v

run_alg8: sim_alg8.out
	$(VVP) $<

# --- F6+F9: alg11 端到端(黄金模型比对 + 并发drain + 背压 + Top-32) ------------
sim_alg11.out: tb_sbm_alg11_accum.v sbm_alg11_accum.v sbm_accum_lane.v \
		xpm_memory_spram_beh.v xpm_fifo_async_beh.v
	$(IV) -g2012 -o $@ \
		tb_sbm_alg11_accum.v sbm_alg11_accum.v sbm_accum_lane.v \
		xpm_memory_spram_beh.v xpm_fifo_async_beh.v

run_alg11: sim_alg11.out
	$(VVP) $<

# --- F4 FAIL 演示: band7 边界差一(仅 7 band 落盘) -> 期望 FAIL -----------------
sim_alg9_fail.out: tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v similarity_lut.mem
	$(IV) -g2012 -DALG9_BAND7_BUG -o $@ \
		tb_sbm_alg9_cosim.v sbm_alg9_lut.v xpm_memory_sdpram_beh.v

run_alg9_fail: sim_alg9_fail.out
	$(VVP) $<

# --- 几何单一真源自检: 默认 5000x5000 + 覆盖模拟换相机 --------------------------
sim_geom.out: tb_sbm_geom_check.v sbm_geometry.vh
	$(IV) -g2012 -o $@ tb_sbm_geom_check.v

run_geom: sim_geom.out
	$(VVP) $<

# --- N-8 前端协议闭环: alg1 两级高斯 C-golden TB ------------------------------
#  tb 内部 `include sbm_alg1_gaussian.v; 逐行逐字节比对 + tuser/tlast 校验,
#  任意像素/标记不符 -> FAIL (grep "^PASS" 把关, 无 PASS 即非零退出)
sim_alg1.out: tb_sbm_alg1_gaussian.sv sbm_alg1_gaussian.v sbm_gauss_h.v sbm_gauss_v.v \
		xpm_memory_sdpram_beh.v sbm_geometry.vh
	$(IV) -g2012 -o $@ \
		tb_sbm_alg1_gaussian.sv sbm_gauss_h.v sbm_gauss_v.v xpm_memory_sdpram_beh.v

run_alg1: sim_alg1.out
	$(VVP) $< | tee sim_alg1.log
	@grep -q '^PASS' sim_alg1.log

# --- N-8 前端协议闭环: alg2 Sobel+CORDIC C-golden TB --------------------------
sim_alg2.out: tb_sbm_alg2_sobel.v sbm_alg2_sobel.v cordic_atan2_beh.v cordic_atan2_func.vh \
		xpm_memory_sdpram_beh.v sbm_geometry.vh
	$(IV) -g2012 -o $@ \
		tb_sbm_alg2_sobel.v cordic_atan2_beh.v xpm_memory_sdpram_beh.v

run_alg2: sim_alg2.out
	$(VVP) $< | tee sim_alg2.log
	@grep -q '^PASS' sim_alg2.log

# --- N-8 前端协议闭环: alg3 量化投票 C-golden TB ------------------------------
sim_alg3.out: tb_sbm_alg3_quantize.v sbm_alg3_quantize.v xpm_memory_sdpram_beh.v sbm_geometry.vh
	$(IV) -g2012 -o $@ \
		tb_sbm_alg3_quantize.v xpm_memory_sdpram_beh.v

run_alg3: sim_alg3.out
	$(VVP) $< | tee sim_alg3.log
	@grep -q '^PASS' sim_alg3.log

clean:
	rm -f sim_fix.out sim_alg9.out sim_alg9_m.out sim_alg9_m2.out sim_alg9_m3.out \
	      sim_alg11.out sim_alg9_fail.out sim_geom.out sim_alg9_cont.out sim_alg8.out \
	      sim_alg1.out sim_alg2.out sim_alg3.out sim_alg1.log sim_alg2.log sim_alg3.log

distclean: clean
	rm -f golden_sbm stimulus.hex expected_pos.txt hits.txt
