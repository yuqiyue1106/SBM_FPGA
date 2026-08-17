// ==================================================================
// tb_sbm_geom_check.v : 几何参数单一真源自检
// 目的: 验证"改相机分辨率只需改 sbm_geometry.vh 一处, 全链路派生量自动跟随,
//       无静默溢出/容量不足/地址错位"。
// 可用 iverilog -D 覆盖宏模拟换相机:
//   iverilog -DSBM_CAM_W=1920 -DSBM_CAM_H=1080 ...
//   iverilog -DSBM_CAM_W=4096 -DSBM_CAM_H=4096 ...
// 检查项:
//   1) 级0/级1 对齐链: L0%(16T)==0, IMG_W%T==0, IMG_H%T==0
//   1b) AXI 64bit 写对齐: WC%8==0(否则 awaddr 非 8B 对齐, 协议违规)
//   2) CELLS == WC*HC
//   3) alg11 通道容量: LANES*CHUNK >= CELLS(任意模板尺寸不丢位置)
//   4) BANK_DEPTH >= ceil(CHUNK/2) 且为 2 的幂
//   5) 各派生位宽能容纳各自值域(SBM_W 宏保护 0 宽度)
//   6) alg9 SEG_BEATS <= 256(AXI awlen 上限)
//   7) 响应图 DDR 区间不超 4GB
// ==================================================================
`include "sbm_geometry.vh"
module tb_sbm_geom_check;

localparam WC    = `SBM_WC;
localparam HC    = `SBM_HC;
localparam CELLS = `SBM_CELLS;
localparam TP_MAX = `SBM_TP_MAX;
localparam LANES = `SBM_LANES;
localparam T     = `SBM_T;
localparam CHUNK = (TP_MAX + LANES - 1) / LANES;
localparam BANK_DEPTH = 1 << $clog2((CHUNK + 1) / 2);
localparam SEG_BEATS = (WC + 7) / 8;

localparam PW  = `SBM_W(`SBM_IMG_W * `SBM_IMG_H);
localparam CW  = `SBM_W(`SBM_IMG_W + 1);
localparam RW  = `SBM_W(`SBM_IMG_H + 1);
localparam BAW = `SBM_W(T * `SBM_IMG_W + 1);
localparam HCW = `SBM_W(HC + 1);
localparam DCW = `SBM_W(T * T * WC + 1);
localparam QW  = `SBM_W(32 + 1);
localparam TPW = `SBM_W(TP_MAX + 1);
localparam AW  = `SBM_W(BANK_DEPTH);

integer err;

task chk;
	input ok;
	input [255:0] msg;
begin
	if (!ok) begin
		$display("GEOM FAIL: %0s", msg);
		err = err + 1;
	end
end
endtask

initial begin
	err = 0;
	$display("=== SBM geometry check ===");
	$display("CAM=%0dx%0d T=%0d LANES=%0d", `SBM_CAM_W, `SBM_CAM_H, `SBM_T, `SBM_LANES);
	$display("L0=%0dx%0d  IMG(L1)=%0dx%0d  WC=%0d HC=%0d CELLS=%0d",
	         `SBM_L0_W, `SBM_L0_H, `SBM_IMG_W, `SBM_IMG_H, WC, HC, CELLS);
	$display("TP_MAX=%0d CHUNK=%0d BANK_DEPTH=%0d SEG_BEATS=%0d",
	         TP_MAX, CHUNK, BANK_DEPTH, SEG_BEATS);
	$display("widths: PW=%0d CW=%0d RW=%0d BAW=%0d HCW=%0d DCW=%0d QW=%0d TPW=%0d AW=%0d",
	         PW, CW, RW, BAW, HCW, DCW, QW, TPW, AW);
	$display("DDR LM bytes = %0d (%0d MB)", `SBM_LM_BYTES_TOTAL, `SBM_LM_BYTES_TOTAL/1048576);

	// 1) 对齐链(16T: 级0 对齐保证级1 8T 对齐 → WC 被 8 整除 → AXI 写 8B 对齐)
	chk((`SBM_L0_W % (16 * `SBM_T)) == 0, "L0_W not 16T-aligned");
	chk((`SBM_L0_H % (16 * `SBM_T)) == 0, "L0_H not 16T-aligned");
	chk((`SBM_IMG_W % `SBM_T) == 0, "IMG_W not T-aligned");
	chk((`SBM_IMG_H % `SBM_T) == 0, "IMG_H not T-aligned");
	chk((`SBM_IMG_W == `SBM_L0_W / 2) && (`SBM_IMG_H == `SBM_L0_H / 2), "L1 != L0/2");
	// 1b) AXI 64bit 写地址 8B 对齐(WC 与 CELLS 均为 8 的倍数)
	chk((WC % 8) == 0, "WC not 8-aligned (64bit AXI write addr)");
	// 2) CELLS
	chk(CELLS == WC * HC, "CELLS != WC*HC");
	// 3) 容量
	chk(LANES * CHUNK >= TP_MAX, "LANES*CHUNK < TP_MAX (positions dropped)");
	// 4) bank
	chk(BANK_DEPTH * 2 >= CHUNK, "BANK_DEPTH < ceil(CHUNK/2)");
	chk((BANK_DEPTH & (BANK_DEPTH - 1)) == 0, "BANK_DEPTH not power of 2");
	// 5) 位宽
	chk(PW >= 1, "PW width 0");
	chk(CW >= 1 && RW >= 1 && BAW >= 1 && HCW >= 1 && DCW >= 1, "zero-width counter");
	chk(TPW >= 1, "TPW width 0");
	// 6) SEG_BEATS
	chk(SEG_BEATS <= 256, "SEG_BEATS > 256 (AXI awlen overflow)");
	// 7) DDR 区间
	chk((`SBM_RESP_BASE + `SBM_LM_BYTES_TOTAL) <= 32'hFFFF_FFFF, "LM region exceeds 4GB");
	// 8) 坐标 12bit 打包
	chk(WC <= 4096 && HC <= 4096, "WC/HC exceed 12-bit hit packing");

	if (err == 0) $display("==== GEOM RESULT: PASS ====");
	else          $display("==== GEOM RESULT: FAIL (%0d) ====", err);
	$finish;
end
endmodule
