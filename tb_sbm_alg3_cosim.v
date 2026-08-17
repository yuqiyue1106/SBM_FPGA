// ==================================================================
// tb_sbm_alg3_cosim.v : alg3(F3) 强梯度门对齐联仿
// 关键观察：强梯度门 strong_sh[STRONG_DLY-1] 必须与数据通路已对齐的
//   strong_s5(best_dir_r 同源) 相等，输出才与投票方向同源像素一致。
//   两路均经同一 flush 拍，相对对齐只由常数 STRONG_DLY 决定，与行/帧
//   补拍无关 -> 本校验对 flush 免疫。
// 校验：每个发射像素，输出非零 == strong_s5 && 非边界(DUT 边界)。
//   angle 恒 0 -> 投票恒 9(>=5) -> 方向恒通过，输出非零只由强门决定。
// 用 `define STRONG_DLY_VAL 覆盖延迟深度做扫描，找 err=0 的值。
// ==================================================================
`timescale 1ns/1ps
`ifndef STRONG_DLY_VAL
`define STRONG_DLY_VAL 3
`endif

module tb_sbm_alg3_cosim;
parameter IMG_W = 30;
parameter IMG_H = 20;

reg clk, rst_n;
reg s_axis_tvalid;
reg [21:0] s_axis_mag2;
reg [15:0] s_axis_angle;
reg s_axis_tuser, s_axis_tlast;
wire m_axis_tvalid;
wire [7:0] m_axis_tdata;

integer rr, cc;
integer err, tot;
reg g;

always #5 clk = ~clk;

task send_frame;
begin
	for (rr=0; rr<IMG_H; rr=rr+1) for (cc=0; cc<IMG_W; cc=cc+1) begin
		@(posedge clk);
		s_axis_tvalid <= 1'b1;
		s_axis_mag2   <= (((rr+cc)%2)==0) ? 22'd1024 : 22'd0;   // 棋盘强梯度
		s_axis_angle  <= 16'd0;                                 // 恒 0 -> 方向恒通过
		s_axis_tuser  <= (rr==0 && cc==0);
		s_axis_tlast  <= (rr==IMG_H-1 && cc==IMG_W-1);
	end
	@(posedge clk);
	s_axis_tvalid <= 1'b0;
end
endtask

initial begin
	clk=0; rst_n=0; s_axis_tvalid=0; s_axis_mag2=0; s_axis_angle=0;
	s_axis_tuser=0; s_axis_tlast=0; err=0; tot=0;
	#100 rst_n=1;
	send_frame;
	repeat (IMG_W*IMG_H + 120) @(posedge clk);
	$display("STRONG_DLY=%0d total=%0d err=%0d", `STRONG_DLY_VAL, tot, err);
	if (err==0) $display("RESULT: PASS (alg3 F3 strong-gate == data-path strong, %0d px checked)", tot);
	else        $display("RESULT: FAIL (alg3 F3 strong-gate align, err=%0d)", err);
	$finish;
end

// 纯 F3 对齐校验(对 flush 免疫)：强门 strong_sh[STRONG_DLY-1] 须等于
//   数据通路已对齐的 strong_s5(与 best_dir_r 同源)。每个 vld_s5 周期比较。
always @(posedge clk) begin
	if (dut.vld_s5) begin
		tot <= tot + 1;
		if (dut.strong_sh[`STRONG_DLY_VAL-1] !== dut.strong_s5) err <= err + 1;
	end
end

sbm_alg3_quantize #(.IMG_W(IMG_W), .IMG_H(IMG_H),
	.STRONG_DLY(`STRONG_DLY_VAL)) dut (
	.clk(clk), .rst_n(rst_n),
	.s_axis_tvalid(s_axis_tvalid), .s_axis_tready(),
	.s_axis_mag2(s_axis_mag2), .s_axis_angle(s_axis_angle),
	.s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast),
	.m_axis_tvalid(m_axis_tvalid), .m_axis_tready(1'b1),
	.m_axis_tdata(m_axis_tdata), .m_axis_tuser(), .m_axis_tlast());
endmodule
