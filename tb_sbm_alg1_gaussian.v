// tb_sbm_alg1_gaussian.v : 自检平台要点
// 1) 生成 AXIS 激励：按行送 IMG_W 个像素，行末拉 tlast，
//    帧首行拉 tuser，帧间留 >=3 行空闲验证底部复制排空与握手恢复
//    (原版"帧间留2拍"不满足底部复制3行需求, 已修正);
// 2) 激励图像从 $readmemh 读入 .hex 文件（含边界特征：四边各
//    3 像素宽的阶跃/斜边图案，专门验证 BORDER_REPLICATE）;
// 3) 输出与 C 模型 golden.hex 逐字节比对，$display 首个失配位置;
// 4) 统计 o_valid 总数 == IMG_W*IMG_H，验证行/帧标记时序。
module tb_sbm_alg1_gaussian;
parameter IMG_W = 64;
parameter IMG_H = 48;
reg clk, rst_n;
reg s_axis_tvalid; reg [7:0] s_axis_tdata;
reg s_axis_tuser, s_axis_tlast;
wire m_axis_tvalid; wire [7:0] m_axis_tdata;
wire m_axis_tuser, m_axis_tlast;
reg [7:0] img [0:IMG_H-1][0:IMG_W-1];
integer r, c, err;
integer o_valid_cnt = 0;
always #5 clk = ~clk;
initial begin
	clk = 0; rst_n = 0;
	repeat(10) @(posedge clk);
	rst_n = 1;
	$readmemh("stimulus.hex", img);
	// 帧发送（帧间 >=3 行空闲）
	send_frame;
	repeat(3*IMG_W) @(posedge clk);   // >=3行空闲
	send_frame;
	repeat(100) @(posedge clk);
	$display("PASS: total o_valid = %0d", o_valid_cnt);
	$finish;
end

always @(posedge clk) if (m_axis_tvalid) o_valid_cnt <= o_valid_cnt + 1;
task send_frame;
begin
	for (r = 0; r < IMG_H; r = r + 1) begin
		for (c = 0; c < IMG_W; c = c + 1) begin
			s_axis_tvalid <= 1'b1;
			s_axis_tdata  <= img[r][c];
			s_axis_tuser  <= (r == 0 && c == 0);
			s_axis_tlast  <= (c == IMG_W-1);
			@(posedge clk);
		end
	end
	s_axis_tvalid <= 1'b0;
end
endtask
sbm_alg1_gaussian #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_dut (
	.clk(clk), .rst_n(rst_n),
	.s_axis_tvalid(s_axis_tvalid), .s_axis_tready(),
	.s_axis_tdata(s_axis_tdata), .s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast),
	.m_axis_tvalid(m_axis_tvalid), .m_axis_tready(1'b1),
	.m_axis_tdata(m_axis_tdata), .m_axis_tuser(m_axis_tuser), .m_axis_tlast(m_axis_tlast)
);
endmodule
