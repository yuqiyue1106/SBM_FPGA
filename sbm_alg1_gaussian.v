// ==================================================================
// sbm_alg1_gaussian.v : 高斯平滑IP核顶层（AXI4-Stream封装）
// 组合 sbm_gauss_h + sbm_gauss_v，输出带tuser/tlast标记
// 修正记录：原版行首/行尾复制控制位于顶层且valid与data错1拍对齐、
//   底部复制3行未实现；本版行尾复制/左边界复制内置于sbm_gauss_h、
//   底部复制内置于sbm_gauss_v，顶层仅做流握手与行标记重建。
// 帧间约束：两帧之间需预留≥3行空闲（底部复制排空），否则前一帧底部
//   复制行与后一帧首行冲突，本模块以s_axis_tready反压上游保证。
// ==================================================================
`include "sbm_geometry.vh"
module sbm_alg1_gaussian #(
	parameter IMG_W = `SBM_L0_W,
	parameter IMG_H = `SBM_L0_H
)(
	input  wire       clk,
	input  wire       rst_n,
	input  wire       s_axis_tvalid,
	output wire       s_axis_tready,
	input  wire [7:0] s_axis_tdata,
	input  wire       s_axis_tuser,
	input  wire       s_axis_tlast,
	output wire       m_axis_tvalid,
	input  wire       m_axis_tready,
	output wire [7:0] m_axis_tdata,
	output wire       m_axis_tuser,
	output wire       m_axis_tlast
);
// 底部复制排空期标记：帧末(tlast)置1，本帧输出排空后清0（反压新帧）
reg        busy_flush;
// 输出坐标跟踪（tuser/tlast 与 busy_flush 清除判定）
reg [12:0] out_row, out_pix;
// ---------- 输入侧行激活（tuser起、tlast止） ----------
reg        row_active;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		row_active <= 1'b0;
	end else begin
		if (s_axis_tvalid && s_axis_tuser && !busy_flush)
			row_active <= 1'b1;
		if (s_axis_tvalid && s_axis_tlast)
			row_active <= 1'b0;
	end
end
// 反压：底部复制期不接受新帧（帧间≥3行约束的硬件保证）
assign s_axis_tready = ~busy_flush;
// 帧末行指示（底部复制触发源，电平保持到下一帧tuser）
reg frame_end;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) frame_end <= 1'b0;
	else begin
		if (s_axis_tvalid && s_axis_tuser && !busy_flush) frame_end <= 1'b0;
		if (s_axis_tvalid && s_axis_tlast) frame_end <= 1'b1;
	end
end
// 行首标记（输入流）
reg [12:0] pix_cnt;
wire w_row_start = (pix_cnt == 13'd0) && s_axis_tvalid && row_active;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) pix_cnt <= 13'd0;
	else if (s_axis_tvalid && row_active) begin
		if (pix_cnt == IMG_W-1) pix_cnt <= 13'd0;
		else pix_cnt <= pix_cnt + 13'd1;
	end
end
// ---------- 水平级（行尾复制/左边界复制/列丢弃均内置于模块） ----------
wire       h_valid_out;
wire       h_row_start;
wire [7:0] h_data_out;
sbm_gauss_h #(.IMG_W(IMG_W)) u_h (
	.clk(clk), .rst_n(rst_n),
	.i_valid(s_axis_tvalid && row_active),
	.i_row_start(w_row_start),
	.i_data(s_axis_tdata),
	.o_valid(h_valid_out),
	.o_row_start(h_row_start),
	.o_data(h_data_out)
);
// ---------- 垂直级（上/下边界复制内置于模块） ----------
wire       v_valid_out;
wire [7:0] v_data_out;
sbm_gauss_v #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_v (
	.clk(clk), .rst_n(rst_n),
	.i_valid(h_valid_out),
	.i_row_start(h_row_start),
	.i_frame_end(frame_end),
	.i_data(h_data_out),
	.o_valid(v_valid_out),
	.o_data(v_data_out)
);
// 底部复制排空期标记：帧末(tlast)置1，本帧输出排空后清0（反压新帧）
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) busy_flush <= 1'b0;
	else begin
		if (s_axis_tvalid && s_axis_tlast) busy_flush <= 1'b1;
		if (frame_end && v_valid_out && (out_row == IMG_H-1) && (out_pix == IMG_W-1))
			busy_flush <= 1'b0;
	end
end
// 底部复制期标记：末行输入结束(frame_end)至垂直级复制排空
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		out_row <= 13'd0; out_pix <= 13'd0;
	end else begin
		if (v_valid_out) begin
			if (out_pix == IMG_W-1) begin
				out_pix <= 13'd0;
				if (out_row == IMG_H-1) begin
					out_row <= 13'd0;
				end else
					out_row <= out_row + 13'd1;
			end else
				out_pix <= out_pix + 13'd1;
		end
	end
end
assign m_axis_tvalid = v_valid_out;
assign m_axis_tdata  = v_data_out;
assign m_axis_tuser  = v_valid_out && (out_row == 13'd0) && (out_pix == 13'd0);
assign m_axis_tlast  = v_valid_out && (out_pix == IMG_W-1);
endmodule
