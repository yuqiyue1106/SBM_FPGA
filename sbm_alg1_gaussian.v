// ==================================================================
// sbm_alg1_gaussian.v : 高斯平滑IP核顶层（AXI4-Stream封装）
// 组合 sbm_gauss_h + sbm_gauss_v，输出带tuser/tlast标记
//
// N-8 整改（统一 AXI4-Stream 帧协议：tuser=帧首拍、tlast=帧末拍）：
//   ① 删除旧版 row_active 对输入数据消费的门控（N-8a：帧首拍被握手
//     消费却因 row_active 寄存延迟丢失，每行第 0 像素被吞）。数据消费
//     仅取决于 AXI-S 握手（s_axis_tvalid && s_axis_tready）。
//   ② 行边界由顶层自由列计数器 pix_cnt 自行派生（与 alg2/alg3 的
//     s_col 风格对齐），不依赖外部行同步信号；行首标记 = 握手成立且
//     pix_cnt==0。
//   ③ 帧首拍（tuser）显式清零计数器与冲刷状态，帧间无状态残留；
//     帧末拍（tlast）置位底部冲刷状态。
//   ④ gauss_h 行尾补 3 拍经 o_ready 硬件强制行间隙（N-8b），顶层据此
//     联合拉低 s_axis_tready，背靠背满速送数亦不吞补零拍。
//   ⑤ 底部冲刷期间（busy_flush）拉低 s_axis_tready，帧间强制 ≥3 行
//     间隙（底部复制排空），无需上游软约定。
// 吞吐: 1 像素/时钟；每帧输出 IMG_W×IMG_H 个有效像素，行间无气泡。
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
	input  wire       s_axis_tuser,    ///< 帧首拍（每帧第一个像素同拍为 1）
	input  wire       s_axis_tlast,    ///< 帧末拍（每帧最后一个像素同拍为 1）
	output wire       m_axis_tvalid,
	input  wire       m_axis_tready,
	output wire [7:0] m_axis_tdata,
	output wire       m_axis_tuser,    ///< 输出帧首拍
	output wire       m_axis_tlast     ///< 输出行尾拍（每行末拍）
);
// ==================== 自由列/行计数器（行边界内部派生） ====================
// 仅在握手成立（被消费）时推进；tuser 帧首拍清零，天然帧间无状态残留。
reg [12:0] pix_cnt;   ///< 0..IMG_W-1（行内列计数）
reg [12:0] row_cnt;   ///< 0..IMG_H-1（帧内行计数，防御性回绕）
// 底部冲刷状态：帧末拍（tlast）置位；本帧输出排空后清零（反压新帧）
reg        busy_flush;
// 末行电平指示（gauss_v.i_frame_end）：末行被消费期间置位，
// 覆盖末行像素穿越水平级的延迟窗口；下一帧 tuser 清零
reg        frame_last_row;
// 输出坐标跟踪（tuser/tlast 重建与 busy_flush 清除判定）
reg [12:0] out_row, out_pix;

// 输入消费 = 纯握手（不被任何行激活窗口门控，N-8a 修复）
wire w_consume = s_axis_tvalid && s_axis_tready;
// N-9 修复（TB 逐拍探针定位）：行首标记必须用寄存单拍脉冲，不能由组合
//   列计数器当场判定——pix_cnt 在 posedge 才更新，组合判据
//   (pix_cnt==0) 会在行首两拍连续为 1（第 2 拍仍读到旧值 0），
//   2 拍宽脉冲把下游 gauss_v 的行同步也带偏一拍，全图错位。
//   改为：消费行末列（或 tuser 帧首拍）时预置 row_start_r，行首拍输出
//   1，下一拍起自动清 0，脉宽恒 1 拍。
reg row_start_r;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n)
		row_start_r <= 1'b1;          // 复位后第一帧首拍即行首
	else if (w_consume)
		row_start_r <= s_axis_tuser || (pix_cnt == IMG_W-1);
end
wire w_row_start = w_consume && row_start_r;

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		pix_cnt <= 13'd0;
		row_cnt <= 13'd0;
	end else if (w_consume) begin
		if (s_axis_tuser) begin
			// 帧首拍显式清零：防御上一帧被截断时的状态残留
			pix_cnt <= 13'd0;
			row_cnt <= 13'd0;
		end else if (pix_cnt == IMG_W-1) begin
			pix_cnt <= 13'd0;
			row_cnt <= (row_cnt == IMG_H-1) ? 13'd0 : row_cnt + 13'd1;
		end else
			pix_cnt <= pix_cnt + 13'd1;
	end
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n)
		frame_last_row <= 1'b0;
	else if (w_consume && s_axis_tuser)
		frame_last_row <= 1'b0;   // 新帧起始清零
	else if (w_consume && (row_cnt == IMG_H-1))
		frame_last_row <= 1'b1;   // 末行任一像素被消费即置位（含 tlast 拍）
end

// ==================== 水平级（行尾复制/左边界复制/列丢弃均内置于模块） ====================
wire       h_valid_out;
wire       h_row_start;
wire       h_ready;      ///< 行尾补 3 拍期间为 0（硬件强制行间隙）
wire [7:0] h_data_out;
sbm_gauss_h #(.IMG_W(IMG_W)) u_h (
	.clk(clk), .rst_n(rst_n),
	.i_valid(w_consume),
	.i_row_start(w_row_start),
	.i_data(s_axis_tdata),
	.o_ready(h_ready),
	.o_valid(h_valid_out),
	.o_row_start(h_row_start),
	.o_data(h_data_out)
);
// ==================== 垂直级（上/下边界复制内置于模块） ====================
wire       v_valid_out;
wire [7:0] v_data_out;
sbm_gauss_v #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_v (
	.clk(clk), .rst_n(rst_n),
	.i_valid(h_valid_out),
	.i_row_start(h_row_start),
	.i_frame_end(frame_last_row),
	.i_data(h_data_out),
	.o_valid(v_valid_out),
	.o_data(v_data_out)
);
// ==================== 底部冲刷状态机 ====================
// 帧末拍（tlast）置位；冲刷期间 tready=0 阻塞新帧，
// 直至底部 3 行冲刷排空（本帧最后一个输出像素发射）
always @(posedge clk or negedge rst_n) begin
	if (!rst_n)
		busy_flush <= 1'b0;
	else if (w_consume && s_axis_tlast)
		busy_flush <= 1'b1;   // tlast 置位优先（退化单像素帧同拍带 tuser 时仍启动冲刷）
	else if (w_consume && s_axis_tuser)
		busy_flush <= 1'b0;   // 帧首拍防御性清零（与计数器清零同语义）
	else if (busy_flush && v_valid_out && (out_row == IMG_H-1) && (out_pix == IMG_W-1))
		busy_flush <= 1'b0;
end
// ==================== 输出坐标跟踪（tuser/tlast 重建） ====================
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		out_row <= 13'd0; out_pix <= 13'd0;
	end else if (v_valid_out) begin
		if (out_pix == IMG_W-1) begin
			out_pix <= 13'd0;
			out_row <= (out_row == IMG_H-1) ? 13'd0 : out_row + 13'd1;
		end else
			out_pix <= out_pix + 13'd1;
	end
end
// 反压：底部冲刷期不接受新帧；水平级补零期不接受新像素（N-8b）
assign s_axis_tready = ~busy_flush && h_ready;
assign m_axis_tvalid = v_valid_out;
assign m_axis_tdata  = v_data_out;
assign m_axis_tuser  = v_valid_out && (out_row == 13'd0) && (out_pix == 13'd0);
assign m_axis_tlast  = v_valid_out && (out_pix == IMG_W-1);
// 输出吞吐恒定，m_axis_tready 仅透传（当前不反压流水线，端口保留）
endmodule
