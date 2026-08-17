// ==================================================================
// sbm_alg3_quantize.v : 滞后梯度量化(16桶→8方向→3×3投票→单热输出)
// 与line2Dup.cpp hysteresisGradient逐像素对齐:
//   q16 = round(angle*16/360) = (angle_q16+2048)>>12, 桶号=相位高4bit
//   label = q16 & 7; 边界像素label强制0; mag2>900; 票数>=5; 平票取小索引
// 输出: 8bit单热(1<<dir)或0; 边界像素输出0
// 修正记录：①投票窗口由因果[r-2..r]×[c-2..c]改为与C++一致的中心窗口
//   (输出像素=窗口中心(r-1,c-1)，丢弃第0行与每行第0列)；
//   ②幅值门控与边界判定改为作用于输出像素(r-1,c-1)(原版错1行1列)；
//   ③补行尾1拍/帧尾1行(原版缺失,末列/末行输出丢失)；
//   ④行缓冲改为2缓冲按行号mod2轮转+端口B读取(原版链式在读延迟1拍下错位)；
//   ⑤CORDIC归一化相位16bit全圆映射16桶已按有符号回绕校验(见4.2.2节)。
// ==================================================================
`include "sbm_geometry.vh"
module sbm_alg3_quantize #(
	parameter IMG_W = `SBM_L0_W,
	parameter IMG_H = `SBM_L0_H,
	parameter STRONG_DLY = 3   // 强梯度门延迟深度：须使 strong 门与数据通路 strong_s5(=best_dir_r 同源)对齐。
	                          // 标定：strong_s1 在级1寄存后，级2(窗口)于同拍取 strong_s1(0拍延迟)，
	                          //   s3->s4->s5 再加 2 拍，故 s1->s5 总延迟=2，延迟线深度=2+1=3。
	                          //   该值为结构常数，与 IMG_W/IMG_H 无关(原 IMG_W+5 / 审查建议 IMG_W+6 均错：
	                          //   误把行缓冲尺度当成流水线对齐延迟)。
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        s_axis_tvalid,
	output wire        s_axis_tready,
	input  wire [21:0] s_axis_mag2,
	input  wire [15:0] s_axis_angle,
	input  wire        s_axis_tuser,
	input  wire        s_axis_tlast,
	output wire        m_axis_tvalid,
	input  wire        m_axis_tready,
	output wire [7:0]  m_axis_tdata,
	output wire        m_axis_tuser,
	output wire        m_axis_tlast
);
assign s_axis_tready = 1'b1;
// ---------- 对齐流行列跟踪（s_row:0..IMG_H含帧尾补1行; s_col:0..IMG_W含行尾补1拍） ----------
reg [12:0] s_row, s_col;
reg        row_active;
reg [1:0]  flush_c;               // 行尾补1拍计数
reg        flush_row;             // 帧尾补1行标记
reg [2:0]  last_label;            // 行末标签（右边界复制源）
wire w_in_valid = (s_axis_tvalid && row_active) || (flush_c != 2'd0) || flush_row;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		s_row <= 13'd0; s_col <= 13'd0; row_active <= 1'b0;
		flush_c <= 2'd0; flush_row <= 1'b0; last_label <= 3'd0;
	end else begin
		if (s_axis_tvalid && s_axis_tuser) begin
			row_active <= 1'b1; flush_c <= 2'd0; flush_row <= 1'b0;
			s_row <= 13'd0; s_col <= 13'd0;
		end
		if (s_axis_tvalid && s_axis_tlast) row_active <= 1'b0;
		if (s_axis_tvalid && row_active && (s_col == IMG_W-1)) begin
			flush_c <= 2'd1;
			if (s_row == IMG_H-1) flush_row <= 1'b1;
		end else if (flush_c != 2'd0)
			flush_c <= flush_c - 2'd1;
		if (flush_row && w_in_valid && (s_col == IMG_W) && (s_row == IMG_H)) begin
			flush_row <= 1'b0;
		end
		if (w_in_valid) begin
			if (s_col == IMG_W) begin
				s_col <= 13'd0;
				s_row <= (s_row == IMG_H) ? 13'd0 : s_row + 13'd1;
			end else
				s_col <= s_col + 13'd1;
		end
	end
end
// ---------- 级1: 桶号/标签/幅值比较/边界清零 ----------
wire [3:0]  w_q16  = (s_axis_angle + 16'd2048) >> 12;   // 16桶
wire [2:0]  w_label= w_q16[2:0];                        // &7合并8方向
wire        w_strong = (s_axis_mag2 > 22'd900);         // weak_threshold²=900
wire        w_border = (s_col == 13'd0) || (s_col == IMG_W-1)
	|| (s_row == 13'd0) || (s_row == IMG_H-1);
wire [2:0]  w_label_in = (flush_c != 2'd0) ? last_label  // 行尾补1拍：复制末标签
	: (flush_row) ? 3'd0                    // 帧尾补1行：末行标签为0
	: (s_axis_tvalid ? w_label : 3'd0);
reg [2:0]  label_s1;
reg        strong_s1, vld_s1;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin label_s1 <= 3'd0; strong_s1 <= 1'b0; vld_s1 <= 1'b0; end
	else begin
		label_s1  <= w_border ? 3'd0 : w_label_in;   // 边界强制0
		strong_s1 <= w_strong;
		vld_s1    <= w_in_valid;
		if (s_axis_tvalid) last_label <= w_border ? 3'd0 : w_label_in;   // 锁存行末标签
	end
end
// ---------- 2个行缓冲（3bit标签，按行号mod2轮转；写地址=延迟1拍的列，读地址=当前列） ----------
// 读时序：addrb(t)=s_col(t)=c → doutb(t+1)=行r-1/r-2的c列，与label_s1(t+1)=标签(r,c)
// 同拍对齐；写时序：addra(t+1)=s_col(t)=c、dina=label_s1(t+1)=标签(r,c)
reg [12:0] s_col_d;
reg [12:0] s_row_d;              // 对齐流像素行列（延迟1拍，与窗口数据对齐）
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin s_col_d <= 13'd0; s_row_d <= 13'd0; end
	else begin
		s_col_d <= s_col;
		s_row_d <= s_row;
	end
end
wire [2:0] lb1_out, lb2_out;
wire       w_we0 = vld_s1 && !flush_row && (s_row[0] == 1'b0);
wire       w_we1 = vld_s1 && !flush_row && (s_row[0] == 1'b1);
xpm_memory_sdpram #(
	.MEMORY_SIZE(IMG_W*3), .WRITE_DATA_WIDTH_A(3),
	.READ_DATA_WIDTH_B(3), .READ_LATENCY_B(1),
	.WRITE_MODE_A("read_first"), .MEMORY_PRIMITIVE("auto")
) u_buf0 (
	.clka(clk), .ena(1'b1), .wea(w_we0), .addra(s_col_d),
	.dina(label_s1),
	.clkb(clk), .enb(1'b1), .addrb(s_col), .doutb(lb1_out)
);
xpm_memory_sdpram #(
	.MEMORY_SIZE(IMG_W*3), .WRITE_DATA_WIDTH_A(3),
	.READ_DATA_WIDTH_B(3), .READ_LATENCY_B(1),
	.WRITE_MODE_A("read_first"), .MEMORY_PRIMITIVE("auto")
) u_buf1 (
	.clka(clk), .ena(1'b1), .wea(w_we1), .addra(s_col_d),
	.dina(label_s1),
	.clkb(clk), .enb(1'b1), .addrb(s_col), .doutb(lb2_out)
);
// ---------- 级2: 3×3标签窗口（行首3列整体装载=左边界复制） ----------
// 行r-1读缓冲(r-1)%2=~r2；行r-2读缓冲r%2=r2(旧内容)；r2=s_row[0]
wire [2:0] w_row_p1 = (s_row[0] == 1'b0) ? lb2_out : lb1_out;   // 行r-1
wire [2:0] w_row_p2 = (s_row[0] == 1'b0) ? lb1_out : lb2_out;   // 行r-2
reg        row_start_d;         // 行首标记延迟1拍（左边界复制触发）
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) row_start_d <= 1'b0;
	else        row_start_d <= (s_col == 13'd0) && vld_s1;
end
reg [2:0] win [0:2][0:2];
reg       strong_s3, vld_s3;
reg [12:0] s_row_w, s_col_w;    // 窗口输入像素行列（输出像素=r-1,c-1）
integer   r, c;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for (r=0;r<3;r=r+1) for (c=0;c<3;c=c+1) win[r][c] <= 3'd0;
		strong_s3 <= 1'b0; vld_s3 <= 1'b0; s_row_w <= 13'd0; s_col_w <= 13'd0;
	end else if (vld_s1) begin
		if (row_start_d) begin        // 行首：3列整体装载（左边界复制）
			win[0][0] <= w_row_p2; win[0][1] <= w_row_p2; win[0][2] <= w_row_p2;
			win[1][0] <= w_row_p1; win[1][1] <= w_row_p1; win[1][2] <= w_row_p1;
			win[2][0] <= label_s1; win[2][1] <= label_s1; win[2][2] <= label_s1;
		end else begin                // 正常左移
			for (r=0;r<3;r=r+1) begin win[r][0] <= win[r][1]; win[r][1] <= win[r][2]; end
			win[0][2] <= w_row_p2;
			win[1][2] <= w_row_p1;
			win[2][2] <= label_s1;
		end
		strong_s3 <= strong_s1;
		vld_s3 <= 1'b1;
		s_row_w <= s_row_d;             // 对齐流像素的行（窗口=该像素的因果3×3）
		s_col_w <= s_col_d;
	end else
		vld_s3 <= 1'b0;
end
// ---------- 级3: 3×3直方图(9个标签单热展开相加) ----------
reg [3:0] votes [0:7];
reg       strong_s4, vld_s4;
integer   d;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for (d=0;d<8;d=d+1) votes[d] <= 4'd0;
		strong_s4 <= 1'b0; vld_s4 <= 1'b0;
	end else begin
		for (d=0;d<8;d=d+1) begin
			votes[d] <= ((win[0][0]==d)?1'b1:1'b0) + ((win[0][1]==d)?1'b1:1'b0)
				+ ((win[0][2]==d)?1'b1:1'b0) + ((win[1][0]==d)?1'b1:1'b0)
				+ ((win[1][1]==d)?1'b1:1'b0) + ((win[1][2]==d)?1'b1:1'b0)
				+ ((win[2][0]==d)?1'b1:1'b0) + ((win[2][1]==d)?1'b1:1'b0)
				+ ((win[2][2]==d)?1'b1:1'b0);
		end
		strong_s4 <= strong_s3;
		vld_s4 <= vld_s3;
	end
end
// ---------- 级4: 最大票方向(链式严格大于, 平票取小索引) ----------
reg [2:0] best_dir;
reg [3:0] best_votes;
always @(*) begin
	best_dir   = 3'd0;
	best_votes = votes[0];
	for (d=1; d<8; d=d+1) begin
		if (votes[d] > best_votes) begin
			best_dir   = d[2:0];
			best_votes = votes[d];
		end
	end
end
reg [2:0]  best_dir_r;
reg        strong_s5, vld_s5;
reg [12:0] s_row_w2, s_col_w2;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin best_dir_r <= 3'd0; strong_s5 <= 1'b0; vld_s5 <= 1'b0;
		s_row_w2 <= 13'd0; s_col_w2 <= 13'd0; end
	else begin
		best_dir_r <= best_dir; strong_s5 <= strong_s4; vld_s5 <= vld_s4;
		s_row_w2 <= s_row_w; s_col_w2 <= s_col_w;
	end
end
// ---------- 幅值门控对齐：强梯度门须与 best_dir_r 同源像素的 strong 对齐 ----------
// strong_s1(级1) -> 级2窗口于同拍取 strong_s1(0拍) -> strong_s3 -> strong_s4
//   -> strong_s5(与 best_dir_r 同拍)。故 s1->s5 总延迟 = 2 拍，延迟线深度
//   STRONG_DLY = 3 即令 strong_sh[STRONG_DLY-1] == strong_s5(同源)。该值为
//   结构常数，与 IMG_W/IMG_H 无关。原 F3 审查建议 IMG_W+6、原代码 IMG_W+5 均错。
reg [STRONG_DLY-1:0] strong_sh;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) strong_sh <= 0;
	else        strong_sh <= {strong_sh[STRONG_DLY-2:0], strong_s1};
end
// F3 运行时自校验：强门强位必须与数据通路 strong_s5 对齐(同源像素)
reg f3_align_err_d;
always @(posedge clk) begin
	if (!rst_n) f3_align_err_d <= 1'b0;
	else if (vld_s5 && (strong_sh[STRONG_DLY-1] !== strong_s5) && !f3_align_err_d) begin
		$error("F3 ALIGN MISMATCH: strong gate not aligned with data-path strong (STRONG_DLY=%0d wrong)", STRONG_DLY);
		f3_align_err_d <= 1'b1;
	end
end
// ---------- 级5: 双条件门控输出单热编码 ----------
// 输出像素=(r-1,c-1)：边界判定 = r==1 || r==IMG_H || c==1 || c==IMG_W
wire w_border_out = (s_row_w2 == 13'd1) || (s_row_w2 == IMG_H)
	|| (s_col_w2 == 13'd1) || (s_col_w2 == IMG_W);
reg [7:0]  out_r;
reg        vld_out;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin out_r <= 8'd0; vld_out <= 1'b0; end
	else begin
		if (strong_sh[STRONG_DLY-1] && !w_border_out && (best_votes >= 4'd5))
			out_r <= 8'b1 << best_dir_r;
		else
			out_r <= 8'd0;
		vld_out <= vld_s5;
	end
end
// ---------- 输出发射：丢弃第0行与每行第0列 ----------
wire w_emit = vld_out && (s_row_w2 >= 13'd1) && (s_col_w2 >= 13'd1);
reg [12:0] out_row, out_pix;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin out_row <= 13'd0; out_pix <= 13'd0; end
	else if (w_emit) begin
		if (out_pix == IMG_W-1) begin out_pix <= 13'd0; out_row <= out_row + 13'd1; end
		else                    out_pix <= out_pix + 13'd1;
	end
end
assign m_axis_tvalid = w_emit;
assign m_axis_tdata  = out_r;
assign m_axis_tuser  = w_emit && (out_row == 13'd0) && (out_pix == 13'd0);
assign m_axis_tlast  = w_emit && (out_pix == IMG_W-1);
endmodule
