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
//   ⑥【N-14】w_in_valid 补入 s_axis_tuser 项：原版帧首拍被 row_active=0
//   门控丢弃，整帧左移1列，行尾补拍触发条件(s_col==IMG_W-1)错拍命中，
//   量化方向整体错列（C-golden TB3 逐拍比对捕获）。
//   ⑦【N-19】行尾补拍硬件强制：原版补拍依赖“上游行间≥1拍空闲”软契约，
//   照搬 alg2/alg8 P2 修法：补拍期间拉低 s_axis_tready，并将 tready 纳入
//   外部数据消费/触发条件（AXI-S 握手语义）。
//   ⑧【N-24】输出帧计数器帧间不复位：out_row 每帧末递增为 IMG_H 后
//   不回卷，第二帧起 tuser 帧首标记丢失（C-golden TB3 双帧协议校验捕获）。
//   ⑨【N-25】flush_row 清除沿吞掉补行换行拍：原版补行的换行拍(c=IMG_W，
//   发射末行末列输出)与 flush_row 清除同拍重叠，清除后 flush_row=0 使
//   该拍窗口标签丢失、末行末列输出永缺（C-golden TB3 捕获）。修法：
//   补拍(flush_c≠0)期间不清除，延迟一拍后换行拍正常生成。
//   ⑩【N-26】坐标标签链与数据路径错位 2 拍：数据路径 consume→out_r 共
//   5 级(s1→win→votes→best→out)，而标签链 s_row_w→s_row_w2 只随路 3 级，
//   丢弃/边界判定用的是 +2 拍后的标签：每行 c=0 拍不再被丢弃、帧首行
//   丢弃窗口偏移、帧尾多发射 H-1 拍（C-golden TB3 捕获：got=W*(H+1)-1、
//   背景区方向错）。修法：标签链补 2 级延迟(s_row_w3/w4)对齐 vld_out；
//   并补注册 best_votes(原版级5 直用级3 组合 votes，同错位)。
//   ⑪【N-27】强梯度门源错位：注释②声称幅值门控作用于输出像素(r-1,c-1)，
//   但 strong_s1→strong_s5 链只随数据路径同拍传递，级5 实际门控的是
//   消费像素(r,c)的 strong（比中心像素多 1 行 1 列）：块边界±1 行/列
//   强弱反转（C-golden TB3 捕获：块边缘对角错、弱中心出强输出）。
//   修法：strong 随标签写入行缓冲(3bit→4bit 高位=strong)，消费拍当拍
//   lb 读出口即中心像素(r-1,c-1)的 strong，再寄存 4 拍与 best_dir_r 同源。
//   原 STRONG_DLY 固定深度移位链无法实现跨行延迟，已移除。
// ==================================================================
`include "sbm_geometry.vh"
module sbm_alg3_quantize #(
	parameter IMG_W = `SBM_L0_W,
	parameter IMG_H = `SBM_L0_H,
	parameter STRONG_DLY = 3   // 【N-27 后废弃】原强梯度门延迟深度；strong 门已改由
	                          // 行缓冲回读中心像素实现（见下方 sc_d 链），本参数
	                          // 仅为保持例化接口兼容而保留，内部不再使用。
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
// ---------- 对齐流行列跟踪（s_row:0..IMG_H含帧尾补1行; s_col:0..IMG_W含行尾补1拍） ----------
reg [12:0] s_row, s_col;
reg        row_active;
reg [1:0]  flush_c;               // 行尾补1拍计数
reg        flush_row;             // 帧尾补1行标记
reg [2:0]  last_label;            // 行末标签（右边界复制源）
// 【N-19】补拍期间暂停接收，硬件强制行间隙（照搬 alg8 P2 / alg2 修法）
assign s_axis_tready = (flush_c == 2'd0);
// 【N-14】补入 s_axis_tuser 项：帧首拍 row_active=0 但必须消费
wire w_in_valid = (s_axis_tvalid && s_axis_tready && (row_active || s_axis_tuser))
               || (flush_c != 2'd0) || flush_row;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		s_row <= 13'd0; s_col <= 13'd0; row_active <= 1'b0;
		flush_c <= 2'd0; flush_row <= 1'b0; last_label <= 3'd0;
	end else begin
		if (s_axis_tvalid && s_axis_tready && s_axis_tuser) begin
			row_active <= 1'b1; flush_c <= 2'd0; flush_row <= 1'b0;
			s_row <= 13'd0; s_col <= 13'd0;
		end
		if (s_axis_tvalid && s_axis_tready && s_axis_tlast) row_active <= 1'b0;
		if (s_axis_tvalid && s_axis_tready && row_active && (s_col == IMG_W-1)) begin
			flush_c <= 2'd1;
			if (s_row == IMG_H-1) flush_row <= 1'b1;
		end else if (flush_c != 2'd0)
			flush_c <= flush_c - 2'd1;
		// 【N-25】flush_row 清除延迟一拍（补拍 flush_c≠0 时不清除）：原版
		// 清除沿与补行的换行拍(c=IMG_W)同拍重叠，该窗口拍被吞，末行末列
		// 输出丢失且帧尾状态残留（C-golden TB3 捕获：got=W*(H+1)-1）。
		// 延迟后换行拍正常生成 tag(H,W)，随后一拍幻影 beat(0,0) 被丢弃
		// 条件吸收，不产生多余输出。
		if (flush_row && w_in_valid && (s_col == IMG_W) && (s_row == IMG_H)
		    && (flush_c == 2'd0)) begin
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
		if (s_axis_tvalid && s_axis_tready) last_label <= w_border ? 3'd0 : w_label_in;   // 锁存行末标签
	end
end
// ---------- 2个行缓冲（4bit={strong,label}，按行号mod2轮转；写地址=延迟1拍的列，读地址=当前列） ----------
// 读时序：addrb(t)=s_col(t)=c → doutb(t+1)=行r-1/r-2的c列，与label_s1(t+1)=标签(r,c)
// 同拍对齐；写时序：addra(t+1)=s_col(t)=c、dina=label_s1(t+1)=标签(r,c)
// 【N-27】高位 strong 随行缓存：doutb(t)=行r-1 列 c-1 的 strong 恰为
// 消费拍 t 的中心像素(r-1,c-1)强梯度位（供 sc_d 链）。
reg [12:0] s_col_d;
reg [12:0] s_row_d;              // 对齐流像素行列（延迟1拍，与窗口数据对齐）
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin s_col_d <= 13'd0; s_row_d <= 13'd0; end
	else begin
		s_col_d <= s_col;
		s_row_d <= s_row;
	end
end
wire [3:0] lb1_out, lb2_out;
wire       w_we0 = vld_s1 && !flush_row && (s_row[0] == 1'b0);
wire       w_we1 = vld_s1 && !flush_row && (s_row[0] == 1'b1);
xpm_memory_sdpram #(
	.MEMORY_SIZE(IMG_W*4), .WRITE_DATA_WIDTH_A(4),
	.READ_DATA_WIDTH_B(4), .READ_LATENCY_B(1),
	.WRITE_MODE_A("read_first"), .MEMORY_PRIMITIVE("auto")
) u_buf0 (
	.clka(clk), .ena(1'b1), .wea(w_we0), .addra(s_col_d),
	.dina({strong_s1, label_s1}),
	.clkb(clk), .enb(1'b1), .addrb(s_col), .doutb(lb1_out)
);
xpm_memory_sdpram #(
	.MEMORY_SIZE(IMG_W*4), .WRITE_DATA_WIDTH_A(4),
	.READ_DATA_WIDTH_B(4), .READ_LATENCY_B(1),
	.WRITE_MODE_A("read_first"), .MEMORY_PRIMITIVE("auto")
) u_buf1 (
	.clka(clk), .ena(1'b1), .wea(w_we1), .addra(s_col_d),
	.dina({strong_s1, label_s1}),
	.clkb(clk), .enb(1'b1), .addrb(s_col), .doutb(lb2_out)
);
// ---------- 级2: 3×3标签窗口（行首3列整体装载=左边界复制） ----------
// 行r-1读缓冲(r-1)%2=~r2；行r-2读缓冲r%2=r2(旧内容)；r2=s_row[0]
wire [2:0] w_row_p1 = (s_row[0] == 1'b0) ? lb2_out[2:0] : lb1_out[2:0];   // 行r-1
wire [2:0] w_row_p2 = (s_row[0] == 1'b0) ? lb1_out[2:0] : lb2_out[2:0];   // 行r-2
// 【N-27】消费拍当拍的 lb 读出口 = 行r-1 列c-1（上一拍 addrb）：
// 即中心像素(r-1,c-1)的 strong
wire       w_strong_ctr = (s_row[0] == 1'b0) ? lb2_out[3] : lb1_out[3];
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
reg [3:0]  best_votes_r;          // 【N-26】原版级5 直用组合 votes(错 2 拍)
reg        strong_s5, vld_s5;
reg [12:0] s_row_w2, s_col_w2;
reg [12:0] s_row_w3, s_col_w3;    // 【N-26】标签链补齐 2 级延迟
reg [12:0] s_row_w4, s_col_w4;    //     对齐 vld_out(数据路径共 5 级)
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin best_dir_r <= 3'd0; best_votes_r <= 4'd0;
		strong_s5 <= 1'b0; vld_s5 <= 1'b0;
		s_row_w2 <= 13'd0; s_col_w2 <= 13'd0;
		s_row_w3 <= 13'd0; s_col_w3 <= 13'd0;
		s_row_w4 <= 13'd0; s_col_w4 <= 13'd0; end
	else begin
		best_dir_r <= best_dir; best_votes_r <= best_votes;
		strong_s5 <= strong_s4; vld_s5 <= vld_s4;
		s_row_w2 <= s_row_w;  s_col_w2 <= s_col_w;
		s_row_w3 <= s_row_w2; s_col_w3 <= s_col_w2;
		s_row_w4 <= s_row_w3; s_col_w4 <= s_col_w3;
	end
end
// ---------- 强梯度门对齐（中心像素） ----------
// 【N-27】消费拍 c（窗口像素(r,c)）当拍的 lb 读出口 = 行 r-1 列 c-1
//   = 中心像素(r-1,c-1)的 strong。寄存 4 拍：sc_d1(c+1)..sc_d4(c+4)，
//   与 best_dir_r/best_votes_r(c+4) 同源，级5 于 c+4 沿锁存时采样。
//   帧内拍背靠背，链节拍与消费拍严格对齐；边界输出的中心 strong 来源
//   越界/陈旧值无害（输出被 w_border_out 强制 0）。
reg [3:0] sc_d;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) sc_d <= 4'd0;
	else        sc_d <= {sc_d[2:0], w_strong_ctr};
end
// ---------- 级5: 双条件门控输出单热编码 ----------
// 输出像素=(r-1,c-1)：边界判定 = r==1 || r==IMG_H || c==1 || c==IMG_W
// 【N-26】out_r 在 c+4 沿锁存，标签取当拍(c+4)值 s_row_w3(=consume c)；
// w_emit 为 c+5 拍组合判定，用 s_row_w4（见下）。
wire w_border_out = (s_row_w3 == 13'd1) || (s_row_w3 == IMG_H)
	|| (s_col_w3 == 13'd1) || (s_col_w3 == IMG_W);
reg [7:0]  out_r;
reg        vld_out;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin out_r <= 8'd0; vld_out <= 1'b0; end
	else begin
		if (sc_d[3] && !w_border_out && (best_votes_r >= 4'd5))
			out_r <= 8'b1 << best_dir_r;
		else
			out_r <= 8'd0;
		vld_out <= vld_s5;
	end
end
// ---------- 输出发射：丢弃第0行与每行第0列 ----------
// 【N-26】丢弃判定用对齐后的标签 s_row_w4/s_col_w4
wire w_emit = vld_out && (s_row_w4 >= 13'd1) && (s_col_w4 >= 13'd1);
reg [12:0] out_row, out_pix;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin out_row <= 13'd0; out_pix <= 13'd0; end
	else if (w_emit) begin
		if (out_pix == IMG_W-1) begin
			out_pix <= 13'd0;
			// 【N-24】帧末像素后行计数回卷归零，保证下一帧 tuser 命中
			if (out_row == IMG_H-1) out_row <= 13'd0;
			else                    out_row <= out_row + 13'd1;
		end
		else                    out_pix <= out_pix + 13'd1;
	end
end
assign m_axis_tvalid = w_emit;
assign m_axis_tdata  = out_r;
assign m_axis_tuser  = w_emit && (out_row == 13'd0) && (out_pix == 13'd0);
assign m_axis_tlast  = w_emit && (out_pix == IMG_W-1);
endmodule
