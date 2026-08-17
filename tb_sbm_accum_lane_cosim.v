// ============================================================================
// @file      tb_sbm_accum_lane_cosim.v
// @brief     Minimal self-checking co-sim for sbm_accum_lane (F1 verification)
// @details
//   Drives sbm_accum_lane with a small synthetic linear-memory and validates the
//   candidate-scan result TWO ways:
//     (1) PER-POSITION (primary, definitive for F1): via hierarchical access to
//         the DUT internal scan signals (dut.ev_pos / w_score / ev_x / ev_y /
//         ev_vld) — compares score AND coordinate of EVERY position 0..cnt_l-1
//         against the golden model. This directly proves the score/coordinate
//         alignment is correct (the essence of F1), independent of the FIFO.
//         Guarded by `ifdef F1_SIGNALS (only the FIXED DUT exposes these).
//     (2) FIFO path (secondary): the DUT hit FIFO output is compared in order
//         against the golden hit list. The FIFO is drained AFTER the scan (no
//         pop during scan), so every hit is written before any read begins and
//         the DUT's `!hit_pop` write-gating never drops an entry. Hits are kept
//         <= 7 so the 8-deep FIFO (RTL accepts only hcnt<7 => 7 entries) does
//         not overflow. This check runs on BOTH DUTs and is what shows the
//         ORIGINAL (buggy) DUT FAIL (its stored score/coord are shifted).
//
//   A behavioral XPM model supplies the RAM so the test runs under iverilog
//   without Vivado. The DUT itself is unchanged by this TB (we only READ its
//   internal signals hierarchically).
//
//   Run:
//     # FIXED DUT (per-position + FIFO checks):
//     iverilog -g2012 -DF1_SIGNALS -o sim_fix.out tb_sbm_accum_lane_cosim.v \
//              sbm_accum_lane.v xpm_memory_spram_beh.v && vvp sim_fix.out
//     # ORIGINAL (buggy) DUT (FIFO check only -> should FAIL):
//     iverilog -g2012 -o sim_orig.out tb_sbm_accum_lane_cosim.v \
//              sbm_accum_lane.v.bak_preF1fix xpm_memory_spram_beh.v && vvp sim_orig.out
// ============================================================================
`timescale 1ns/1ps
module tb_sbm_accum_lane_cosim;

	// ---- must match golden_sbm.c ----
	parameter WC        = 16;
	parameter CHUNK     = 40;
	parameter BANK_DEPTH= 32;
	parameter LB        = 0;
	parameter X0        = 0;
	parameter Y0        = 0;
	parameter TP        = 40;
	parameter THRESH    = 10;
	parameter STIM_BYTES= 40;

	reg  clk, rst_n, init;
	reg  [31:0] base_addr;
	reg  [11:0] tp;
	reg  [15:0] thresh;
	reg         zero_en;
	reg  [10:0] zero_addr;
	reg         run_en;
	wire        lane_done;
	wire        need_req;
	wire [31:0] req_addr;
	wire        next_slot;
	reg         grant;
	reg         fill_slot;
	reg         fill_vld;
	reg  [255:0] fill_data;
	reg         scan_en;
	reg         drain_en;   // pop the hit FIFO only during the post-scan drain
	wire        hit_vld;
	wire [31:0] hit_dout;
	wire        hit_pop;
	wire        hit_empty;
	wire [7:0]  lane_max;

	// ---------------- stimulus (per-position byte) ----------------
	reg [7:0] stim [0:STIM_BYTES-1];
	initial $readmemh("stimulus.hex", stim);

	// ---------------- golden: per-position reference ----------------
	reg [7:0]  ex_score_all [0:STIM_BYTES-1];
	reg [11:0] ex_x_all     [0:STIM_BYTES-1];
	reg [11:0] ex_y_all     [0:STIM_BYTES-1];
	// ---------------- golden: expected hit list (FIFO order) ----------------
	reg [11:0] ex_pos   [0:STIM_BYTES-1];
	reg [7:0]  ex_score [0:STIM_BYTES-1];
	reg [11:0] ex_x     [0:STIM_BYTES-1];
	reg [11:0] ex_y     [0:STIM_BYTES-1];
	integer ex_cnt, ex_idx;
	integer ef, ep;
	reg [11:0] p; reg [7:0] s; reg [11:0] xx, yy;
	initial begin
		ep = $fopen("expected_pos.txt", "r");
		if (ep == 0) begin $display("ERROR: cannot open expected_pos.txt"); $finish; end
		while (!$feof(ep)) begin
			if ($fscanf(ep, "%d %d %d %d", p, s, xx, yy) == 4) begin
				ex_score_all[p]=s; ex_x_all[p]=xx; ex_y_all[p]=yy;
			end
		end
		$fclose(ep);

		ef = $fopen("hits.txt", "r");
		if (ef == 0) begin $display("ERROR: cannot open hits.txt"); $finish; end
		ex_cnt = 0; ex_idx = 0;
		while (!$feof(ef)) begin
			if ($fscanf(ef, "%d %d %d %d", p, s, xx, yy) == 4) begin
				ex_pos[ex_cnt]=p; ex_score[ex_cnt]=s; ex_x[ex_cnt]=xx; ex_y[ex_cnt]=yy;
				ex_cnt = ex_cnt + 1;
			end
		end
		$fclose(ef);
		$display("INFO: golden expected positions=40 hits=%0d", ex_cnt);
	end

	// ---------------- clock ----------------
	always #5 clk = ~clk;

	// ---------------- DUT ----------------
	sbm_accum_lane #(.CHUNK(CHUNK), .BANK_DEPTH(BANK_DEPTH), .WC(WC),
	                 .LB(LB), .X0(X0), .Y0(Y0)) dut (
		.clk(clk), .rst_n(rst_n), .init(init), .base_addr(base_addr),
		.tp(tp), .thresh(thresh), .zero_en(zero_en), .zero_addr(zero_addr),
		.run_en(run_en), .lane_done(lane_done), .need_req(need_req),
		.req_addr(req_addr), .next_slot(next_slot), .grant(grant),
		.fill_slot(fill_slot), .fill_vld(fill_vld), .fill_data(fill_data),
		.scan_en(scan_en), .hit_vld(hit_vld), .hit_dout(hit_dout),
		.hit_pop(hit_pop), .hit_empty(hit_empty), .lane_max(lane_max));

	// ---------------- upstream byte-stream feeder ----------------
	reg [10:0] stim_ptr;
	integer    dbg_fill_cnt;
	reg        fail;
	integer    cyc, checked;
	initial begin clk=0; rst_n=0; init=0; run_en=0; scan_en=0;
		grant=0; fill_vld=0; fill_slot=0; stim_ptr=0;
		zero_en=0; fail=0; cyc=0; checked=0; dbg_fill_cnt=0; drain_en=0;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			grant<=0; fill_vld<=0; fill_slot<=0; stim_ptr<=0;
		end else begin
			if (need_req && !grant && !fill_vld) begin
				grant      <= 1'b1;
				fill_slot <= next_slot;
			end else if (grant) begin
				grant     <= 1'b0;
				fill_vld  <= 1'b1;
				// fill_slot retains the value latched at the grant cycle
				fill_data <= 256'b0;
				begin: fillb
					integer b;
					for (b = 0; b < 32; b = b + 1)
						fill_data[b*8 +: 8] <= (stim_ptr + b < STIM_BYTES)
							? stim[stim_ptr + b] : 8'h00;
				end
				stim_ptr <= stim_ptr + 32;
			end else begin
				fill_vld <= 1'b0;
			end
		end
	end
	always @(posedge clk) if (fill_vld) dbg_fill_cnt <= dbg_fill_cnt + 1;

	// drain the hit FIFO ONLY during the post-scan drain window. While scan is
	// running drain_en=0 so hit_pop=0 and the DUT writes EVERY hit (its write is
	// gated by !hit_pop, so popping during scan would starve the writes). During
	// the drain window we pop whenever data is available and capture hit_dout on
	// the SAME cycle the pop is asserted (the DUT advances hit_rd in that same
	// cycle, so hit_dout still shows the correct head entry). This captures each
	// entry exactly once, in order, with no drops and no same-cycle RW conflict.
	assign hit_pop = drain_en && hit_vld;

	// ---------------- (1) per-position scan validation (F1 core) ----------------
	// Reads the DUT internal scan evaluation directly (hierarchical). For every
	// evaluated position < cnt_l, score AND coordinate must match the golden.
	// Guarded by F1_SIGNALS: only the fixed DUT exposes ev_*/w_score internals.
	`ifdef F1_SIGNALS
	always @(posedge clk) begin
		if (dut.ev_vld && (dut.ev_pos < dut.cnt_l)) begin
			checked <= checked + 1;
			if (dut.w_score != ex_score_all[dut.ev_pos] ||
			    dut.ev_x   != ex_x_all[dut.ev_pos]     ||
			    dut.ev_y   != ex_y_all[dut.ev_pos]) begin
				$display("FAIL[scan]: pos=%0d  got(score,x,y)=%0d/%0d/%0d  exp=%0d/%0d/%0d",
					dut.ev_pos, dut.w_score, dut.ev_x, dut.ev_y,
					ex_score_all[dut.ev_pos], ex_x_all[dut.ev_pos], ex_y_all[dut.ev_pos]);
				fail <= 1'b1;
			end
		end
	end
	`endif

	// ---------------- (2) FIFO hit-path validation ----------------
	// On the SAME cycle a pop is issued (drain_en && hit_vld) the DUT presents
	// the head entry on hit_dout and advances hit_rd in that same cycle. We
	// compare hit_dout directly here (no 1-cycle capture register, which would
	// otherwise lag by one and emit a spurious "extra hit" on the final cycle).
	// Each entry is compared exactly once, in order.
	always @(posedge clk) begin
		if (drain_en && hit_vld) begin
			if (ex_idx >= ex_cnt) begin
				$display("FAIL[fifo]: extra hit got={%0d,%0d,%0d}",
					hit_dout[31:24], hit_dout[23:12], hit_dout[11:0]);
				fail <= 1'b1;
			end else if (hit_dout != {ex_score[ex_idx], ex_y[ex_idx], ex_x[ex_idx]}) begin
				$display("FAIL[fifo]: pos exp=%0d got(score,y,x)=%0d/%0d/%0d exp=%0d/%0d/%0d",
					ex_pos[ex_idx], hit_dout[31:24], hit_dout[23:12], hit_dout[11:0],
					ex_score[ex_idx], ex_y[ex_idx], ex_x[ex_idx]);
				fail <= 1'b1;
			end
			ex_idx <= ex_idx + 1;
		end
	end

	// ---------------- watchdog ----------------
	always @(posedge clk) cyc <= cyc + 1;

	// ---------------- stimulus sequence ----------------
	initial begin
		#20 rst_n = 1'b1;
		base_addr = 32'd0; tp = TP; thresh = THRESH;
		@(posedge clk); init <= 1'b1; @(posedge clk); init <= 1'b0;
		repeat (3) @(posedge clk);   // let cnt_l settle; lane_done drops to 0

		// ---- accumulation ----
		run_en <= 1'b1;
		fork
			begin: wdog
				#20000;
				$display("FAIL: lane_done never asserted (accumulation stall)");
				$finish;
			end
		join_none
		wait (lane_done); run_en <= 1'b0;
		disable wdog;
		$display("INFO: accumulation done, lane_max=%0d fill_chunks=%0d", lane_max, dbg_fill_cnt);
		@(posedge clk);

		// ---- candidate scan ----
		scan_en <= 1'b1;
		repeat (TP + 8) @(posedge clk);
		scan_en <= 1'b0;
		// ---- drain hit FIFO: pop every cycle data is available ----
		drain_en <= 1'b1;
		repeat (40) @(posedge clk);
		drain_en <= 1'b0;
		$display("INFO: after-scan lane_max=%0d (golden max score=139)", dut.max_r);

		// ---- verdict ----
		#1;
		`ifdef F1_SIGNALS
		if (checked != dut.cnt_l) begin
			$display("FAIL: scan evaluated %0d positions, expected %0d", checked, dut.cnt_l);
			fail <= 1'b1;
		end
		`endif
		#1;
		if (ex_idx != ex_cnt) begin
			$display("FAIL: FIFO hit count got=%0d exp=%0d", ex_idx, ex_cnt);
			fail <= 1'b1;
		end
		#1;
		if (fail) $display("==== RESULT: FAIL ====");
		else      $display("==== RESULT: PASS (%0d FIFO hits matched) ====", ex_cnt);
		#20 $finish;
	end

endmodule
