// ============================================================================
// @file      golden_sbm.c
// @brief     Golden reference for the sbm_accum_lane co-simulation
// @details
//   Generates three artifacts consumed by tb_sbm_accum_lane_cosim.v:
//     - stimulus.hex     : one hex byte per line (per-position feature byte)
//     - expected_pos.txt : ALL positions "pos score x y" (full per-position ref)
//     - hits.txt         : only the EXPECTED hits "pos score x y" (FIFO check)
//
//   Per-position byte: a few ISOLATED high-score positions (kept <= 7 so the
//   8-deep FIFO, which in the RTL accepts only hcnt<7 i.e. 7 entries, does not
//   overflow) plus small distinct non-hit scores so that BOTH score and
//   coordinate alignment are validated for every position via the internal
//   scan signals.
//
//   Coordinate mapping (must match sbm_accum_lane scan logic):
//     global_x = p % WC ,  global_y = p / WC
//   Hit rule (must match DUT): score > THRESH (strict).
//
//   These constants MUST match the Verilog testbench parameters.
// ============================================================================
#include <stdio.h>

#define WC      16
#define N       40
#define THRESH  10

static int is_hit(int p) {
	// every-3rd position capped at 7 hits (p = 3,6,9,12,15,18,21). This density
	// is what originally exposed the F1 scan misalignment, while staying <= 7 so
	// the RTL's 8-deep FIFO (accepts only hcnt<7 => 7 entries) never overflows
	// on the FIXED DUT.
	return (p >= 3 && (p % 3) == 0 && p <= 21);
}
static int byte_of(int p) {
	return is_hit(p) ? (100 + p) : (p % 10);   // hits >> THRESH, others < THRESH
}

int main(void) {
	FILE *f = fopen("stimulus.hex", "w");
	FILE *h = fopen("expected_pos.txt", "w");
	FILE *g = fopen("hits.txt", "w");
	if (!f || !h || !g) { fprintf(stderr, "cannot open output files\n"); return 1; }

	int cnt = 0;
	for (int p = 0; p < N; p++) {
		int sc = byte_of(p);
		int x = p % WC, y = p / WC;
		fprintf(f, "%02x\n", sc & 0xff);
		fprintf(h, "%d %d %d %d\n", p, sc, x, y);
		if (sc > THRESH) {
			fprintf(g, "%d %d %d %d\n", p, sc, x, y);
			cnt++;
		}
	}
	fclose(f); fclose(h); fclose(g);
	printf("golden: N=%d WC=%d THRESH=%d  expected_hits=%d\n", N, WC, THRESH, cnt);
	return 0;
}
