# Print the first output pixels straight from the DUT so they can be compared
# with theory_check.py (an independent NumPy model of the same algorithm).
# usage: vsim -c -debugdb -do "do spot_check.do" work.tb_sbm_alg1_gaussian
# Console (-c) mode has no `breakpoint`; `when` is the batch equivalent, and its
# condition is re-evaluated every delta cycle -- so one pixel can trigger the
# callback twice. Dedupe in Tcl; HDL names resolve directly inside the
# condition, but values must be fetched with `examine` in the action.
set ::n 0
set ::last -1
when {m_axis_tvalid === 1'b1 && total < 14} {
  set ::v [examine -radix unsigned /tb_sbm_alg1_gaussian/m_axis_tdata]
  if {$::n < 10 && $::v != $::last} {
    incr ::n
    echo "THEORY-SPOT \[$::n\] tdata=$::v"
  }
  set ::last $::v
}
run -all
quit -f
