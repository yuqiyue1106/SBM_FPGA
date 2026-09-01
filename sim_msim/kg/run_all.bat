@echo off
rem ============================================================
rem  alg1 Gaussian IP - one-shot ModelSim run
rem  usage:  run_all.bat                default 128x128
rem          run_all.bat 64 48          retarget resolution to 64x48
rem          run_all.bat 128 128 wave   also dump vsim.wlf (first 60us)
rem  Keep this file ASCII-only: cmd.exe is not UTF-8 clean.
rem ============================================================
setlocal
set MSIM=C:\modeltech64_2020.4\win64

set W=%1
set H=%2
if "%W%"=="" set W=128
if "%H%"=="" set H=128

if not exist work "%MSIM%\vlib" work
"%MSIM%\vlog" -sv tb_sbm_alg1_gaussian.v sbm_gauss_h.v sbm_gauss_v.v xpm_memory_sdpram_beh.v
if errorlevel 1 (echo [COMPILE FAILED] & exit /b 1)

if "%3"=="wave" (
  "%MSIM%\vsim" -c -voptargs=+acc -gIMG_W=%W% -gIMG_H=%H% -g/u_dut/IMG_W=%W% -g/u_dut/IMG_H=%H% ^
      -do "log -r /*; run 60us; quit -f" work.tb_sbm_alg1_gaussian
) else (
  "%MSIM%\vsim" -c -gIMG_W=%W% -gIMG_H=%H% -g/u_dut/IMG_W=%W% -g/u_dut/IMG_H=%H% ^
      -do "run -all; quit -f" work.tb_sbm_alg1_gaussian
)

set /a EXP=2*W*H
echo ---------------------------------------------------------
echo resolution %W% x %H%   expected PASS line: "PASS: %EXP% pixels matched"
endlocal
