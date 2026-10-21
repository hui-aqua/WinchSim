@ECHO OFF
REM Creating max out
avspostproc.exe "150s01pfat.avz" -out "150s_max_out.avs" -mode max
avspack.exe -join "150s_max_out.avs" "150s.xml"
ECHO Creating max out file
del "150s_max_out.avs"