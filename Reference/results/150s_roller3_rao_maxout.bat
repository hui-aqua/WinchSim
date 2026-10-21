@ECHO OFF
REM Creating max out
avspostproc.exe "150s_roller3_rao01pfat.avz" -out "150s_roller3_rao_max_out.avs" -mode max
avspack.exe -join "150s_roller3_rao_max_out.avs" "150s_roller3_rao.xml"
ECHO Creating max out file
del "150s_roller3_rao_max_out.avs"