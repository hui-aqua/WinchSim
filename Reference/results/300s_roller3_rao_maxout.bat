@ECHO OFF
REM Creating max out
avspostproc.exe "300s_roller3_rao01pfat.avz" -out "300s_roller3_rao_max_out.avs" -mode max
avspack.exe -join "300s_roller3_rao_max_out.avs" "300s_roller3_rao.xml"
ECHO Creating max out file
del "300s_roller3_rao_max_out.avs"