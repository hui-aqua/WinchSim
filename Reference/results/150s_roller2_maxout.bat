@ECHO OFF
REM Creating max out
avspostproc.exe "150s_roller201pfat.avz" -out "150s_roller2_max_out.avs" -mode max
avspack.exe -join "150s_roller2_max_out.avs" "150s_roller2.xml"
ECHO Creating max out file
del "150s_roller2_max_out.avs"