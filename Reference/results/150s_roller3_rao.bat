@ECHO OFF
REM Date exported, 2026/10/21 12:24:30
REM Default solver: aquasim_2_22_1.exe
if not defined AQUASIM_SOLVER set "AQUASIM_SOLVER=aquasim_2_22_1.exe"
REM Setting variable ok to 1, meaning true. If an error occurs then it is set to 0, meaning false.
SET /A ok=1

:150s_roller3_rao01
REM Normal load [Amp = 0.0, Period = 10.0]
TITLE %CD% \150s_roller3_rao01.txt
%AQUASIM_SOLVER% "150s_roller3_rao01"
REM If the error level is 1 or higher we will print the error and jump to the next run.
if %ERRORLEVEL% GTR 0 CALL :printError
if %ERRORLEVEL% GTR 0 GOTO :okCheck
REM packing files
avspack.exe -join "150s_roller3_rao01.avs" "150s_roller3_rao01.xml"
avspack.exe -join "150s_roller3_rao01PFAT.avs" "150s_roller3_rao01.xml"
REM Deleting AVS files
IF %ERRORLEVEL% EQU 0 del "150s_roller3_rao01.avs"
IF %ERRORLEVEL% EQU 0 del "150s_roller3_rao01PFAT.avs"

REM Create max out file
"150s_roller3_rao_maxout.bat"
:okCheck
if %ok% EQU 0 CALL :ERROR

:OK
EXIT

:ERROR
echo One or more of the runs did not complete.
pause
EXIT

:printError
if %ERRORLEVEL% EQU 1 echo The Aquasim run did not complete. Detached node.
if %ERRORLEVEL% EQU 2 echo The Aquasim run did not complete. Invalid point3.
if %ERRORLEVEL% EQU 3 echo The Aquasim run did not complete. Invalid point3.
if %ERRORLEVEL% EQU 4 echo The Aquasim run did not complete. Please refer to the previous text for more information.
if %ERRORLEVEL% EQU 5 echo The Aquasim run did not complete. Please refer to the previous text for more information.
if %ERRORLEVEL% EQU 6 echo The Aquasim run did not complete. Please refer to the previous text for more information.
if %ERRORLEVEL% EQU 7 echo The Aquasim run did not complete. Demo version.
if %ERRORLEVEL% EQU 8 echo Seed outside boundries
if %ERRORLEVEL% EQU 9 echo Failed intersection with bottom
if %ERRORLEVEL% EQU 10 echo License is expired or invalid
if %ERRORLEVEL% EQU 11 echo NaN detected in RDELTA. Possible unstable analysis.
if %ERRORLEVEL% GTR 11 echo The Aquasim run did not complete.
if %ERRORLEVEL% EQU 9009 echo Failed to find aquasim executable (aquasim_2_22_1.exe) Please see FAQ - AquaEdit cannot locate solver on our website at http://aquasim.no/faq.html
SET /A ok=0

:END