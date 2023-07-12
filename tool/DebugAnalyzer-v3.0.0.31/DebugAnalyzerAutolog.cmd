@echo off
setlocal enabledelayedexpansion

echo (*)Using the standard UAC prompt.

rem ***************************************************************
rem                  Request UAC prompt														
rem ***************************************************************

:: BatchGotAdmin
:-------------------------------------
REM  --> Check for permissions
>nul 2>&1 "%SYSTEMROOT%\system32\cacls.exe" "%SYSTEMROOT%\system32\config\system"

REM --> If error flag set, we do not have admin.
if '%errorlevel%' NEQ '0' (
    echo Requesting administrative privileges...
    goto UACPrompt
) else ( goto gotAdmin )

:UACPrompt
    echo Set UAC = CreateObject^("Shell.Application"^) > "%temp%\getadmin.vbs"
    echo UAC.ShellExecute "%~s0", "", "", "runas", 1 >> "%temp%\getadmin.vbs"

    wscript "%temp%\getadmin.vbs"
    exit /B

:gotAdmin
    if exist "%temp%\getadmin.vbs" ( del "%temp%\getadmin.vbs" )
    pushd "%CD%"
    CD /D "%~dp0"
:--------------------------------------

SETLOCAL ENABLEEXTENSIONS 
set LOGDIR="%PROGRAMDATA%\Realtek"
if not exist %LOGDIR% md %LOGDIR%

@echo enable app dump
set APP_DUMP_PATH="%LOGDIR%\AppDump"
if not exist %APP_DUMP_PATH% md %APP_DUMP_PATH%

@REG ADD "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Windows\Windows Error Reporting\LocalDumps\DebugAnalyzer.exe" /v DumpCount /t  REG_DWORD /d 10 /f
@REG ADD "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Windows\Windows Error Reporting\LocalDumps\DebugAnalyzer.exe" /v DumpType /t  REG_DWORD /d 2 /f
@REG ADD "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Windows\Windows Error Reporting\LocalDumps\DebugAnalyzer.exe" /v CustomDumpFlags /t  REG_DWORD /d 0 /f
@REG ADD "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Windows\Windows Error Reporting\LocalDumps\DebugAnalyzer.exe" /v DumpFolder /t  REG_SZ /d %APP_DUMP_PATH% /f

:end
pause