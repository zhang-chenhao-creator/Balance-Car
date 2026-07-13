@echo off
set UV4=D:\Keil_v5\UV4\UV4.exe
set PROJECT=%~dp0USER\MiniBalance.uvprojx

if exist "%UV4%" (
  start "" "%UV4%" "%PROJECT%"
) else (
  start "" "%PROJECT%"
)
