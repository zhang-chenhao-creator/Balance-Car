@echo off
cd /d "%~dp0"
python ".\tools\pid_tuning_helper.pyw"
if errorlevel 1 (
  py ".\tools\pid_tuning_helper.pyw"
)
