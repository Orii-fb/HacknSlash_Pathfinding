@echo off
setlocal
cd /d "%~dp0"
where msbuild >nul 2>nul
if errorlevel 1 (
 echo Ouvrir le terminal de developpement Visual Studio 2026 ou ouvrir CampusPathEngine.sln.
 pause
 exit /b 1
)
msbuild CampusPathEngine.sln /m:2 /p:Configuration=Debug /p:Platform=x64
if errorlevel 1 exit /b 1
bin\x64\Debug\cpe_tests.exe
if errorlevel 1 exit /b 1
bin\x64\Debug\campus_path_engine.exe
