@echo off
chcp 65001 >nul
echo ====================================
echo    ST-Link 下载脚本
echo ====================================
echo.

set KEIL_PATH=C:\Keil_v5\UV4\UV4.exe
set PROJECT_FILE=Project.uvprojx
set HEX_FILE=Objects\Project.hex

echo [1/3] 检查 HEX 文件是否存在...
if not exist "%HEX_FILE%" (
    echo 警告: 未找到 HEX 文件!
    echo 位置: %HEX_FILE%
    echo.
    echo 是否先编译项目? (Y/N)
    set /p choice=
    if /i "%choice%"=="Y" (
        call build.bat
    ) else (
        echo 取消下载
        pause
        exit /b 1
    )
)
echo ✓ HEX 文件已找到

echo.
echo [2/3] 使用 Keil 下载到芯片...
echo 项目文件: %PROJECT_FILE%
echo.

"%KEIL_PATH%" -f "%PROJECT_FILE%" -o download_log.txt

if %ERRORLEVEL% equ 0 (
    echo.
    echo ====================================
    echo    ✓ 下载成功!
    echo ====================================
) else (
    echo.
    echo ====================================
    echo    ✗ 下载失败! 错误码: %ERRORLEVEL%
    echo ====================================
    echo.
    echo 请检查:
    echo 1. ST-Link 是否连接
    echo 2. 目标板是否上电
    echo 3. Keil 项目配置是否正确
)

echo.
echo [3/3] 完成
pause

