@echo off
chcp 65001 >nul
echo ====================================
echo    Keil 项目编译脚本
echo ====================================
echo.

set KEIL_PATH=C:\Keil_v5\UV4\UV4.exe
set PROJECT_FILE=Project.uvprojx
set BUILD_LOG=build_log.txt

echo [1/3] 检查 Keil 是否安装...
if not exist "%KEIL_PATH%" (
    echo 错误: 找不到 Keil! 
    echo 请检查路径: %KEIL_PATH%
    pause
    exit /b 1
)
echo ✓ Keil 已找到

echo.
echo [2/3] 开始编译项目...
echo 项目文件: %PROJECT_FILE%
echo 编译日志: %BUILD_LOG%
echo.

"%KEIL_PATH%" -b "%PROJECT_FILE%" -o "%BUILD_LOG%"

if %ERRORLEVEL% equ 0 (
    echo.
    echo ====================================
    echo    ✓ 编译成功!
    echo ====================================
    echo.
    echo 生成文件位于: Objects\
    echo.
) else (
    echo.
    echo ====================================
    echo    ✗ 编译失败! 错误码: %ERRORLEVEL%
    echo ====================================
    echo.
    echo 查看详细日志: %BUILD_LOG%
    echo.
)

echo [3/3] 显示编译日志:
echo ====================================
type "%BUILD_LOG%"
echo ====================================
echo.
pause

