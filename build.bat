@echo off
setlocal
cd /d "%~dp0"
call gradlew.bat build
set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" (
    echo.
    echo Build failed with exit code %EXIT_CODE%.
    pause
    exit /b %EXIT_CODE%
)
echo.
echo Build complete. The mod JAR is in build\libs\
pause
endlocal
