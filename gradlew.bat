@echo off
setlocal EnableExtensions

rem PMWeather-IV Gradle bootstrap for Windows.
rem Downloads the pinned Gradle distribution on first use, then forwards all arguments.

set "GRADLE_VERSION=8.10.2"
set "GRADLE_SHA256=31c55713e40233a8303827ceb42ca48a47267a0ad4bab9177123121e71524c26"
set "GRADLE_URL=https://services.gradle.org/distributions/gradle-%GRADLE_VERSION%-bin.zip"
set "CACHE_ROOT=%USERPROFILE%\.gradle\pmweather-iv-wrapper"
set "GRADLE_HOME=%CACHE_ROOT%\gradle-%GRADLE_VERSION%"
set "ZIP_PATH=%CACHE_ROOT%\gradle-%GRADLE_VERSION%-bin.zip"

if exist "%GRADLE_HOME%\bin\gradle.bat" goto runGradle

where powershell.exe >NUL 2>&1
if errorlevel 1 (
    echo ERROR: PowerShell is required to download Gradle automatically. 1>&2
    exit /b 1
)

if not exist "%CACHE_ROOT%" mkdir "%CACHE_ROOT%"
if errorlevel 1 (
    echo ERROR: Could not create "%CACHE_ROOT%". 1>&2
    exit /b 1
)

echo Downloading Gradle %GRADLE_VERSION%...
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference = 'Stop';" ^
  "[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12;" ^
  "$zip = $env:ZIP_PATH;" ^
  "$expected = $env:GRADLE_SHA256;" ^
  "if (Test-Path $zip) { $actual = (Get-FileHash -Algorithm SHA256 $zip).Hash.ToLowerInvariant(); if ($actual -ne $expected) { Remove-Item -Force $zip } };" ^
  "if (-not (Test-Path $zip)) { Invoke-WebRequest -UseBasicParsing -Uri $env:GRADLE_URL -OutFile $zip };" ^
  "$actual = (Get-FileHash -Algorithm SHA256 $zip).Hash.ToLowerInvariant();" ^
  "if ($actual -ne $expected) { throw ('Gradle archive checksum mismatch. Expected ' + $expected + ', got ' + $actual) };" ^
  "$gradleInstallDir = $env:GRADLE_HOME;" ^
  "if (Test-Path $gradleInstallDir) { Remove-Item -Recurse -Force $gradleInstallDir };" ^
  "Expand-Archive -LiteralPath $zip -DestinationPath $env:CACHE_ROOT -Force;"
if errorlevel 1 (
    echo ERROR: Gradle download or extraction failed. 1>&2
    exit /b 1
)

:runGradle
call "%GRADLE_HOME%\bin\gradle.bat" %*
set "EXIT_CODE=%ERRORLEVEL%"
endlocal & exit /b %EXIT_CODE%
