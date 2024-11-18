@echo off
echo.

IF "%1" == "/?" GOTO help
IF "%1" == "-h" GOTO help
IF "%1" == "--help" GOTO help
IF "%1" == "" GOTO noparams

:checkperm
    net session >nul 2>&1
    if NOT %errorLevel% == 0 (
        echo Rerun as admin
        GOTO end
    )

IF "%1" == "start" GOTO start
IF "%1" == "stop" GOTO stop

GOTO help

:start
  IF "%2" == "" GOTO noparams
  IF "%3" == "" GOTO noparams
  netsh interface portproxy add v4tov4 listenaddress=127.0.0.1 listenport="%3" connectaddress="%2" connectport="%3"
  GOTO end

:stop
  IF "%2" == "" GOTO noparams
  netsh interface portproxy delete v4tov4 listenaddress=127.0.0.1 listenport="%2"
    GOTO end

:noparams
    echo Please give a command
    GOTO help

:help
    echo Usage: adb-client [command]
    echo.
    echo Generic commands:
    echo   start [address] [port] - Sets up forwarding
    echo   stop [port] - Stops forwarding
    GOTO end

:end
    echo.