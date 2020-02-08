Reinstall:
@SET ADB=%LOCALAPPDATA%\Android\Sdk\platform-tools\adb
%ADB% start-server
%ADB% uninstall com.qualcomm.ftcdriverstation
%ADB% install .\doc\apk\FtcDriverStation-release.apk
pause