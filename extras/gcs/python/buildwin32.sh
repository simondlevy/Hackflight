pyinstaller --onefile --windowed hackflight.py
mv dist hackflight
mkdir hackflight/media
cp media/*.gif hackflight/media
cp media/hackflight.png hackflight/media
cp media/hackflight.ico hackflight/media
