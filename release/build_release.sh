cd ..
make
cd release
ffmpeg -i Tile.png -f rawvideo -pix_fmt rgb565 tile.raw
~/Projects/ESP32/esplay/esplay-base-firmware/tools/mkfw/mkfw OpenTyrian tile.raw 0 16 1048576 app ../build/OpenTyrian.bin
rm OpenTyrian.fw
mv firmware.fw OpenTyrian.fw
